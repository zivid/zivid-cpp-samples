/*
Capture point clouds with multiple cameras in parallel.
*/

#include <Zivid/Zivid.h>

#include <chrono>
#include <future>
#include <iostream>
#include <mutex>
#include <vector>

namespace
{
    using HighResClock = std::chrono::high_resolution_clock;
    using Duration = std::chrono::nanoseconds;

    struct MeasuredTimes
    {
        Duration captureDuration;
        Duration pointCloudDuration;
        Duration processDuration;
        Duration copyDuration;
        Duration totalDuration;

        MeasuredTimes &operator+=(const MeasuredTimes &t)
        {
            captureDuration += t.captureDuration;
            pointCloudDuration += t.pointCloudDuration;
            processDuration += t.processDuration;
            copyDuration += t.copyDuration;
            totalDuration += t.totalDuration;

            return *this;
        }
        MeasuredTimes &operator/=(float f)
        {
            captureDuration /= f;
            pointCloudDuration /= f;
            processDuration /= f;
            copyDuration /= f;
            totalDuration /= f;

            return *this;
        }
    };

    MeasuredTimes captureInThread(Zivid::Camera &camera, Zivid::Settings &settings /*, std::mutex &m*/)
    {
        // Capturing frame
        const auto beforeCapture = HighResClock::now();

        const auto frame = camera.capture(settings);

        const auto afterCapture = HighResClock::now();

        const auto pointCloud = frame.pointCloud();

        const auto afterPointCloud = HighResClock::now();

        Zivid::Detail::waitUntilProcessingIsComplete(pointCloud);

        const auto afterProcess = HighResClock::now();

        const auto data = pointCloud.copyData<Zivid::PointXYZColorRGBA>();

        const auto afterCopy = HighResClock::now();

        MeasuredTimes times;

        times.captureDuration = afterCapture - beforeCapture;
        times.pointCloudDuration = afterPointCloud - afterCapture;
        times.processDuration = afterProcess - afterPointCloud;
        times.copyDuration = afterCopy - afterProcess;
        times.totalDuration = afterCopy - beforeCapture;

        return times;
    }
    void warmupCaptureInThread(Zivid::Camera &camera, Zivid::Settings &settings)
    {
        const auto frame = camera.capture(settings).pointCloud().copyData<Zivid::PointXYZColorRGBA>();
    }

    template<typename T>
    std::string valueToStringWithPrecision(const T &value, const size_t precision)
    {
        std::ostringstream ss;
        ss << std::setprecision(precision) << std::fixed << value;
        return ss.str();
    }
    std::string formatDuration(const Duration &duration)
    {
        return valueToStringWithPrecision(
                   std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(duration).count(), 3)
               + " ms";
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Finding cameras" << std::endl;
        auto cameras = zivid.cameras();
        std::cout << "Number of cameras found: " << cameras.size() << std::endl;

        for(auto &camera : cameras)
        {
            std::cout << "Connecting to camera : " << camera.info().serialNumber().value() << std::endl;
            camera.connect();
        }

        auto settingsFile = "settingsSlow.yml";
        auto settings = Zivid::Settings(settingsFile);

        std::cout << "Warmup task" << std::endl;
        for(size_t i = 0; i < 5; i++)
        {
            std::vector<std::future<MeasuredTimes>> warmupTimes;
            for(auto &camera : cameras)
            {
                warmupTimes.emplace_back(std::async(std::launch::async, captureInThread, std::ref(camera), std::ref(settings)));
            }
            for (auto &future : warmupTimes)
            {
                future.wait();
            }
        }


        const size_t numFrames3D = 10;

        std::vector<std::vector<MeasuredTimes>> allTimes(numFrames3D, {cameras.size(), MeasuredTimes{}});

        for(size_t i = 0; i < numFrames3D; i++)
        {
            std::vector<std::future<MeasuredTimes>> times;

            for(auto &camera : cameras)
            {
                std::cout << "Starting task with camera: " << camera.info().serialNumber().value() << std::endl;
                times.emplace_back(std::async(std::launch::async,
                                                  captureInThread,
                                                  std::ref(camera),
                                                  std::ref(settings)));
            }

            for(size_t j = 0; j < cameras.size(); ++j)
            {
                std::cout << "Waiting for task of camera " << cameras[j].info().serialNumber().value() << " to finish" << std::endl;
                const auto camTimes = times[j].get();
                allTimes[i][j] = camTimes;
            }
        }
        
        // after this its all single threaded.
        // allTimes[i][j] contains the times for the i-th run of the j-th camera
        // I only calculated the the averages and printed the average of the captureDuration
        // You can generate different statistics (median for example)
        std::cout << "Generating statistics" << std::endl;
        std::vector<MeasuredTimes> averageTimes(cameras.size(), MeasuredTimes{});
        for(size_t i = 0; i < numFrames3D; i++)
        {
            for(size_t j = 0; j < cameras.size(); j++)
            {
                averageTimes[j] += allTimes[i][j];
            }
        }
        for(auto &avg : averageTimes)
        {
            avg /= numFrames3D;
        }

        for (size_t j = 0; j < cameras.size(); j++)
        {
            std::cout << "Average capture time for camera " << cameras[j].info().serialNumber().value() << ": "
                      << formatDuration(averageTimes[j].captureDuration) << " point cloud time: "
                      << formatDuration(averageTimes[j].pointCloudDuration) << " processing time: "
                      << formatDuration(averageTimes[j].processDuration) << " copy time: "
                      << formatDuration(averageTimes[j].copyDuration) << " total time: "
                      << formatDuration(averageTimes[j].totalDuration) << std::endl;
        }

    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }
}
