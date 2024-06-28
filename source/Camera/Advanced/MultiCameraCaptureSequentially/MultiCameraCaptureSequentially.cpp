/*
Capture point clouds with multiple cameras sequentially.
*/

#include <Zivid/Zivid.h>

#include <chrono>
#include <future>
#include <iostream>
#include <mutex>
#include <vector>

namespace
{
    Zivid::Array2D<Zivid::PointXYZColorRGBA> processAndSaveInThread(const Zivid::Frame &frame)
    {
        const auto pointCloud = frame.pointCloud();
        auto data = pointCloud.copyData<Zivid::PointXYZColorRGBA>();

        // This is where you should run your processing

        const auto dataFile = "Frame_" + frame.cameraInfo().serialNumber().value() + ".zdf";
        std::cout << "Saving frame to file: " << dataFile << std::endl;
        frame.save(dataFile);

        return data;
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
            std::cout << "Connecting to camera: " << camera.info().serialNumber().value() << std::endl;
            camera.connect();
        }

        std::vector<Zivid::Frame> frames;

        for(auto &camera : cameras)
        {
            std::cout << "Capturing frame with camera: " << camera.info().serialNumber() << std::endl;
            const auto parameters = Zivid::CaptureAssistant::SuggestSettingsParameters{
                Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency::none,
                Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{ std::chrono::milliseconds{ 800 } }
            };
            const auto settings = Zivid::CaptureAssistant::suggestSettings(camera, parameters);
            const auto frame = camera.capture(settings);
            frames.push_back(frame);
        }

        // This is where the scene can move relative to the cameras

        std::vector<std::future<Zivid::Array2D<Zivid::PointXYZColorRGBA>>> futureData;

        for(auto &frame : frames)
        {
            std::cout << "Starting to process and save (in a separate thread) the frame captured with camera: "
                      << frame.cameraInfo().serialNumber().value() << std::endl;
            futureData.emplace_back(std::async(std::launch::async, processAndSaveInThread, frame));
        }

        std::vector<Zivid::Array2D<Zivid::PointXYZColorRGBA>> allData;

        for(size_t i = 0; i < frames.size(); ++i)
        {
            std::cout << "Waiting for processing and saving to finish for camera "
                      << frames[i].cameraInfo().serialNumber().value() << std::endl;
            const auto data = futureData[i].get();
            allData.push_back(data);
        }

        // This is where all data is available for further processing, e.g., stitching
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
