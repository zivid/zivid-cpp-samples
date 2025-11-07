/*
Zividbenchmark is a sample that will test the average speed of different operations on your computer.
It will provide the mean and median for connects, disconnects, single imaging, HDR and filtering.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.
*/

#include <Zivid/Zivid.h>

#include <clipp.h>

#include <algorithm>
#include <future>
#include <iostream>
#include <numeric>
#include <thread>

namespace
{
    const int printWidth = 56;

    using SteadyClock = std::chrono::steady_clock;
    using Duration = std::chrono::nanoseconds;

    Duration computeAverageDuration(const std::vector<Duration> &durations)
    {
        return std::accumulate(durations.begin(), durations.end(), Duration{ 0 }) / durations.size();
    }

    Duration computeMedianDuration(std::vector<Duration> durations)
    {
        std::sort(durations.begin(), durations.end());
        if(durations.size() % 2 == 0)
        {
            return (durations.at(durations.size() / 2 - 1) + durations.at(durations.size() / 2)) / 2;
        }

        return durations.at(durations.size() / 2);
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

    template<typename Target>
    std::string makeSettingList3D(const Zivid::Settings &settings2D3D)
    {
        std::string settingList = "{ ";
        for(size_t i = 0; i < settings2D3D.acquisitions().size(); i++)
        {
            settingList += settings2D3D.acquisitions().at(i).get<Target>().toString();
            if(i + 1 != settings2D3D.acquisitions().size())
            {
                settingList += ", ";
            }
        }
        settingList += " }";
        return settingList;
    }

    std::string makeFilterList(const Zivid::Settings &settings2D3D)
    {
        if(settings2D3D.processing().filters().smoothing().gaussian().isEnabled().value())
        {
            std::string gaussianString;
            gaussianString = std::string{ "Gaussian (Sigma = " }
                             + settings2D3D.processing().filters().smoothing().gaussian().sigma().toString() + " )";

            if(settings2D3D.processing().filters().reflection().removal().isEnabled().value())
            {
                return "{ " + gaussianString + ", Reflection }";
            }
            return "{ " + gaussianString + " }";
        }
        if(settings2D3D.processing().filters().reflection().removal().isEnabled().value())
        {
            return "{ Reflection }";
        }
        return {};
    }

    void printSeparationLine(const char &separator, const std::string &followingString)
    {
        std::cout << std::left << std::setfill(separator) << std::setw(printWidth) << followingString << std::endl;
    }

    void printPrimarySeparationLine()
    {
        printSeparationLine('=', "");
    }

    void printSecondarySeparationLine()
    {
        printSeparationLine('-', "  ");
    }

    void printCentered(const std::string &text)
    {
        constexpr size_t columns{ printWidth };
        std::cout << std::string((columns - text.size()) / 2, ' ') << text << std::endl;
    }

    void printFormatted(const std::vector<std::string> &stringList)
    {
        std::cout << std::left << std::setfill(' ') << std::setw(32) << stringList.at(0) << std::setw(13)
                  << stringList.at(1) << stringList.at(2) << std::endl;
    }

    void printHeader(const std::string &firstString)
    {
        printPrimarySeparationLine();
        std::cout << std::endl;
        printPrimarySeparationLine();
        std::cout << firstString << std::endl;
    }

    void printHeaderLine(const std::string &firstString, size_t num, const std::string &secondString)
    {
        printPrimarySeparationLine();
        std::cout << firstString << num << secondString << std::endl;
    }

    void printConnectHeader(const size_t numConnects)
    {
        printHeaderLine("Connecting and disconnecting ", numConnects, " times each (be patient):");
    }

    void printSubtestHeader(const std::string &subtest)
    {
        printPrimarySeparationLine();
        std::cout << subtest << std::endl;
    }

    void printCapture3DHeader(const size_t numFrames, const Zivid::Settings &settings2D3D)
    {
        const auto filterList = makeFilterList(settings2D3D);
        printHeaderLine("Capturing ", numFrames, " 3D frames:");
        std::cout << "  Exposure Time = " << makeSettingList3D<Zivid::Settings::Acquisition::ExposureTime>(settings2D3D)
                  << std::endl;
        std::cout << "  Aperture = " << makeSettingList3D<Zivid::Settings::Acquisition::Aperture>(settings2D3D)
                  << std::endl;
        if(!filterList.empty())
        {
            std::cout << "  Filters = " << filterList << std::endl;
        }
    }

    void printCapture2DHeader(const size_t numFrames, const Zivid::Settings2D &settings2D)
    {
        printHeaderLine("Capturing ", numFrames, " 2D frames:");
        std::cout << "  exposure Time = { " << settings2D.acquisitions().at(0).exposureTime() << " }" << std::endl;
    }

    void printCopyHeader(const size_t numCopies)
    {
        printHeaderLine("Copying various data ", numCopies, " times each (be patient):");
    }

    void printSaveHeader(const size_t numFrames)
    {
        printHeaderLine("Saving point cloud ", numFrames, " times each (be patient):");
    }

    void printResultLine(const std::string &name, const Duration &durationMedian, const Duration &durationMean)
    {
        printFormatted({ name, formatDuration(durationMedian), formatDuration(durationMean) });
    }

    void printResults(const std::vector<std::string> &names, const std::vector<Duration> &durations)
    {
        printSecondarySeparationLine();
        printFormatted({ "  Time:", "Median", "Mean" });
        for(size_t i = 0; i < names.size(); i++)
        {
            printResultLine(names.at(i), durations.at(i + i), durations.at(i + i + 1));
        }
    }

    void printConnectResults(const std::vector<Duration> &durations)
    {
        printResults({ "  Connect:", "  Disconnect:" }, durations);
    }

    void printCapture3DResults(const std::vector<Duration> &durations)
    {
        printResults(
            { "  3D image acquisition time:", "  Point cloud processing time:", "  Total 3D capture time:" },
            durations);
    }

    void printCapture3Dincl2DResults(const std::vector<Duration> &durations)
    {
        printResults(
            { "  2D+3D image acquisition time:", "  Point cloud processing time:", "  Total 3D capture time:" },
            durations);
    }

    void printCapture2D3DResults(const std::vector<Duration> &durations)
    {
        printResults(
            { "  2D image acquisition time:",
              "  3D image acquisition time:",
              "  2D + 3D acquisition time:",
              "  2D image processing time:",
              "  Point cloud processing time:",
              "  Total 2D + 3D capture time:" },
            durations);
    }

    void printCapture3D2DResults(const std::vector<Duration> &durations)
    {
        printResults(
            { "  3D image acquisition time:",
              "  2D image acquisition time:",
              "  3D + 2D acquisition time:",
              "  Point cloud processing time:",
              "  2D image processing time:",
              "  Total 3D + 2D capture time:" },
            durations);
    }

    void printNegligibleFilters()
    {
        const std::string negligible = "negligible";
        printFormatted({ "  Noise", negligible, negligible });
        printFormatted({ "  Outlier", negligible, negligible });
    }

    void printFilterResults(const std::vector<Duration> &durations)
    {
        printPrimarySeparationLine();
        std::cout << "Filter processing time:" << std::endl;
        printResults({ "  Gaussian:", "  Reflection:", "  Gaussian and Reflection:" }, durations);
        printSecondarySeparationLine();
        printNegligibleFilters();
    }

    void printCapture2DResults(const std::vector<Duration> &durations)
    {
        printResults({ "  Total 2D capture time:" }, durations);
    }

    void printCopyDataResults(std::array<std::vector<Duration>, 9> &durations, const size_t numCopies)
    {
        printCopyHeader(numCopies);
        printResults({ "   copyData<PointXYZ>: " }, durations[0]);
        printResults({ "  copyData<PointXYZW>: " }, durations[1]);
        printResults({ "     copyData<PointZ>: " }, durations[2]);
        printResults({ "  copyData<ColorRGBA_SRGB>: " }, durations[3]);
        printResults({ "        copyData<SNR>: " }, durations[4]);
        printResults({ "  copyData<ColorRGBA_SRGB>: " }, durations[5]);
        printResults({ "  copyData<ColorBGRA_SRGB>: " }, durations[6]);
        printResults({ "      copyImageRGBA_SRGB(): " }, durations[7]);
        printResults({ "  copyData<NormalXYZ>: " }, durations[8]);
    }

    void printSaveResults(const std::vector<Duration> &durations)
    {
        printResults({ "  Save ZDF:", "  Save PLY:", "  Save PCD:", "  Save XYZ:" }, durations);
    }

    void printZividInfo(const Zivid::Camera &camera, const Zivid::Application &zivid)
    {
        std::cout << "API: " << Zivid::Version::coreLibraryVersion() << std::endl;
        std::cout << "OS: " << OS_NAME << std::endl;
        std::cout << "Camera: " << camera << std::endl;
        std::cout << "Compute device: " << zivid.computeDevice() << std::endl;
        printPrimarySeparationLine();
        printCentered("Starting Zivid Benchmark");
    }

    Zivid::Camera getFirstCamera(Zivid::Application &zivid)
    {
        const auto cameras = zivid.cameras();
        for(const auto &camera : cameras)
        {
            if(camera.state().status() == Zivid::CameraState::Status::available)
            {
                std::cout << "Available camera: " << camera.info().serialNumber() << std::endl;
                printZividInfo(camera, zivid);
                return camera;
            }
            std::cout << "Camera " << camera.info().serialNumber() << "is not available. "
                      << "Camera status: " << camera.state().status() << std::endl;
        }
        throw std::runtime_error("At least one camera needs to be available");
    }

    std::chrono::microseconds getExposureTimeForAllModels()
    {
        return std::chrono::microseconds{ 1677 };
    }

    double getApertureForAllModels()
    {
        return 3.0;
    }

    bool doesNotSupportColorWithoutProjector(const Zivid::CameraInfo::Model model)
    {
        return (
            model.value() == Zivid::CameraInfo::Model::ValueType::zivid2PlusMR130
            || model.value() == Zivid::CameraInfo::Model::ValueType::zivid2PlusLR110
            || model.value() == Zivid::CameraInfo::Model::ValueType::zivid2PlusMR60
            || model.value() == Zivid::CameraInfo::Model::ValueType::zivid3XL250);
    }

    Zivid::Settings makeSettings3D(
        const std::vector<double> &apertures,
        const std::vector<std::chrono::microseconds> &exposureTimes,
        const bool enableGaussian,
        const bool enableReflection)
    {
        if(apertures.size() != exposureTimes.size())
        {
            throw std::runtime_error("Unequal input vector size");
        }

        Zivid::Settings settings3D{
            Zivid::Settings::Engine::phase,
            Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled{ enableGaussian },
            Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{ 1.5 },
            Zivid::Settings::Processing::Filters::Noise::Removal::Enabled{ true },
            Zivid::Settings::Processing::Filters::Outlier::Removal::Enabled{ true },
            Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled{ enableReflection },
            Zivid::Settings::Sampling::Color::disabled,
        };
        for(size_t i = 0; i < apertures.size(); ++i)
        {
            settings3D.acquisitions().emplaceBack(
                Zivid::Settings::Acquisition{
                    Zivid::Settings::Acquisition::ExposureTime{ exposureTimes.at(i) },
                    Zivid::Settings::Acquisition::Aperture{ apertures.at(i) },
                    Zivid::Settings::Acquisition::Brightness{ 1.0 },
                    Zivid::Settings::Acquisition::Gain{ 1.0 },
                });
        }

        return settings3D;
    }

    Zivid::Settings2D makeSettings2D(Zivid::Camera &camera, const std::chrono::microseconds exposureTime)
    {
        const auto colorSampling = doesNotSupportColorWithoutProjector(camera.info().model())
                                       ? Zivid::Settings2D::Sampling::Color::grayscale
                                       : Zivid::Settings2D::Sampling::Color::rgb;

        Zivid::Settings2D settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{
                                          Zivid::Settings2D::Acquisition::ExposureTime(exposureTime) } },
                                      colorSampling };
        return settings2D;
    }

    Zivid::Settings makeSettings(
        Zivid::Camera &camera,
        const std::vector<double> &apertures,
        const std::vector<std::chrono::microseconds> &exposureTimes,
        const std::chrono::microseconds &exposureTime2D,
        const bool enableGaussian,
        const bool enableReflection)
    {
        return makeSettings3D(apertures, exposureTimes, enableGaussian, enableReflection)
            .copyWith(
                Zivid::Settings::Color{ makeSettings2D(camera, exposureTime2D) },
                // Unset legacy color settings, so that any color settings are used from `Zivid::Settings::Color`.
                Zivid::Settings::Processing::Color{},
                Zivid::Settings::Sampling::Color{});
    }

    template<typename FrameT>
    struct FrameAndCaptureTime
    {
        FrameT frame;
        Duration captureTime;
    };

    FrameAndCaptureTime<Zivid::Frame2D> captureAndMeasure2D(Zivid::Camera &camera, const Zivid::Settings2D &settings)
    {
        const auto before = SteadyClock::now();
        const auto frame = camera.capture2D(settings);
        const auto after = SteadyClock::now();
        return { frame, (after - before) };
    }

    FrameAndCaptureTime<Zivid::Frame> captureAndMeasure3D(Zivid::Camera &camera, const Zivid::Settings &settings)
    {
        const auto before = SteadyClock::now();
        const auto frame = camera.capture3D(settings);
        const auto after = SteadyClock::now();
        return { frame, (after - before) };
    }

    template<typename T>
    Duration useFrame(const T &frame);

    template<>
    Duration useFrame<Zivid::Frame>(const Zivid::Frame &frame)
    {
        const auto before = SteadyClock::now();
        const auto pointCloud = frame.pointCloud();
        const auto data = pointCloud.copyData<Zivid::PointXYZColorRGBA_SRGB>();
        const auto after = SteadyClock::now();
        return (after - before);
    }

    template<>
    Duration useFrame<Zivid::Frame2D>(const Zivid::Frame2D &frame2D)
    {
        const auto before = SteadyClock::now();
        const auto image = frame2D.imageRGBA_SRGB();
        const auto after = SteadyClock::now();
        return (after - before);
    }

    void benchmarkCapture2DFirstAndThen3D(
        Zivid::Camera &camera,
        const Zivid::Settings &settings2D3D,
        const size_t numFrames,
        const bool withProjectorFor2D)
    {
        auto settings2D = settings2D3D.color().value();
        if(!withProjectorFor2D && doesNotSupportColorWithoutProjector(camera.info().model())
           && settings2D.sampling().color().value() != Zivid::Settings2D::Sampling::Color::ValueType::grayscale)
        {
            std::cout << "Skipping test: " << camera.info().model() << " does not support color without projector"
                      << std::endl;
            return;
        }
        for(auto &acquisition : settings2D.acquisitions())
        {
            acquisition.set(Zivid::Settings2D::Acquisition::Brightness{ withProjectorFor2D ? 1.0 : 0.0 });
        }

        for(size_t i = 0; i < 5; i++) // setup time
        {
            camera.capture2D(settings2D);
            camera.capture3D(settings2D3D);
        }

        std::vector<Duration> captureDurations2D;
        std::vector<Duration> captureDurations;
        std::vector<Duration> processDurations2D;
        std::vector<Duration> processDurations;
        std::vector<Duration> totalCaptureDurations;
        std::vector<Duration> totalDurations;
        std::vector<Duration> allDurations;

        for(size_t i = 0; i < numFrames; i++)
        {
            const auto before = SteadyClock::now();
            const auto frame2dAndCaptureTime = captureAndMeasure2D(camera, settings2D);
            std::future<Duration> userThread =
                std::async(std::launch::async, useFrame<Zivid::Frame2D>, std::ref(frame2dAndCaptureTime.frame));
            const auto frameAndCaptureTime = captureAndMeasure3D(camera, settings2D3D);
            const auto processTime = useFrame(frameAndCaptureTime.frame);
            const auto processTime2D = userThread.get();
            const auto after = SteadyClock::now();

            captureDurations2D.push_back(frame2dAndCaptureTime.captureTime);
            captureDurations.push_back(frameAndCaptureTime.captureTime);
            processDurations2D.push_back(processTime2D);
            processDurations.push_back(processTime);
            totalCaptureDurations.push_back(frame2dAndCaptureTime.captureTime + frameAndCaptureTime.captureTime);
            totalDurations.push_back(after - before);

            // Adding a small delay to simulate a real-life capture cycle where there is some pause between captures, fex. when moving the robot
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        allDurations.push_back(computeMedianDuration(captureDurations2D));
        allDurations.push_back(computeAverageDuration(captureDurations2D));
        allDurations.push_back(computeMedianDuration(captureDurations));
        allDurations.push_back(computeAverageDuration(captureDurations));
        allDurations.push_back(computeMedianDuration(totalCaptureDurations));
        allDurations.push_back(computeAverageDuration(totalCaptureDurations));
        allDurations.push_back(computeMedianDuration(processDurations2D));
        allDurations.push_back(computeAverageDuration(processDurations2D));
        allDurations.push_back(computeMedianDuration(processDurations));
        allDurations.push_back(computeAverageDuration(processDurations));
        allDurations.push_back(computeMedianDuration(totalDurations));
        allDurations.push_back(computeAverageDuration(totalDurations));

        printCapture2D3DResults(allDurations);
    }

    void benchmarkCapture3DFirstAndThen2D(
        Zivid::Camera &camera,
        const Zivid::Settings &settings2D3D,
        const size_t numFrames,
        const bool withProjectorFor2D)
    {
        auto settings2D = settings2D3D.color().value();
        if(!withProjectorFor2D && doesNotSupportColorWithoutProjector(camera.info().model())
           && settings2D.sampling().color().value() != Zivid::Settings2D::Sampling::Color::ValueType::grayscale)
        {
            std::cout << "Skipping test: " << camera.info().model() << " does not support color without projector"
                      << std::endl;
            return;
        }
        for(auto &acquisition : settings2D.acquisitions())
        {
            acquisition.set(Zivid::Settings2D::Acquisition::Brightness{ withProjectorFor2D ? 1.0 : 0.0 });
        }

        for(size_t i = 0; i < 5; i++) // setup time
        {
            camera.capture3D(settings2D3D);
            camera.capture2D(settings2D);
        }

        std::vector<Duration> captureDurations2D;
        std::vector<Duration> captureDurations;
        std::vector<Duration> processDurations2D;
        std::vector<Duration> processDurations;
        std::vector<Duration> totalCaptureDurations;
        std::vector<Duration> totalDurations;
        std::vector<Duration> allDurations;

        for(size_t i = 0; i < numFrames; i++)
        {
            const auto before = SteadyClock::now();
            const auto frameAndCaptureTime = captureAndMeasure3D(camera, settings2D3D);
            std::future<Duration> userThread =
                std::async(std::launch::async, useFrame<Zivid::Frame>, std::ref(frameAndCaptureTime.frame));
            const auto frame2dAndCaptureTime = captureAndMeasure2D(camera, settings2D);
            const auto processTime2D = useFrame(frame2dAndCaptureTime.frame);
            const auto processTime = userThread.get();
            const auto after = SteadyClock::now();

            captureDurations2D.push_back(frame2dAndCaptureTime.captureTime);
            captureDurations.push_back(frameAndCaptureTime.captureTime);
            processDurations2D.push_back(processTime2D);
            processDurations.push_back(processTime);
            totalCaptureDurations.push_back(frame2dAndCaptureTime.captureTime + frameAndCaptureTime.captureTime);
            totalDurations.push_back(after - before);

            // Adding a small delay to simulate a real-life capture cycle where there is some pause between captures, fex. when moving the robot
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        allDurations.push_back(computeMedianDuration(captureDurations));
        allDurations.push_back(computeAverageDuration(captureDurations));
        allDurations.push_back(computeMedianDuration(captureDurations2D));
        allDurations.push_back(computeAverageDuration(captureDurations2D));
        allDurations.push_back(computeMedianDuration(totalCaptureDurations));
        allDurations.push_back(computeAverageDuration(totalCaptureDurations));
        allDurations.push_back(computeMedianDuration(processDurations));
        allDurations.push_back(computeAverageDuration(processDurations));
        allDurations.push_back(computeMedianDuration(processDurations2D));
        allDurations.push_back(computeAverageDuration(processDurations2D));
        allDurations.push_back(computeMedianDuration(totalDurations));
        allDurations.push_back(computeAverageDuration(totalDurations));

        printCapture3D2DResults(allDurations);
    }

    void benchmarkCapture3DIncluding2D(
        Zivid::Camera &camera,
        const Zivid::Settings &baseSettings2D3D,
        const size_t numFrames,
        bool withProjectorFor2D)
    {
        auto settings2D3D = Zivid::Settings{ baseSettings2D3D };
        if(!withProjectorFor2D && doesNotSupportColorWithoutProjector(camera.info().model())
           && settings2D3D.color().value().sampling().color().value()
                  != Zivid::Settings2D::Sampling::Color::ValueType::grayscale)
        {
            std::cout << "Skipping test: " << camera.info().model() << " does not support color without projector"
                      << std::endl;
            return;
        }
        for(auto &acquisition : settings2D3D.color().value().acquisitions())
        {
            acquisition.set(Zivid::Settings2D::Acquisition::Brightness{ withProjectorFor2D ? 1.0 : 0.0 });
        }

        for(size_t i = 0; i < 5; i++) // setup time
        {
            camera.capture2D3D(settings2D3D);
        }

        std::vector<Duration> captureDurations;
        std::vector<Duration> processDurations;
        std::vector<Duration> totalDurations;
        std::vector<Duration> allDurations;

        for(size_t i = 0; i < numFrames; i++)
        {
            const auto start = SteadyClock::now();
            const auto frame = camera.capture2D3D(settings2D3D);
            const auto afterCapture = SteadyClock::now();
            const auto processTime = useFrame(frame);
            const auto afterAll = SteadyClock::now();

            captureDurations.push_back(afterCapture - start);
            processDurations.push_back(processTime);
            totalDurations.push_back(afterAll - start);

            // Adding a small delay to simulate a real-life capture cycle where there is some pause between captures, fex. when moving the robot
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        allDurations.push_back(computeMedianDuration(captureDurations));
        allDurations.push_back(computeAverageDuration(captureDurations));
        allDurations.push_back(computeMedianDuration(processDurations));
        allDurations.push_back(computeAverageDuration(processDurations));
        allDurations.push_back(computeMedianDuration(totalDurations));
        allDurations.push_back(computeAverageDuration(totalDurations));

        printCapture3Dincl2DResults(allDurations);
    }

    void benchmarkConnect(Zivid::Camera &camera, const size_t numConnects)
    {
        printConnectHeader(numConnects);

        std::vector<Duration> connectDurations;
        std::vector<Duration> disconnectDurations;
        std::vector<Duration> allDurations;

        for(size_t i = 0; i < numConnects; i++)
        {
            const auto beforeConnect = SteadyClock::now();
            camera.connect();
            const auto afterConnect = SteadyClock::now();
            camera.disconnect();
            const auto afterDisconnect = SteadyClock::now();

            connectDurations.push_back(afterConnect - beforeConnect);
            disconnectDurations.push_back(afterDisconnect - afterConnect);
        }

        allDurations.push_back(computeMedianDuration(connectDurations));
        allDurations.push_back(computeAverageDuration(connectDurations));
        allDurations.push_back(computeMedianDuration(disconnectDurations));
        allDurations.push_back(computeAverageDuration(disconnectDurations));

        printConnectResults(allDurations);
    }

    std::vector<Duration>
    benchmarkCapture2D3D(Zivid::Camera &camera, const Zivid::Settings &settings2D3D, const size_t numFrames)
    {
        printCapture2DHeader(numFrames, settings2D3D.color().value());
        printCapture3DHeader(numFrames, settings2D3D);

        for(size_t i = 0; i < 5; i++) // setup time
        {
            camera.capture2D3D(settings2D3D);
        }

        std::vector<Duration> captureDurations2D;
        std::vector<Duration> captureDurations;
        std::vector<Duration> processDurations2D;
        std::vector<Duration> processDurations;
        std::vector<Duration> totalCaptureDurations;
        std::vector<Duration> totalDurations;
        std::vector<Duration> allDurations;

        for(size_t i = 0; i < numFrames; i++)
        {
            const auto beforeCapture2D = SteadyClock::now();
            const auto frame2D = camera.capture2D(settings2D3D.color().value());
            const auto afterCapture2D = SteadyClock::now();
            const auto frame = camera.capture3D(settings2D3D);
            const auto afterCapture = SteadyClock::now();
            const auto image = frame2D.imageRGBA_SRGB();
            const auto afterProcess2D = SteadyClock::now();
            const auto pointCloud = frame.pointCloud();
            const auto data = pointCloud.copyData<Zivid::PointXYZColorRGBA_SRGB>();
            const auto afterProcess = SteadyClock::now();

            captureDurations2D.push_back(afterCapture2D - beforeCapture2D);
            captureDurations.push_back(afterCapture - afterCapture2D);
            processDurations2D.push_back(afterProcess2D - afterCapture);
            processDurations.push_back(afterProcess - afterProcess2D);
            totalCaptureDurations.push_back(afterCapture - beforeCapture2D);
            totalDurations.push_back(afterProcess - beforeCapture2D);

            // Adding a small delay to simulate a real-life capture cycle where there is some pause between captures, fex. when moving the robot
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        allDurations.push_back(computeMedianDuration(captureDurations2D));
        allDurations.push_back(computeAverageDuration(captureDurations2D));
        allDurations.push_back(computeMedianDuration(captureDurations));
        allDurations.push_back(computeAverageDuration(captureDurations));
        allDurations.push_back(computeMedianDuration(totalCaptureDurations));
        allDurations.push_back(computeAverageDuration(totalCaptureDurations));
        allDurations.push_back(computeMedianDuration(processDurations2D));
        allDurations.push_back(computeAverageDuration(processDurations2D));
        allDurations.push_back(computeMedianDuration(processDurations));
        allDurations.push_back(computeAverageDuration(processDurations));
        allDurations.push_back(computeMedianDuration(totalDurations));
        allDurations.push_back(computeAverageDuration(totalDurations));

        printCapture2D3DResults(allDurations);

        return totalDurations;
    }

    std::vector<Duration>
    benchmarkCapture3D2D(Zivid::Camera &camera, const Zivid::Settings &settings2D3D, const size_t numFrames)
    {
        printCapture3DHeader(numFrames, settings2D3D);
        printCapture2DHeader(numFrames, settings2D3D.color().value());

        for(size_t i = 0; i < 5; i++) // setup time
        {
            camera.capture3D(settings2D3D);
            camera.capture2D(settings2D3D.color().value());
        }

        std::vector<Duration> captureDurations;
        std::vector<Duration> captureDurations2D;
        std::vector<Duration> processDurations;
        std::vector<Duration> processDurations2D;
        std::vector<Duration> totalCaptureDurations;
        std::vector<Duration> totalDurations;
        std::vector<Duration> allDurations;

        for(size_t i = 0; i < numFrames; i++)
        {
            const auto beforeCapture = SteadyClock::now();
            const auto frame = camera.capture3D(settings2D3D);
            const auto afterCapture = SteadyClock::now();
            const auto frame2D = camera.capture2D(settings2D3D.color().value());
            const auto afterCapture2D = SteadyClock::now();
            const auto pointCloud = frame.pointCloud();
            const auto data = pointCloud.copyData<Zivid::PointXYZColorRGBA_SRGB>();
            const auto afterProcess = SteadyClock::now();
            const auto image = frame2D.imageRGBA_SRGB();
            const auto afterProcess2D = SteadyClock::now();

            captureDurations.push_back(afterCapture - beforeCapture);
            captureDurations2D.push_back(afterCapture2D - afterCapture);
            processDurations.push_back(afterProcess - afterCapture2D);
            processDurations2D.push_back(afterProcess2D - afterProcess);
            totalCaptureDurations.push_back(afterCapture2D - beforeCapture);
            totalDurations.push_back(afterProcess2D - beforeCapture);

            // Adding a small delay to simulate a real-life capture cycle where there is some pause between captures, fex. when moving the robot
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        allDurations.push_back(computeMedianDuration(captureDurations));
        allDurations.push_back(computeAverageDuration(captureDurations));
        allDurations.push_back(computeMedianDuration(captureDurations2D));
        allDurations.push_back(computeAverageDuration(captureDurations2D));
        allDurations.push_back(computeMedianDuration(totalCaptureDurations));
        allDurations.push_back(computeAverageDuration(totalCaptureDurations));
        allDurations.push_back(computeMedianDuration(processDurations));
        allDurations.push_back(computeAverageDuration(processDurations));
        allDurations.push_back(computeMedianDuration(processDurations2D));
        allDurations.push_back(computeAverageDuration(processDurations2D));
        allDurations.push_back(computeMedianDuration(totalDurations));
        allDurations.push_back(computeAverageDuration(totalDurations));

        printCapture3D2DResults(allDurations);

        return totalDurations;
    }

    std::vector<Duration>
    benchmarkCapture3D(Zivid::Camera &camera, const Zivid::Settings &settings2D3D, const size_t numFrames)
    {
        printCapture3DHeader(numFrames, settings2D3D);

        for(size_t i = 0; i < 5; i++) // setup time
        {
            const auto data = camera.capture3D(settings2D3D).pointCloud().copyData<Zivid::PointXYZColorRGBA_SRGB>();
        }

        std::vector<Duration> captureDurations;
        std::vector<Duration> processDurations;
        std::vector<Duration> totalDurations;
        std::vector<Duration> allDurations;

        for(size_t i = 0; i < numFrames; i++)
        {
            const auto beforeCapture = SteadyClock::now();
            const auto frame = camera.capture3D(settings2D3D);
            const auto afterCapture = SteadyClock::now();
            const auto pointCloud = frame.pointCloud();
            const auto data = pointCloud.copyData<Zivid::PointXYZColorRGBA_SRGB>();
            const auto afterProcess = SteadyClock::now();

            captureDurations.push_back(afterCapture - beforeCapture);
            processDurations.push_back(afterProcess - afterCapture);
            totalDurations.push_back(afterProcess - beforeCapture);

            // Adding a small delay to simulate a real-life capture cycle where there is some pause between captures, fex. when moving the robot
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

        allDurations.push_back(computeMedianDuration(captureDurations));
        allDurations.push_back(computeAverageDuration(captureDurations));
        allDurations.push_back(computeMedianDuration(processDurations));
        allDurations.push_back(computeAverageDuration(processDurations));
        allDurations.push_back(computeMedianDuration(totalDurations));
        allDurations.push_back(computeAverageDuration(totalDurations));

        printCapture3DResults(allDurations);

        return totalDurations;
    }

    std::tuple<Duration, Duration> benchmarkFilterProcessing(
        const std::vector<Duration> &captureDuration,
        const std::vector<Duration> &captureDurationFilter)
    {
        return std::make_tuple(
            (computeMedianDuration(captureDurationFilter) - computeMedianDuration(captureDuration)),
            computeAverageDuration(captureDurationFilter) - computeAverageDuration(captureDuration));
    }

    void benchmarkCapture3DAndFilters(
        Zivid::Camera &camera,
        const std::vector<double> &apertures,
        const std::vector<std::chrono::microseconds> &exposureTimes,
        const size_t numFrames3D)
    {
        std::vector<std::string> subtestName{
            "Without filters", "With Gaussian filter", "With Reflection filter", "With Gaussian and Reflection filter"
        };

        printSubtestHeader(subtestName.at(0));

        const std::vector<Duration> captureDurationWithoutFilter =
            benchmarkCapture3D(camera, makeSettings3D(apertures, exposureTimes, false, false), numFrames3D);

        const std::vector<bool> gaussian{ true, false, true };
        const std::vector<bool> reflection{ false, true, true };

        std::vector<Duration> filterProcessingDurations;
        for(size_t i = 0; i < gaussian.size(); i++)
        {
            printSubtestHeader(subtestName.at(i + 1));

            const std::vector<Duration> captureDurationWithFilter = benchmarkCapture3D(
                camera, makeSettings3D(apertures, exposureTimes, gaussian.at(i), reflection.at(i)), numFrames3D);

            const auto meanAndAverageFilterDurations =
                benchmarkFilterProcessing(captureDurationWithoutFilter, captureDurationWithFilter);

            filterProcessingDurations.push_back(std::get<0>(meanAndAverageFilterDurations));
            filterProcessingDurations.push_back(std::get<1>(meanAndAverageFilterDurations));
        }
        printFilterResults(filterProcessingDurations);
    }

    void benchmarkCapture2D(Zivid::Camera &camera, const Zivid::Settings2D &settings2D, const size_t numFrames)
    {
        printCapture2DHeader(numFrames, settings2D);

        for(size_t i = 0; i < 5; i++) // setup time
        {
            camera.capture2D(settings2D);
        }

        std::vector<Duration> captureDurations;
        std::vector<Duration> allDurations;

        for(size_t i = 0; i < numFrames; i++)
        {
            const auto beforeCapture = SteadyClock::now();
            // The 2D capture API returns after the 2D image is available in CPU memory.
            // All the acquisition, processing, and copying happen inside this function call.
            const auto frame2D = camera.capture2D(settings2D);
            const auto afterCapture = SteadyClock::now();

            captureDurations.push_back(afterCapture - beforeCapture);

            // Adding a small delay to simulate a real-life capture cycle where there is some pause between captures, fex. when moving the robot
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        allDurations.push_back(computeMedianDuration(captureDurations));
        allDurations.push_back(computeAverageDuration(captureDurations));

        printCapture2DResults(allDurations);
    }

    template<typename DataType>
    Duration copyDataTime(Zivid::Frame &frame)
    {
        auto pointCloud = frame.pointCloud();
        const auto beforeCopyData = SteadyClock::now();
        pointCloud.copyData<DataType>();
        const auto afterCopyData = SteadyClock::now();
        return afterCopyData - beforeCopyData;
    }

    Duration copyDataTime(Zivid::Frame2D &frame2D)
    {
        const auto beforeCopyData = SteadyClock::now();
        // The method to get the image from the Frame2D object returns the image right away.
        // The image object holds a handle to the image data in CPU memory.
        frame2D.imageRGBA_SRGB();
        const auto afterCopyData = SteadyClock::now();
        return afterCopyData - beforeCopyData;
    }

    void benchmarkCopyData(Zivid::Camera &camera, const std::chrono::microseconds exposureTime, const size_t numCopies)
    {
        constexpr int numData = 9;
        std::array<std::vector<Duration>, numData> copyDataDurations;
        std::array<std::vector<Duration>, numData> allDurations;

        const std::vector<std::chrono::microseconds> twoExposureTimes{ exposureTime, exposureTime };
        const std::vector<double> twoApertures{ 3.0, 3.0 };
        const auto settings2D3D = makeSettings(camera, twoApertures, twoExposureTimes, exposureTime, false, false);
        auto warmupFrame = camera.capture2D3D(settings2D3D);
        auto warmupFrame2D = warmupFrame.frame2D().value();

        copyDataTime<Zivid::PointXYZ>(warmupFrame);
        copyDataTime<Zivid::PointXYZW>(warmupFrame);
        copyDataTime<Zivid::PointZ>(warmupFrame);
        copyDataTime<Zivid::ColorRGBA_SRGB>(warmupFrame);
        copyDataTime<Zivid::SNR>(warmupFrame);
        copyDataTime<Zivid::PointXYZColorRGBA_SRGB>(warmupFrame);
        copyDataTime<Zivid::PointXYZColorBGRA_SRGB>(warmupFrame);
        copyDataTime(warmupFrame2D);
        copyDataTime<Zivid::NormalXYZ>(warmupFrame);

        for(size_t i = 0; i < numCopies; i++)
        {
            auto frame = camera.capture2D3D(settings2D3D);
            auto frame2D = frame.frame2D().value();

            copyDataDurations[0].push_back(copyDataTime<Zivid::PointXYZ>(frame));
            copyDataDurations[1].push_back(copyDataTime<Zivid::PointXYZW>(frame));
            copyDataDurations[2].push_back(copyDataTime<Zivid::PointZ>(frame));
            copyDataDurations[3].push_back(copyDataTime<Zivid::ColorRGBA_SRGB>(frame));
            copyDataDurations[4].push_back(copyDataTime<Zivid::SNR>(frame));
            copyDataDurations[5].push_back(copyDataTime<Zivid::PointXYZColorRGBA_SRGB>(frame));
            copyDataDurations[6].push_back(copyDataTime<Zivid::PointXYZColorBGRA_SRGB>(frame));
            copyDataDurations[7].push_back(copyDataTime(frame2D));
            copyDataDurations[8].push_back(copyDataTime<Zivid::NormalXYZ>(frame));
        }

        for(size_t i = 0; i < numData; i++)
        {
            allDurations[i].push_back(computeMedianDuration(copyDataDurations[i]));
            allDurations[i].push_back(computeAverageDuration(copyDataDurations[i]));
        }

        printCopyDataResults(allDurations, numCopies);
    }

    void benchmarkSave(Zivid::Camera &camera, const size_t numFrames)
    {
        printSaveHeader(numFrames);

        const auto frame =
            camera.capture3D(Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} } });
        frame.pointCloud();

        std::vector<Duration> allDurations;
        std::vector<std::string> dataFiles{ "Zivid3D.zdf", "Zivid3D.ply", "Zivid3D.pcd", "Zivid3D.xyz" };
        for(const auto &dataFile : dataFiles)
        {
            std::vector<Duration> durationsPerFormat;
            for(size_t j = 0; j < numFrames; j++)
            {
                const auto beforeSave = SteadyClock::now();
                frame.save(dataFile);
                const auto afterSave = SteadyClock::now();

                durationsPerFormat.push_back(afterSave - beforeSave);
            }

            allDurations.push_back(computeMedianDuration(durationsPerFormat));
            allDurations.push_back(computeAverageDuration(durationsPerFormat));
        }
        printSaveResults(allDurations);
    }
} // namespace

int main(int argc, char **argv)
{
    try
    {
        bool settingsFromYML = false;
        bool settings2DFromYML = false;
        std::string settings2DFile;
        std::string settingsFile;

        auto cli =
            ((clipp::option("--settings-2d").set(settings2DFromYML, true)
              & clipp::value("settings-2d-file", settings2DFile)),
             (clipp::option("--settings").set(settingsFromYML, true) & clipp::value("settings-file", settingsFile)));

        if(!parse(argc, argv, cli))
        {
            auto fmt = clipp::doc_formatting{}.alternatives_min_split_size(1).surround_labels("\"", "\"");
            std::cout << clipp::usage_lines(cli, "ZividBenchmark", fmt) << std::endl;
            throw std::runtime_error{ "Invalid usage" };
        }

        Zivid::Application zivid;

        auto camera = getFirstCamera(zivid);

        const size_t numConnects = 10;
        const size_t numFrames3D = 20;
        const size_t numFrames2D = 50;
        const size_t numFramesSave = 10;
        const size_t numCopies = 10;

        const std::chrono::microseconds exposureTime = getExposureTimeForAllModels();
        const std::vector<std::chrono::microseconds> oneExposureTime{ exposureTime };
        const std::vector<std::chrono::microseconds> twoExposureTimes{ exposureTime, exposureTime };
        const std::vector<std::chrono::microseconds> threeExposureTimes{ exposureTime, exposureTime, exposureTime };

        const double aperture = getApertureForAllModels();
        const std::vector<double> oneAperture{ aperture };
        const std::vector<double> twoApertures{ aperture, aperture };
        const std::vector<double> threeApertures{ aperture, aperture, aperture };

        auto settings2D3D = settingsFromYML
                                ? Zivid::Settings(settingsFile)
                                : makeSettings(camera, twoApertures, twoExposureTimes, exposureTime, false, false);
        if(settings2DFromYML)
        {
            settings2D3D = settings2D3D.copyWith(Zivid::Settings::Color{ Zivid::Settings2D(settings2DFile) });
        }
        else if(!settings2D3D.color().hasValue())
        {
            settings2D3D = settings2D3D.copyWith(Zivid::Settings::Color{ makeSettings2D(camera, exposureTime) });
        }

        printHeader("TEST: Connect/Disconnect");
        benchmarkConnect(camera, numConnects);

        camera.connect();

        if(settingsFromYML)
        {
            printHeader("TEST: 3D Capture");
            benchmarkCapture3D(camera, settings2D3D, numFrames3D);
        }
        else
        {
            printHeader("TEST: One Acquisition Capture");
            benchmarkCapture3DAndFilters(camera, oneAperture, oneExposureTime, numFrames3D);
            printHeader("TEST: Two Acquisitions (HDR) Capture");
            benchmarkCapture3D(camera, makeSettings3D(twoApertures, twoExposureTimes, false, false), numFrames3D);
            printHeader("TEST: Three Acquisitions (HDR) Capture");
            benchmarkCapture3DAndFilters(camera, threeApertures, threeExposureTimes, numFrames3D);
        }
        printHeader("TEST: 2D Capture");
        benchmarkCapture2D(camera, settings2D3D.color().value(), numFrames2D);

        printHeader("TEST: 3D + 2D Capture");
        benchmarkCapture3D2D(camera, settings2D3D, numFrames3D);
        printHeader("TEST: 2D + 3D Capture");
        benchmarkCapture2D3D(camera, settings2D3D, numFrames3D);

        printHeader("TEST: Copy Data");
        benchmarkCopyData(camera, exposureTime, numCopies);
        printHeader("TEST: Save");
        benchmarkSave(camera, numFramesSave);

        printHeader("TEST: 2D without projector followed by 3D");
        benchmarkCapture2DFirstAndThen3D(camera, settings2D3D, numFrames3D, false);
        printHeader("TEST: 2D with projector followed by 3D");
        benchmarkCapture2DFirstAndThen3D(camera, settings2D3D, numFrames3D, true);

        printHeader("TEST: 3D followed by 2D without projector");
        benchmarkCapture3DFirstAndThen2D(camera, settings2D3D, numFrames3D, false);
        printHeader("TEST: 3D followed by 2D with projector");
        benchmarkCapture3DFirstAndThen2D(camera, settings2D3D, numFrames3D, true);

        printHeader("TEST: 3D including 2D without projector");
        benchmarkCapture3DIncluding2D(camera, settings2D3D, numFrames3D, false);
        printHeader("TEST: 3D including 2D with projector");
        benchmarkCapture3DIncluding2D(camera, settings2D3D, numFrames3D, true);
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
