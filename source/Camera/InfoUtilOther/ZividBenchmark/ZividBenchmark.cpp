/*
Zividbenchmarks is a sample that will test the average speed of different operations on your computer. It will provide
the mean and median for connects, disconnects, single imaging, HDR and filtering.
*/

#include <Zivid/Zivid.h>

#include <algorithm>
#include <iostream>
#include <numeric>

namespace
{
    const int printWidth = 56;

    using HighResClock = std::chrono::high_resolution_clock;
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
    std::string makeSettingList(const Zivid::Settings &settings)
    {
        std::string settingList = "{ ";
        for(size_t i = 0; i < settings.acquisitions().size(); i++)
        {
            settingList += settings.acquisitions().at(i).get<Target>().toString();
            if(i + 1 != settings.acquisitions().size())
            {
                settingList += ", ";
            }
        }
        settingList += " }";
        return settingList;
    }

    std::string makefilterList(const Zivid::Settings &settings)
    {
        if(settings.processing().filters().smoothing().gaussian().isEnabled().value())
        {
            std::string gaussianString;
            gaussianString = std::string{ "Gaussian (Sigma = " }
                             + settings.processing().filters().smoothing().gaussian().sigma().toString() + " )";

            if(settings.processing().filters().reflection().removal().isEnabled().value())
            {
                return "{ " + gaussianString + ", Reflection }";
            }
            return "{ " + gaussianString + " }";
        }
        if(settings.processing().filters().reflection().removal().isEnabled().value())
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

    void printFormated(const std::vector<std::string> &stringList)
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

    void printCapture3DHeader(const size_t numFrames, const Zivid::Settings &settings)
    {
        const auto filterList = makefilterList(settings);
        printHeaderLine("Capturing ", numFrames, " 3D frames:");
        std::cout << "  Exposure Time = " << makeSettingList<Zivid::Settings::Acquisition::ExposureTime>(settings)
                  << std::endl;
        std::cout << "  Aperture = " << makeSettingList<Zivid::Settings::Acquisition::Aperture>(settings) << std::endl;
        if(!filterList.empty())
        {
            std::cout << "  Filters = " << filterList << std::endl;
        }
    }

    void printAssistedCapture3DHeader(const size_t numFrames)
    {
        printHeaderLine("Running assisted capture ", numFrames, " times:");
    }

    void printCapture2DHeader(const size_t numFrames, const Zivid::Settings2D &settings)
    {
        printHeaderLine("Capturing ", numFrames, " 2D frames:");
        std::cout << "  exposure Time = { " << settings.acquisitions().at(0).exposureTime() << " }" << std::endl;
    }

    void printSaveHeader(const size_t numFrames)
    {
        printHeaderLine("Saving point cloud ", numFrames, " times each (be patient):");
    }

    void printResultLine(const std::string &name, const Duration &durationMedian, const Duration &durationMean)
    {
        printFormated({ name, formatDuration(durationMedian), formatDuration(durationMean) });
    }

    void printResults(const std::vector<std::string> &names, const std::vector<Duration> &durations)
    {
        printSecondarySeparationLine();
        printFormated({ "  Time:", "Median", "Mean" });
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
        printResults({ "  3D image acquisition time:", "  Point cloud processing time:", "  Total 3D capture time:" },
                     durations);
    }

    void printAssistedCapture3DResults(const std::vector<Duration> &durations)
    {
        printResults({ "  Suggest settings time:" }, durations);
    }

    void printNegligableFilters()
    {
        const std::string negligable = "negligible";
        printFormated({ "  Noise", negligable, negligable });
        printFormated({ "  Outlier", negligable, negligable });
    }

    void printFilterResults(const std::vector<Duration> &durations)
    {
        printPrimarySeparationLine();
        std::cout << "Filter processing time:" << std::endl;
        printResults({ "  Gaussian:", "  Reflection:", "  Gaussian and Reflection:" }, durations);
        printSecondarySeparationLine();
        printNegligableFilters();
    }

    void printCapture2DResults(const std::vector<Duration> &durations)
    {
        printResults({ "  Total 2D capture time:" }, durations);
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
        if(cameras.size() != 1)
        {
            throw std::runtime_error("At least one camera needs to be connected");
        }
        printZividInfo(cameras.at(0), zivid);
        return cameras.at(0);
    }

    std::chrono::microseconds getMinExposureTime(const std::string &modelName)
    {
        if(modelName.substr(0, 14) == "Zivid One Plus")
        {
            return std::chrono::microseconds{ 6500 }; // Min for Zivid One Plus
        }
        return std::chrono::microseconds{ 1677 }; // Min for Zivid Two
    }

    Zivid::Settings makeSettings(const std::vector<double> &apertures,
                                 const std::vector<std::chrono::microseconds> &exposureTimes,
                                 const bool enableGaussian,
                                 const bool enableReflection)
    {
        if(apertures.size() != exposureTimes.size())
        {
            throw std::runtime_error("Unequal input vector size");
        }

        Zivid::Settings settings{ Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled{ enableGaussian },
                                  Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{ 1.5 },
                                  Zivid::Settings::Processing::Filters::Noise::Removal::Enabled{ true },
                                  Zivid::Settings::Processing::Filters::Outlier::Removal::Enabled{ true },
                                  Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled{
                                      enableReflection } };
        for(size_t i = 0; i < apertures.size(); ++i)
        {
            const auto acquisitionSettings = Zivid::Settings::Acquisition{
                Zivid::Settings::Acquisition::Aperture{ apertures.at(i) },
                Zivid::Settings::Acquisition::ExposureTime{ exposureTimes.at(i) },
                Zivid::Settings::Acquisition::Brightness{ 1.0 } // Using above 1.0 may cause thermal throttling
            };
            settings.acquisitions().emplaceBack(acquisitionSettings);
        }

        return settings;
    }

    void benchmarkConnect(Zivid::Camera &camera, const size_t numConnects)
    {
        printConnectHeader(numConnects);

        std::vector<Duration> connectDurations;
        std::vector<Duration> disconnectDurations;
        std::vector<Duration> allDurations;

        for(size_t i = 0; i < numConnects; i++)
        {
            const auto beforeConnect = HighResClock::now();
            camera.connect();
            const auto afterConnect = HighResClock::now();
            camera.disconnect();
            const auto afterDisconnect = HighResClock::now();

            connectDurations.push_back(afterConnect - beforeConnect);
            disconnectDurations.push_back(afterDisconnect - afterConnect);
        }

        allDurations.push_back(computeMedianDuration(connectDurations));
        allDurations.push_back(computeAverageDuration(connectDurations));
        allDurations.push_back(computeMedianDuration(disconnectDurations));
        allDurations.push_back(computeAverageDuration(disconnectDurations));

        printConnectResults(allDurations);
    }

    std::vector<Duration> benchmarkCapture3D(Zivid::Camera &camera,
                                             const Zivid::Settings &settings,
                                             const size_t numFrames)
    {
        printCapture3DHeader(numFrames, settings);

        for(size_t i = 0; i < 5; i++) // Warmup frames
        {
            const auto data = camera.capture(settings).pointCloud().copyData<Zivid::PointXYZColorRGBA>();
        }

        std::vector<Duration> captureDurations;
        std::vector<Duration> processDurations;
        std::vector<Duration> totalDurations;
        std::vector<Duration> allDurations;

        for(size_t i = 0; i < numFrames; i++)
        {
            const auto beforeCapture = HighResClock::now();
            const auto frame = camera.capture(settings);
            const auto afterCapture = HighResClock::now();
            const auto pointCloud = frame.pointCloud();
            const auto data = pointCloud.copyData<Zivid::PointXYZColorRGBA>();
            const auto afterProcess = HighResClock::now();

            captureDurations.push_back(afterCapture - beforeCapture);
            processDurations.push_back(afterProcess - afterCapture);
            totalDurations.push_back(afterProcess - beforeCapture);
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

    void benchmarkAssistedCapture3D(Zivid::Camera &camera, const size_t numFrames)
    {
        printAssistedCapture3DHeader(numFrames);

        const Zivid::CaptureAssistant::SuggestSettingsParameters suggestSettingsParameters{
            Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency::none,
            Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{ std::chrono::milliseconds{ 1200 } }
        };

        for(size_t i = 0; i < 5; i++) // Warmup
        {
            const auto settings{ Zivid::CaptureAssistant::suggestSettings(camera, suggestSettingsParameters) };
        }

        std::vector<Duration> suggestSettingsDurations;

        for(size_t i = 0; i < numFrames; i++)
        {
            const auto beforeSuggestSettings = HighResClock::now();
            const auto settings{ Zivid::CaptureAssistant::suggestSettings(camera, suggestSettingsParameters) };
            const auto afterSuggestSettings = HighResClock::now();

            suggestSettingsDurations.push_back(afterSuggestSettings - beforeSuggestSettings);
        }

        std::vector<Duration> allDurations;
        allDurations.push_back(computeMedianDuration(suggestSettingsDurations));
        allDurations.push_back(computeAverageDuration(suggestSettingsDurations));

        printAssistedCapture3DResults(allDurations);
    }

    std::tuple<Duration, Duration> benchmarkFilterProcessing(const std::vector<Duration> &captureDuration,
                                                             const std::vector<Duration> &captureDurationFilter)
    {
        return std::make_tuple((computeMedianDuration(captureDurationFilter) - computeMedianDuration(captureDuration)),
                               computeAverageDuration(captureDurationFilter) - computeAverageDuration(captureDuration));
    }

    void benchmarkCapture3DAndFilters(Zivid::Camera &camera,
                                      const std::vector<double> &apertures,
                                      const std::vector<std::chrono::microseconds> &exposureTimes,
                                      const size_t numFrames3D)
    {
        std::vector<std::string> subtestName{
            "Without filters", "With Gaussian filter", "With Reflection filter", "With Gaussian and Reflection filter"
        };

        printSubtestHeader(subtestName.at(0));

        const std::vector<Duration> captureDurationWithoutFilter =
            benchmarkCapture3D(camera, makeSettings(apertures, exposureTimes, false, false), numFrames3D);

        const std::vector<bool> gaussian{ true, false, true };
        const std::vector<bool> reflection{ false, true, true };

        std::vector<Duration> filterProcessingDurations;
        for(size_t i = 0; i < gaussian.size(); i++)
        {
            printSubtestHeader(subtestName.at(i + 1));

            const std::vector<Duration> captureDurationWithFilter =
                benchmarkCapture3D(camera,
                                   makeSettings(apertures, exposureTimes, gaussian.at(i), reflection.at(i)),
                                   numFrames3D);

            const auto meanAndAverageFilterDurations =
                benchmarkFilterProcessing(captureDurationWithoutFilter, captureDurationWithFilter);

            filterProcessingDurations.push_back(std::get<0>(meanAndAverageFilterDurations));
            filterProcessingDurations.push_back(std::get<1>(meanAndAverageFilterDurations));
        }
        printFilterResults(filterProcessingDurations);
    }

    Zivid::Settings2D makeSettings2D(const std::chrono::microseconds exposureTime)
    {
        Zivid::Settings2D settings{ Zivid::Settings2D::Acquisitions{
            Zivid::Settings2D::Acquisition{ Zivid::Settings2D::Acquisition::ExposureTime(exposureTime) } } };
        return settings;
    }

    void benchmarkCapture2D(Zivid::Camera &camera, const Zivid::Settings2D &settings, const size_t numFrames)
    {
        printCapture2DHeader(numFrames, settings);

        for(size_t i = 0; i < 5; i++) // Warmup frames
        {
            camera.capture(settings);
        }

        std::vector<Duration> captureDurations;
        std::vector<Duration> allDurations;

        for(size_t i = 0; i < numFrames; i++)
        {
            const auto beforeCapture = HighResClock::now();
            const auto frame2D = camera.capture(settings);
            const auto afterCapture = HighResClock::now();

            captureDurations.push_back(afterCapture - beforeCapture);
        }
        allDurations.push_back(computeMedianDuration(captureDurations));
        allDurations.push_back(computeAverageDuration(captureDurations));

        printCapture2DResults(allDurations);
    }

    void benchmarkSave(Zivid::Camera &camera, const size_t numFrames)
    {
        printSaveHeader(numFrames);

        const auto frame =
            camera.capture(Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} } });
        frame.pointCloud();

        std::vector<Duration> allDurations;
        std::vector<std::string> dataFiles{ "Zivid3D.zdf", "Zivid3D.ply", "Zivid3D.pcd", "Zivid3D.xyz" };
        for(const auto &dataFile : dataFiles)
        {
            std::vector<Duration> durationsPerFormat;
            for(size_t j = 0; j < numFrames; j++)
            {
                const auto beforeSave = HighResClock::now();
                frame.save(dataFile);
                const auto afterSave = HighResClock::now();

                durationsPerFormat.push_back(afterSave - beforeSave);
            }

            allDurations.push_back(computeMedianDuration(durationsPerFormat));
            allDurations.push_back(computeAverageDuration(durationsPerFormat));
        }
        printSaveResults(allDurations);
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        auto camera = getFirstCamera(zivid);

        const size_t numConnects = 10;
        const size_t numFrames3D = 20;
        const size_t numFrames2D = 50;
        const size_t numFramesSave = 10;

        const std::chrono::microseconds exposureTime = getMinExposureTime(camera.info().modelName().toString());
        const std::vector<std::chrono::microseconds> oneExposureTime{ exposureTime };
        const std::vector<std::chrono::microseconds> twoExposureTimes{ exposureTime, exposureTime };
        const std::vector<std::chrono::microseconds> threeExposureTimes{ exposureTime, exposureTime, exposureTime };

        const std::vector<double> oneAperture{ 5.66 };
        const std::vector<double> twoApertures{ 8.0, 4.0 };
        const std::vector<double> threeApertures{ 11.31, 5.66, 2.83 };

        printHeader("TEST 1: Connect/Disconnect");
        benchmarkConnect(camera, numConnects);
        camera.connect();
        printHeader("TEST 2: Assisted Capture");
        benchmarkAssistedCapture3D(camera, numFrames3D);
        printHeader("TEST 3: One Acquisition Capture");
        benchmarkCapture3DAndFilters(camera, oneAperture, oneExposureTime, numFrames3D);
        printHeader("TEST 4: Two Acquisitions (HDR) Capture");
        benchmarkCapture3D(camera, makeSettings(twoApertures, twoExposureTimes, false, false), numFrames3D);
        printHeader("TEST 5: Three Acquisitions (HDR) Capture");
        benchmarkCapture3DAndFilters(camera, threeApertures, threeExposureTimes, numFrames3D);
        printHeader("TEST 6: 2D Capture");
        benchmarkCapture2D(camera, makeSettings2D(exposureTime), numFrames2D);
        printHeader("TEST 7: Save");
        benchmarkSave(camera, numFramesSave);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }
}