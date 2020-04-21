#include <Zivid/Zivid.h>
#include <algorithm>
#include <fstream>
#include <iostream>

namespace
{
    const int PRINT_WIDTH = 56;
    const char *RAW_FILE_NAME = "zivid_benchmark_raw.txt";

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
        else
        {
            return durations.at(durations.size() / 2);
        }
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
    std::string makeSettingList(const std::vector<Zivid::Settings> &settingsVector)
    {
        std::string settingList = "{ ";
        for(size_t i = 0; i < settingsVector.size(); i++)
        {
            settingList += settingsVector.at(i).get<Target>().toString();
            if(i + 1 != settingsVector.size()) settingList += ", ";
        }
        settingList += " }";
        return settingList;
    }

    std::string makefilterList(const std::vector<Zivid::Settings> &settingsVector)
    {
        if(settingsVector.at(0).filters().gaussian().isEnabled().value())
        {
            if(settingsVector.at(0).filters().reflection().isEnabled().value())
            {
                return "{ gaussian, reflection }";
            }
            return "{ gaussian }";
        }
        if(settingsVector.at(0).filters().reflection().isEnabled().value())
        {
            return "{ reflection }";
        }
        return {};
    }

    void createEmptyOutputFile()
    {
        std::ofstream rawFile(RAW_FILE_NAME, std::ios::out);
    }

    void addToRawFile(const std::string &string)
    {
        std::ofstream rawFile(RAW_FILE_NAME, std::ios::out | std::ios::app);
        rawFile << string << std::endl;
    }

    void printProgress(const int iteration, const int total)
    {
        std::cout << "..." << iteration << "/" << total << "   "
                  << "\r";
    }

    void printSeparationLine(const char &separator, const std::string &followingString)
    {
        std::cout << std::left << std::setfill(separator) << std::setw(PRINT_WIDTH) << followingString << std::endl;
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
        constexpr size_t columns{ PRINT_WIDTH };
        std::cout << std::string((columns - text.size()) / 2, ' ') << text << std::endl;
    }

    void printFormatted(const std::vector<std::string> &stringList)
    {
        std::cout << std::left << std::setfill(' ') << std::setw(32) << stringList.at(0) << std::setw(13)
                  << stringList.at(1) << stringList.at(2) << std::endl;
    }

    void printHeaderLine(const size_t num, std::initializer_list<std::string> stringList)
    {
        printPrimarySeparationLine();
        std::string outputString;
        for(auto iterator = stringList.begin(); iterator != stringList.end(); ++iterator)
        {
            if(std::distance(stringList.begin(), iterator) == 1)
            {
                outputString += std::to_string(num) + (*iterator);
            }
            else
            {
                outputString += (*iterator);
            }
        }
        std::cout << outputString << std::endl;
    }

    void printConnectHeader(const size_t numConnects)
    {
        printHeaderLine(numConnects, { "Connecting and disconnecting ", " times each (be patient):" });
        addToRawFile("Connecting and disconnecting ");
    }

    void printCapture3DHeader(const size_t numFrames, const std::vector<Zivid::Settings> &settingsVector)
    {
        const auto filterList = makefilterList(settingsVector);
        printHeaderLine(numFrames, { "Capturing ", " 3D frames:" });
        addToRawFile("Capturing 3D frames:");
        std::cout << "  exposure time = " << makeSettingList<Zivid::Settings::ExposureTime>(settingsVector)
                  << std::endl;
        addToRawFile("  exposure time = " + makeSettingList<Zivid::Settings::ExposureTime>(settingsVector));
        std::cout << "  iris settings = " << makeSettingList<Zivid::Settings::Iris>(settingsVector) << std::endl;
        addToRawFile("  iris settings = " + makeSettingList<Zivid::Settings::Iris>(settingsVector));
        if(!filterList.empty())
        {
            std::cout << "  filters = " << filterList << std::endl;
            addToRawFile("  filters = " + filterList);
        }
    }

    void printAssistedCapture3DHeader(const size_t numFrames)
    {
        printHeaderLine(numFrames, { "Running assisted capture ", " times:" });
        addToRawFile("Running assisted capture:");
    }

    void printCapture2DHeader(const size_t numFrames, const Zivid::Settings2D &settings)
    {
        printHeaderLine(numFrames, { "Capturing ", " 2D frames:" });
        addToRawFile("Capturing 2D frames:");
        std::cout << "  exposure time = { " << settings.exposureTime().toString() << " }" << std::endl;
        addToRawFile("  exposure time = " + settings.exposureTime().toString());
    }

    void printSaveHeader(const size_t numFrames)
    {
        printHeaderLine(numFrames, { "Saving point cloud ", " times each (be patient):" });
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
        printFormatted({ "  Contrast", negligable, negligable });
        printFormatted({ "  Outlier", negligable, negligable });
        printFormatted({ "  Saturated", negligable, negligable });
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

    void printZividInfo(Zivid::Camera camera)
    {
        std::cout << "API: " << Zivid::Version::libraryVersion() << std::endl;
        addToRawFile("API: " + Zivid::Version::libraryVersion());
        std::cout << "OS: " << OS_NAME << std::endl;
        addToRawFile("OS: " + std::string(OS_NAME));
        std::cout << "Camera: " << camera << std::endl;
        addToRawFile("Camera: " + camera.toString());
        std::cout << "Compute device: " << camera.computeDevice() << std::endl;
        addToRawFile("Compute device: " + camera.computeDevice().toString());
        printPrimarySeparationLine();
        printCentered("Starting Zivid Benchmark");
    }

    Zivid::Camera getFirstCamera(Zivid::Application &zivid)
    {
        const auto cameras = zivid.cameras();
        if(cameras.size() != 1) throw std::runtime_error("At least one camera needs to be connected");
        printZividInfo(cameras.at(0));
        return cameras.at(0);
    }

    size_t getMinExposureTime(const std::string &modelName)
    {
        if(modelName.substr(0, 14) == "Zivid One Plus")
            return 6500; // Min for Zivid One Plus
        else
            return 8333; // Min for Zivid One
    }

    std::vector<Zivid::Settings> makeSettingsVector(const std::vector<unsigned int> &irises,
                                                    const std::vector<size_t> &exposureTimes,
                                                    const bool enableGaussian,
                                                    const bool enableReflection)
    {
        std::vector<Zivid::Settings> settingsVector(irises.size());

        Zivid::Settings::Filters filters;
        filters.set(Zivid::Settings::Filters::Contrast::Enabled{ true });
        filters.set(Zivid::Settings::Filters::Outlier::Enabled{ true });
        filters.set(Zivid::Settings::Filters::Saturated::Enabled{ true });
        filters.set(Zivid::Settings::Filters::Gaussian::Enabled{ enableGaussian });
        filters.set(Zivid::Settings::Filters::Reflection::Enabled{ enableReflection });

        Zivid::Settings settings;
        for(size_t i = 0; i < irises.size(); ++i)
        {
            settings.set(Zivid::Settings::Iris{ irises.at(i) });
            settings.set(Zivid::Settings::Filters{ filters });
            settings.set(Zivid::Settings::ExposureTime{ std::chrono::microseconds{ exposureTimes.at(i) } });
            settingsVector.at(i) = settings;
        }
        return settingsVector;
    }

    void benchmarkConnect(Zivid::Camera &camera, const size_t numConnects)
    {
        printConnectHeader(numConnects);

        std::vector<Duration> connectDurations;
        std::vector<Duration> disconnectDurations;
        std::vector<Duration> allDurations;

        for(size_t i = 0; i < numConnects; i++)
        {
            printProgress(i + 1, numConnects);
            const auto beforeConnect = HighResClock::now();
            camera.connect();
            const auto afterConnect = HighResClock::now();
            camera.disconnect();
            const auto afterDisconnect = HighResClock::now();

            connectDurations.push_back(afterConnect - beforeConnect);
            addToRawFile(std::string("3D connect duration: ") + formatDuration(afterConnect - beforeConnect));
            disconnectDurations.push_back(afterDisconnect - afterConnect);
            addToRawFile(std::string("3D disconnect duration: ") + formatDuration(afterDisconnect - afterConnect));
        }

        allDurations.push_back(computeMedianDuration(connectDurations));
        allDurations.push_back(computeAverageDuration(connectDurations));
        allDurations.push_back(computeMedianDuration(disconnectDurations));
        allDurations.push_back(computeAverageDuration(disconnectDurations));

        printConnectResults(allDurations);
    }

    std::vector<Duration> benchmarkCapture3D(Zivid::Camera &camera,
                                             const std::vector<Zivid::Settings> &settingsVector,
                                             const size_t numFrames)
    {
        printCapture3DHeader(numFrames, settingsVector);

        for(size_t i = 0; i < 5; i++) // Warmup frames
        {
            printProgress(i + 1, 5);
            const auto frame = Zivid::HDR::capture(camera, settingsVector);
            frame.getPointCloud();
        }

        std::vector<Duration> captureDurations;
        std::vector<Duration> processDurations;
        std::vector<Duration> totalDurations;
        std::vector<Duration> allDurations;

        for(size_t i = 0; i < numFrames; i++)
        {
            printProgress(i + 1, numFrames);
            const auto beforeCapture = HighResClock::now();
            const auto frame = Zivid::HDR::capture(camera, settingsVector);
            const auto afterCapture = HighResClock::now();
            frame.getPointCloud();
            const auto afterProcess = HighResClock::now();

            captureDurations.push_back(afterCapture - beforeCapture);
            addToRawFile(std::string("capture duration: ") + formatDuration(afterCapture - beforeCapture));
            processDurations.push_back(afterProcess - afterCapture);
            addToRawFile(std::string("process duration: ") + formatDuration(afterProcess - afterCapture));
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

        Zivid::CaptureAssistant::SuggestSettingsParameters suggestSettingsParameters(
            std::chrono::milliseconds{ 1200 }, Zivid::CaptureAssistant::AmbientLightFrequency::none);

        for(size_t i = 0; i < 5; i++) // Warmup
        {
            printProgress(i, 5);
            const auto settingsVector{ Zivid::CaptureAssistant::suggestSettings(camera, suggestSettingsParameters) };
        }

        std::vector<Duration> suggestSettingsDurations;

        for(size_t i = 0; i < numFrames; i++)
        {
            const auto beforeSuggestSettings = HighResClock::now();
            const auto settingsVector{ Zivid::CaptureAssistant::suggestSettings(camera, suggestSettingsParameters) };
            const auto afterSuggestSettings = HighResClock::now();

            suggestSettingsDurations.push_back(afterSuggestSettings - beforeSuggestSettings);
            addToRawFile(std::string("suggestSettings duration: ")
                         + formatDuration(afterSuggestSettings - beforeSuggestSettings));
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
                                      const std::vector<unsigned int> &irises,
                                      const std::vector<size_t> &exposureTimes,
                                      const size_t numFrames3D)
    {
        const std::vector<Duration> captureDurationWithoutFilter =
            benchmarkCapture3D(camera, makeSettingsVector(irises, exposureTimes, false, false), numFrames3D);

        std::vector<bool> gaussian{ true, false, true };
        std::vector<bool> reflection{ false, true, true };

        std::vector<Duration> filterProcessingDurations;
        for(size_t i = 0; i < gaussian.size(); i++)
        {
            const std::vector<Duration> captureDurationWithFilter =
                benchmarkCapture3D(camera,
                                   makeSettingsVector(irises, exposureTimes, gaussian.at(i), reflection.at(i)),
                                   numFrames3D);

            const auto meanAndAverageFilterDurations =
                benchmarkFilterProcessing(captureDurationWithoutFilter, captureDurationWithFilter);

            filterProcessingDurations.push_back(std::get<0>(meanAndAverageFilterDurations));
            filterProcessingDurations.push_back(std::get<1>(meanAndAverageFilterDurations));
        }
        printFilterResults(filterProcessingDurations);
    }

    Zivid::Settings2D makeSettings2D(const size_t exposureTime)
    {
        Zivid::Settings2D settings;
        settings.set(Zivid::Settings2D::ExposureTime(std::chrono::microseconds{ exposureTime }));
        return settings;
    }

    void benchmarkCapture2D(Zivid::Camera &camera, const Zivid::Settings2D &settings, const size_t numFrames)
    {
        printCapture2DHeader(numFrames, settings);

        for(size_t i = 0; i < 5; i++) // Warmup frames
        {
            printProgress(i + 1, 5);
            camera.capture2D(settings);
        }

        std::vector<Duration> captureDurations;
        std::vector<Duration> allDurations;

        for(size_t i = 0; i < numFrames; i++)
        {
            printProgress(i + 1, numFrames);
            const auto beforeCapture = HighResClock::now();
            const auto frame2D = camera.capture2D(settings);
            const auto afterCapture = HighResClock::now();

            captureDurations.push_back(afterCapture - beforeCapture);
            addToRawFile(std::string("2D capture duration: ") + formatDuration(afterCapture - beforeCapture));
        }
        allDurations.push_back(computeMedianDuration(captureDurations));
        allDurations.push_back(computeAverageDuration(captureDurations));

        printCapture2DResults(allDurations);
    }

    void benchmarkSave(Zivid::Camera &camera, const size_t numFrames)
    {
        printSaveHeader(numFrames);

        auto frame = camera.capture();
        frame.getPointCloud();

        std::vector<Duration> allDurations;
        std::vector<std::string> fileNames{ "Zivid3D.zdf", "Zivid3D.ply", "Zivid3D.pcd", "Zivid3D.xyz" };
        for(const auto &fileName : fileNames)
        {
            addToRawFile(fileName);
            std::vector<Duration> durationsPerFormat;
            for(size_t j = 0; j < numFrames; j++)
            {
                printProgress(j + 1, numFrames);
                const auto beforeSave = HighResClock::now();
                frame.save(fileName);
                const auto afterSave = HighResClock::now();

                durationsPerFormat.push_back(afterSave - beforeSave);
                addToRawFile(std::string("save duration: ") + formatDuration(afterSave - beforeSave));
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

        const size_t exposureTime = getMinExposureTime(camera.modelName());
        const std::vector<size_t> exposureTimeOneFrame{ exposureTime };
        const std::vector<size_t> exposureTimeTwoFrames{ exposureTime, exposureTime };
        const std::vector<size_t> exposureTimeThreeFrames{ exposureTime, exposureTime, exposureTime };

        const std::vector<unsigned int> oneIris{ 21U };
        const std::vector<unsigned int> twoIrises{ 17U, 27U };
        const std::vector<unsigned int> threeIrises{ 14U, 21U, 35U };

        createEmptyOutputFile();

        benchmarkConnect(camera, numConnects);
        camera.connect();
        benchmarkAssistedCapture3D(camera, numFrames3D);
        benchmarkCapture3DAndFilters(camera, oneIris, exposureTimeOneFrame, numFrames3D);
        benchmarkCapture3D(camera, makeSettingsVector(twoIrises, exposureTimeTwoFrames, false, false), numFrames3D);
        benchmarkCapture3DAndFilters(camera, threeIrises, exposureTimeThreeFrames, numFrames3D);
        benchmarkCapture2D(camera, makeSettings2D(exposureTime), numFrames2D);
        benchmarkSave(camera, numFramesSave);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
