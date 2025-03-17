/*
Short example of a basic way to warm up the camera with specified time and capture cycle.
*/

#include <Zivid/Zivid.h>

#include <clipp.h>

#include <chrono>
#include <iostream>
#include <thread>

using SteadyClock = std::chrono::steady_clock;
using Duration = std::chrono::nanoseconds;

namespace
{
    Zivid::Settings loadOrDefaultSettings(const std::string &settingsPath)
    {
        if(!settingsPath.empty())
        {
            std::cout << "Loading settings from file" << std::endl;
            return Zivid::Settings(settingsPath);
        }

        std::cout << "Using default 3D settings" << std::endl;
        auto settings = Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} } };

        return settings;
    }
} // namespace

int main(int argc, char **argv)
{
    try
    {
        std::string settingsPath;
        size_t captureCycleSeconds = 5;

        auto settingsOption = clipp::option("--settings-path")
                              & clipp::value("Path to the YML file that contains camera settings", settingsPath);
        auto captureCycleOption =
            clipp::option("--capture-cycle") & clipp::value("Capture cycle in seconds", captureCycleSeconds);

        auto cli = (settingsOption, captureCycleOption);

        if(!parse(argc, argv, cli))
        {
            auto fmt = clipp::doc_formatting{}.alternatives_min_split_size(1).surround_labels("\"", "\"");
            std::cout << clipp::usage_lines(cli, "Warmup", fmt) << std::endl;
            throw std::runtime_error{ "Invalid usage" };
        }
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        const auto warmupTime = std::chrono::minutes(10);
        const auto captureCycle = std::chrono::seconds{ captureCycleSeconds };
        const auto settings = loadOrDefaultSettings(settingsPath);

        std::cout << "Starting warm up for: " << warmupTime.count() << " minutes" << std::endl;

        const auto beforeWarmup = SteadyClock::now();

        while(SteadyClock::now() - beforeWarmup < warmupTime)
        {
            const auto beforeCapture = SteadyClock::now();

            // Use the same capture method as you would use in production
            // to get the most accurate results from warmup
            camera.capture3D(settings);

            const auto afterCapture = SteadyClock::now();
            const auto captureTime = afterCapture - beforeCapture;
            if(captureTime < captureCycle)
            {
                std::this_thread::sleep_for(captureCycle - captureTime);
            }
            else
            {
                std::cout << "Your capture time is longer than your desired capture cycle."
                          << "Please increase the desired capture cycle." << std::endl;
            }

            const auto remainingTime = warmupTime - (SteadyClock::now() - beforeWarmup);

            const auto remainingTimeMinutes = std::chrono::duration_cast<std::chrono::minutes>(remainingTime);
            const auto remainingTimeSeconds =
                std::chrono::duration_cast<std::chrono::seconds>(remainingTime - remainingTimeMinutes);
            std::cout << "Remaining time: " << remainingTimeMinutes.count() << " minutes, "
                      << remainingTimeSeconds.count() << " seconds." << std::endl;
        }
        std::cout << "Warm up completed" << std::endl;
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

