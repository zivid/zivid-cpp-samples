/*
Short example of a basic way to warm up the camera with specified time and capture cycle.
*/

#include <Zivid/Zivid.h>

#include <chrono>
#include <iostream>
#include <thread>

using HighResClock = std::chrono::high_resolution_clock;
using Duration = std::chrono::nanoseconds;

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        const auto warmupTime = std::chrono::minutes(10);
        const auto captureCycle = std::chrono::seconds(5);
        const auto maxCaptureTime = std::chrono::milliseconds(1000);

        std::cout << "Getting camera settings" << std::endl;
        const auto parameters = Zivid::CaptureAssistant::SuggestSettingsParameters{
            Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency::none,
            Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{ maxCaptureTime }
        };
        const auto settings = Zivid::CaptureAssistant::suggestSettings(camera, parameters);

        std::cout << "Starting warm up for: " << warmupTime.count() << " minutes" << std::endl;

        const auto beforeWarmup = HighResClock::now();

        while(HighResClock::now() - beforeWarmup < warmupTime)
        {
            const auto beforeCapture = HighResClock::now();
            camera.capture(settings);
            const auto afterCapture = HighResClock::now();
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

            const auto remainingTime = warmupTime - (HighResClock::now() - beforeWarmup);

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

