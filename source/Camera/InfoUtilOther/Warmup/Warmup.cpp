/*
This example shows how to warm up the camera.
*/

#include <Zivid/Zivid.h>

#include <chrono>
#include <iostream>
#include <thread>

using HighResClock = std::chrono::high_resolution_clock;
using Duration = std::chrono::nanoseconds;

namespace
{
    Zivid::Frame assistedCapture(Zivid::Camera &camera)
    {
        const auto parameters = Zivid::CaptureAssistant::SuggestSettingsParameters{
            Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency::none,
            Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{ std::chrono::milliseconds{ 1000 } }
        };
        const auto settings = Zivid::CaptureAssistant::suggestSettings(camera, parameters);
        return camera.capture(settings);
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        auto warmupTime = std::chrono::minutes(10);
        auto captureCycle = std::chrono::seconds(5);

        std::cout << "Warm-up time: " << warmupTime.count() << " minutes" << std::endl;

        const auto beforeWarmup = HighResClock::now();

        while(HighResClock::now() - beforeWarmup < warmupTime)
        {
            const auto beforeCapture = HighResClock::now();
            assistedCapture(camera);
            const auto afterCapture = HighResClock::now();
            auto captureTime = afterCapture - beforeCapture;
            if(captureTime < captureCycle)
            {
                std::this_thread::sleep_for(captureCycle - captureTime);
            }
            else
            {
                std::cout << "Your capture time is longer than your desired capture cycle."
                          << "Please increase the desired capture cycle." << std::endl;
            }

            auto remainingTime = warmupTime - (HighResClock::now() - beforeWarmup);

            auto remainingTimeMinutes = std::chrono::duration_cast<std::chrono::minutes>(remainingTime);
            auto remainingTimeSeconds =
                std::chrono::duration_cast<std::chrono::seconds>(remainingTime - remainingTimeMinutes);
            std::cout << "Remaining time: " << remainingTimeMinutes.count() << " minutes, "
                      << remainingTimeSeconds.count() << " seconds" << std::endl;
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
