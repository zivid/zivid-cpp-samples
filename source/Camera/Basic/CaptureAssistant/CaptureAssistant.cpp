/*
Capture Assistant to capture point clouds, with color, from the Zivid camera.
*/

#include <Zivid/Zivid.h>

#include <chrono>
#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        const auto suggestSettingsParameters = Zivid::CaptureAssistant::SuggestSettingsParameters{
            Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency::none,
            Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{ std::chrono::milliseconds{ 1200 } }
        };

        std::cout << "Running Capture Assistant with parameters:\n" << suggestSettingsParameters << std::endl;
        auto settings = Zivid::CaptureAssistant::suggestSettings(camera, suggestSettingsParameters);

        std::cout << "Settings suggested by Capture Assistant:" << std::endl;
        std::cout << settings.acquisitions() << std::endl;

        std::cout << "Manually configuring processing settings (Capture Assistant only suggests acquisition settings)"
                  << std::endl;
        const auto processing = Zivid::Settings::Processing{
            Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes,
            Zivid::Settings::Processing::Filters::Reflection::Removal::Experimental::Mode::global,
            Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
            Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{ 1.5 }
        };
        settings.set(processing);

        std::cout << "Capturing frame" << std::endl;
        const auto frame = camera.capture(settings);

        const auto dataFile = "Frame.zdf";
        std::cout << "Saving frame to file: " << dataFile << std::endl;
        frame.save(dataFile);
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
