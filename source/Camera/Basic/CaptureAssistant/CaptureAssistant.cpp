#include <Zivid/CaptureAssistant.h>
#include <Zivid/HDR.h>
#include <Zivid/Zivid.h>

#include <chrono>
#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera{ zivid.connectCamera() };

        const auto *resultFile = "result.zdf";
        Zivid::CaptureAssistant::SuggestSettingsParameters suggestSettingsParameters(
            std::chrono::milliseconds{ 1200 }, Zivid::CaptureAssistant::AmbientLightFrequency::none);

        std::cout << "Running Capture Assistant with parameters: " << suggestSettingsParameters << std::endl;
        const auto settingsVector{ Zivid::CaptureAssistant::suggestSettings(camera, suggestSettingsParameters) };

        std::cout << "Suggested settings are:" << std::endl;
        for(const auto &setting : settingsVector)
        {
            std::cout << setting << std::endl;
        }

        std::cout << "Capture (and merge) frames using automatically suggested settings" << std::endl;
        const auto hdrFrame{ Zivid::HDR::capture(camera, settingsVector) };

        std::cout << "Saving frame to file: " << resultFile << std::endl;
        hdrFrame.save(resultFile);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
