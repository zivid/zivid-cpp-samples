/*
Capture point clouds with multiple cameras sequentially.
*/

#include <Zivid/Zivid.h>
#include <chrono>
#include <iostream>
#include <vector>

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

        for(auto &camera : cameras)
        {
            std::cout << "Capturing frame with camera: " << camera.info().serialNumber() << std::endl;
            const auto parameters = Zivid::CaptureAssistant::SuggestSettingsParameters{
                Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency::none,
                Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{ std::chrono::milliseconds{ 800 } }
            };
            const auto settings = Zivid::CaptureAssistant::suggestSettings(camera, parameters);
            auto frame = camera.capture(settings);

            const auto dataFile = "Frame_" + camera.info().serialNumber().value() + ".zdf";

            std::cout << "Saving frame to file: " << dataFile << std::endl;
            frame.save(dataFile);
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
