/*
This example shows how to capture point clouds, with color, from the Zivid file camera.
This example can be used without access to a physical camera.
*/

#include <Zivid/Zivid.h>

#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        // The fileCamera file is in Zivid Sample Data. See instructions in README.md
        const auto fileCamera = std::string(ZIVID_SAMPLE_DATA_DIR) + "/FileCameraZividOne.zfc";

        std::cout << "Initializing camera emulation using file: " << cameraFile << std::endl;
        auto camera = zivid.createFileCamera(cameraFile);

        std::cout << "Configuring settings" << std::endl;
        const auto settings = Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} },
                                               Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
                                               Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{ 1.5 },
                                               Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes,
                                               Zivid::Settings::Processing::Color::Balance::Red{ 1 },
                                               Zivid::Settings::Processing::Color::Balance::Green{ 1 },
                                               Zivid::Settings::Processing::Color::Balance::Blue{ 1 } };

        std::cout << "Capture a frame" << std::endl;
        const auto frame = camera.capture(settings);

        std::cout << "Saving frame to file: " << resultFile << std::endl;
        frame.save(resultFile);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
