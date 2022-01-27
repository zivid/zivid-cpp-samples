/*
Capture point clouds, with color, from the Zivid file camera. Currently supported by Zivid One.

This example can be used without access to a physical camera.
The ZFC files for this sample can be found under the main instructions for Zivid samples.
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

        std::cout << "Creating virtual camera using file: " << fileCamera << std::endl;
        auto camera = zivid.createFileCamera(fileCamera);

        std::cout << "Configuring settings" << std::endl;
        const auto settings = Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} },
                                               Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
                                               Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{ 1.5 },
                                               Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes,
                                               Zivid::Settings::Processing::Color::Balance::Red{ 1 },
                                               Zivid::Settings::Processing::Color::Balance::Green{ 1 },
                                               Zivid::Settings::Processing::Color::Balance::Blue{ 1 } };

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
}
