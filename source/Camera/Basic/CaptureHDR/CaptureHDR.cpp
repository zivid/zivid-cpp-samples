/*
Capture HDR point clouds, with color, from the Zivid camera.

For scenes with high dynamic range we combine multiple acquisitions to get an HDR point cloud.
*/

#include <Zivid/Zivid.h>

#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Configuring settings" << std::endl;
        Zivid::Settings settings;
        for(const auto aperture : { 11.31, 5.66, 2.83 })
        {
            std::cout << "Adding acquisition with aperture = " << aperture << std::endl;
            const auto acquisitionSettings = Zivid::Settings::Acquisition{
                Zivid::Settings::Acquisition::Aperture{ aperture },
            };
            settings.acquisitions().emplaceBack(acquisitionSettings);
        }

        std::cout << "Capturing frame (HDR)" << std::endl;
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
