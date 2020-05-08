/*
This example shows how to capture point clouds, with color, from the Zivid camera.
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

        std::cout << "Creating settings" << std::endl;
        Zivid::Settings settings;
        for(const auto aperture : { 11.31, 5.66, 2.83 })
        {
            std::cout << "Adding acquisition with aperture= " << aperture << std::endl;
            const auto acquisitionSettings = Zivid::Settings::Acquisition{
                Zivid::Settings::Acquisition::Aperture{ aperture },
            };
            settings.acquisitions().emplaceBack(acquisitionSettings);
        }

        std::cout << "Capturing HDR frame" << std::endl;
        const auto hdrFrame = camera.capture(settings);

        std::cout << "Saving the HDR frame" << std::endl;
        hdrFrame.save("HDR.zdf");
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
