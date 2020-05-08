/*
This example shows how to capture 2D images from the Zivid camera.
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

        std::cout << "Setting the capture settings" << std::endl;
        // Note: The Zivid SDK supports 2D captures with a single acquisition only
        const auto settings2D =
            Zivid::Settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{
                                   Zivid::Settings2D::Acquisition::ExposureTime{ std::chrono::microseconds{ 10000 } },
                                   Zivid::Settings2D::Acquisition::Aperture{ 2.83 },
                                   Zivid::Settings2D::Acquisition::Brightness{ 1.0 },
                                   Zivid::Settings2D::Acquisition::Gain{ 1.0 } } },
                               Zivid::Settings2D::Processing::Color::Balance::Red{ 1 },
                               Zivid::Settings2D::Processing::Color::Balance::Green{ 1 },
                               Zivid::Settings2D::Processing::Color::Balance::Blue{ 1 } };

        std::cout << "Capturing a 2D frame" << std::endl;
        const auto frame = camera.capture(settings2D);

        std::cout << "Get ColorRGBA image from Frame2D" << std::endl;
        const auto image = frame.imageRGBA();

        const auto pixelRow = 100;
        const auto pixelCol = 50;
        std::cout << "Get pixel color at row=" << pixelRow << ","
                  << "column=" << pixelCol << std::endl;
        const auto pixel = image(pixelRow, pixelCol);
        std::cout << "Pixel color: R=" << std::to_string(pixel.r) << ", G=" << std::to_string(pixel.g)
                  << ", B=" << std::to_string(pixel.b) << ", A=" << std::to_string(pixel.a) << std::endl;

        const auto *resultFile = "Image.png";
        std::cout << "Saving the image to " << resultFile << std::endl;
        image.save(resultFile);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
