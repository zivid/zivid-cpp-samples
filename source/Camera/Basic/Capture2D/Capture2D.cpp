/*
Capture 2D images from the Zivid camera.
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

        std::cout << "Configuring 2D settings" << std::endl;
        // Note: The Zivid SDK supports 2D captures with a single acquisition only
        const auto settings2D =
            Zivid::Settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{
                                   Zivid::Settings2D::Acquisition::ExposureTime{ std::chrono::microseconds{ 30000 } },
                                   Zivid::Settings2D::Acquisition::Aperture{ 11.31 },
                                   Zivid::Settings2D::Acquisition::Brightness{ 1.80 },
                                   Zivid::Settings2D::Acquisition::Gain{ 2.0 } } },
                               Zivid::Settings2D::Processing::Color::Balance::Red{ 1 },
                               Zivid::Settings2D::Processing::Color::Balance::Green{ 1 },
                               Zivid::Settings2D::Processing::Color::Balance::Blue{ 1 } };

        std::cout << "Capturing 2D frame" << std::endl;
        const auto frame2D = camera.capture(settings2D);

        std::cout << "Getting RGBA image" << std::endl;
        const auto image = frame2D.imageRGBA();

        const auto pixelRow = 100;
        const auto pixelCol = 50;
        const auto pixel = image(pixelRow, pixelCol);
        std::cout << "Color at pixel (" << pixelRow << "," << pixelCol << "):  R:" << std::to_string(pixel.r)
                  << "  G:" << std::to_string(pixel.g) << "  B:" << std::to_string(pixel.b)
                  << "  A:" << std::to_string(pixel.a) << std::endl;

        const auto *imageFile = "Image.png";
        std::cout << "Saving 2D color image to file: " << imageFile << std::endl;
        image.save(imageFile);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }
}
