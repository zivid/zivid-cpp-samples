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
        auto settings = Zivid::Settings2D();
        settings.set(Zivid::Settings2D::ExposureTime{ std::chrono::microseconds{ 10000 } });
        settings.set(Zivid::Settings2D::Gain{ 1.0 });
        settings.set(Zivid::Settings2D::Iris{ 35 });

        std::cout << "Capture a 2D frame" << std::endl;
        auto frame = camera.capture2D(settings);

        std::cout << "Get RGBA8 image from Frame2D" << std::endl;
        auto image = frame.image<Zivid::RGBA8>();

        std::cout << "Get pixel color at row=100, column=50" << std::endl;
        auto pixel = image(100, 50);
        std::cout << "Pixel color: R=" << static_cast<int>(pixel.r) << ", G=" << static_cast<int>(pixel.g)
                  << ", B=" << static_cast<int>(pixel.b) << ", A=" << static_cast<int>(pixel.a) << std::endl;

        const auto *resultFile = "result.png";
        std::cout << "Saving the image to " << resultFile << std::endl;
        image.save(resultFile);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
