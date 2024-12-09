/*
Capture 2D images from the Zivid camera.

The color information is provided in linear RGB and sRGB color spaces.

Color represented in linear RGB space is suitable as input to traditional computer vision algorithms
for specialized tasks that require precise color measurements or high dynamic range.

Color represented in sRGB color space is suitable for showing an image on a display and for machine
learning based tasks like image classification, object detection, and segmentation as most image
datasets used for training neural networks are in sRGB color space.

More information about linear RGB and sRGB color spaces is available at:
https://support.zivid.com/en/latest/reference-articles/color-spaces-and-output-formats.html#color-spaces

Note: While the data of the saved images is provided in linear RGB and sRGB color space, the meta data
information that indicates the color space is not saved in the .PNG. Hence, both images are likely
to be interpreted as if they were saved in sRGB color space and displayed as such.
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
                                   Zivid::Settings2D::Acquisition::ExposureTime{ std::chrono::microseconds{ 20000 } },
                                   Zivid::Settings2D::Acquisition::Aperture{ 9.51 },
                                   Zivid::Settings2D::Acquisition::Brightness{ 1.80 },
                                   Zivid::Settings2D::Acquisition::Gain{ 2.0 } } },
                               Zivid::Settings2D::Processing::Color::Balance::Red{ 1 },
                               Zivid::Settings2D::Processing::Color::Balance::Green{ 1 },
                               Zivid::Settings2D::Processing::Color::Balance::Blue{ 1 } };

        std::cout << "Capturing 2D frame" << std::endl;
        const auto frame2D = camera.capture(settings2D);

        std::cout << "Getting color image (linear RGB color space)" << std::endl;
        const auto image = frame2D.imageRGBA();

        const auto pixelRow = 100;
        const auto pixelCol = 50;
        const auto pixelRGB = image(pixelRow, pixelCol);
        std::cout << "Color at pixel (" << pixelRow << "," << pixelCol << "):  R:" << std::to_string(pixelRGB.r)
                  << "  G:" << std::to_string(pixelRGB.g) << "  B:" << std::to_string(pixelRGB.b)
                  << "  A:" << std::to_string(pixelRGB.a) << std::endl;

        const auto imageFile = "ImageRGB.png";
        std::cout << "Saving 2D color image (linear RGB color space) to file: " << imageFile << std::endl;
        image.save(imageFile);

        std::cout << "Getting color image (sRGB color space)" << std::endl;
        const auto imageSRGB = frame2D.imageSRGB();

        const auto pixelSRGB = imageSRGB(pixelRow, pixelCol);
        std::cout << "Color at pixel (" << pixelRow << "," << pixelCol << "):  R:" << std::to_string(pixelSRGB.r)
                  << "  G:" << std::to_string(pixelSRGB.g) << "  B:" << std::to_string(pixelSRGB.b)
                  << "  A:" << std::to_string(pixelSRGB.a) << std::endl;

        const auto imageSRGBFile = "ImageSRGB.png";
        std::cout << "Saving 2D color image  (sRGB color space) to file: " << imageSRGBFile << std::endl;
        imageSRGB.save(imageSRGBFile);
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
