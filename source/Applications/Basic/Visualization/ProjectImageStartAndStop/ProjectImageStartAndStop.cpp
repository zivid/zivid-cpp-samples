/*
Start the Image Projection and Stop it.

How to stop the image projection is demonstrated in three different ways:
- calling stop() function on the projected image handle
- projected image handle going out of scope
- triggering a 3D capture

*/

#include <Zivid/Application.h>
#include <Zivid/Projection/Projection.h>
#include <Zivid/Zivid.h>

#include <iostream>

namespace
{
    Zivid::Image<Zivid::ColorBGRA> createProjectorImage(
        const Zivid::Resolution &projectorResolution,
        const Zivid::ColorBGRA &ZividColor)
    {
        const std::vector<Zivid::ColorBGRA> imageData(projectorResolution.size(), ZividColor);
        Zivid::Image<Zivid::ColorBGRA> projectorImage{ projectorResolution, imageData.begin(), imageData.end() };

        return projectorImage;
    }


    void projecting(Zivid::Camera &camera, const Zivid::Image<Zivid::ColorBGRA> &projectorImageFunctionScope)
    {
        auto projectedImageHandle = Zivid::Projection::showImage(camera, projectorImageFunctionScope);

        std::cout << "Press enter to stop projecting by leaving a function scope" << std::endl;
        std::cin.get();
    }


} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Retrieving the projector resolution that the camera supports" << std::endl;
        const auto projectorResolution = Zivid::Projection::projectorResolution(camera);

        const auto redColor = Zivid::ColorBGRA(0, 0, 255, 255);

        auto projectorImage = createProjectorImage(projectorResolution, redColor);

        auto projectedImageHandle = Zivid::Projection::showImage(camera, projectorImage);

        std::cout << "Press enter to stop projecting using the \".stop()\" function." << std::endl;
        std::cin.get();
        projectedImageHandle.stop();

        const auto greenColor = Zivid::ColorBGRA(0, 255, 0, 255);
        {
            projectorImage = createProjectorImage(projectorResolution, greenColor);
            projectedImageHandle = Zivid::Projection::showImage(camera, projectorImage);

            std::cout << "Press enter to stop projecting by leaving a local scope" << std::endl;
            std::cin.get();
        }

        const auto blueColor = Zivid::ColorBGRA(255, 0, 0, 255);
        projectorImage = createProjectorImage(projectorResolution, blueColor);
        projecting(camera, projectorImage);

        const auto zividPinkColor = Zivid::ColorBGRA(114, 52, 237, 255);
        projectorImage = createProjectorImage(projectorResolution, zividPinkColor);
        projectedImageHandle = Zivid::Projection::showImage(camera, projectorImage);

        std::cout << "Press enter to stop projecting by performing a 3D capture" << std::endl;
        std::cin.get();
        const auto settings = Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition() } };
        camera.capture(settings);

        std::cout << "Press enter to exit" << std::endl;
        std::cin.get();
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
