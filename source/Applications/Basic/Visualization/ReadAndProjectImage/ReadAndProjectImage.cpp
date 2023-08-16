/*
Read a 2D image from file and project it using the camera projector.

The image for this sample can be found under the main instructions for Zivid samples.
*/

#include <Zivid/Application.h>
#include <Zivid/Exception.h>
#include <Zivid/Experimental/Calibration/InfieldCorrection.h>
#include <Zivid/Experimental/Projection.h>

#include <opencv2/opencv.hpp>

#include <cmath>
#include <cstring>
#include <iostream>

namespace
{
    Zivid::Image<Zivid::ColorBGRA> resizeAndCreateProjectorImage(
        const cv::Mat &inputImage,
        const Zivid::Resolution &projectorResolution)
    {
        cv::Mat projectorImageResized;
        cv::Mat projectorImageBGRA;
        cv::resize(
            inputImage,
            projectorImageResized,
            cv::Size(projectorResolution.width(), projectorResolution.height()),
            cv::INTER_LINEAR);
        cv::cvtColor(projectorImageResized, projectorImageBGRA, cv::COLOR_BGR2BGRA);

        std::cout << "Creating a Zivid::Image from the OpenCV image" << std::endl;
        Zivid::Image<Zivid::ColorBGRA> projectorImage{ projectorResolution,
                                                       projectorImageBGRA.datastart,
                                                       projectorImageBGRA.dataend };

        return projectorImage;
    }

} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::string imageFile = std::string(ZIVID_SAMPLE_DATA_DIR) + "/ZividLogo.png";
        std::cout << "Reading 2D image from file: " << imageFile << std::endl;
        const auto inputImage = cv::imread(imageFile, cv::IMREAD_UNCHANGED);

        std::cout << "input image size: " << inputImage.size() << std::endl;

        std::cout << "Retrieving the projector resolution that the camera supports" << std::endl;
        const auto projectorResolution = Zivid::Experimental::Projection::projectorResolution(camera);

        std::cout << "Resizing input image to fit projector resolution: " << projectorResolution.toString()
                  << std::endl;
        const auto projectorImage = resizeAndCreateProjectorImage(inputImage, projectorResolution);

        const std::string projectorImageFile = "ProjectorImage.png";
        std::cout << "Saving the projector image to file: " << projectorImageFile << std::endl;
        projectorImage.save(projectorImageFile);

        std::cout << "Displaying the projector image" << std::endl;

        { // A Local Scope to handle the projected image lifetime

            auto projectedImageHandle = Zivid::Experimental::Projection::showImage(camera, projectorImage);

            const Zivid::Settings2D settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{
                Zivid::Settings2D::Acquisition::Brightness{ 0.0 },
                Zivid::Settings2D::Acquisition::ExposureTime{ std::chrono::microseconds{ 20000 } },
                Zivid::Settings2D::Acquisition::Aperture{ 2.83 } } } };

            std::cout << "Capturing a 2D image with the projected image" << std::endl;
            const auto frame2D = projectedImageHandle.capture(settings2D);

            const std::string capturedImageFile = "CapturedImage.png";
            std::cout << "Saving the captured image: " << capturedImageFile << std::endl;
            frame2D.imageBGRA().save(capturedImageFile);

            std::cout << "Press enter to stop projecting..." << std::endl;
            std::cin.get();

        } // projectedImageHandle now goes out of scope, thereby stopping the projection

        std::cout << "Done" << std::endl;
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
