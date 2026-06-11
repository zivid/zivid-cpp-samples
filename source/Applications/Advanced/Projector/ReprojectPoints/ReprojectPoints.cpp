/*
Illuminate checkerboard (Zivid Calibration Board) centers by getting the checkerboard feature points
and illuminating them with the projector.

The checkerboard feature points are first found through the API. These points are then used to get the
corresponding projector pixels. The projector pixel coordinates are then used to draw markers at the
correct locations before displaying the image using the projector.
*/

#include <Zivid/Application.h>
#include <Zivid/Calibration/Detector.h>
#include <Zivid/Exception.h>
#include <Zivid/Projection/Projection.h>

#include <opencv2/opencv.hpp>

#include <cmath>
#include <cstring>
#include <iostream>

namespace
{
    void drawFilledCircles(
        cv::Mat image,
        const Zivid::Array2D<Zivid::PointXY> &positions,
        int circleSizeInPixels,
        const cv::Scalar &circleColor)
    {
        for(const auto &position : positions)
        {
            if(!std::isnan(position.x) && !std::isnan(position.y))
            {
                const cv::Point point(position.x, position.y);
                cv::circle(image, point, circleSizeInPixels, circleColor, -1);
            }
        }
    }

    Zivid::Settings2D get2DCaptureSettings(const Zivid::Camera &camera)
    {
        Zivid::Settings2D settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{
            Zivid::Settings2D::Acquisition::Brightness{ 0.0 },
            Zivid::Settings2D::Acquisition::ExposureTime{ std::chrono::microseconds{ 20000 } },
            Zivid::Settings2D::Acquisition::Aperture{ 3.00 } } } };

        auto model = camera.info().model();
        switch(model.value())
        {
            case Zivid::CameraInfo::Model::ValueType::zividTwo:
            case Zivid::CameraInfo::Model::ValueType::zividTwoL100:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusM130:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusM60:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusL110:
            {
                settings2D.set(Zivid::Settings2D::Sampling::Color::rgb);
                break;
            }
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusMR130:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusMR60:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusLR110:
            case Zivid::CameraInfo::Model::ValueType::zivid3XL250:
            {
                settings2D.set(Zivid::Settings2D::Sampling::Color::grayscale);
                break;
            }
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusSmall:
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusMedium:
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusLarge:
            {
                throw std::runtime_error("Unsupported camera model '" + model.toString() + "'");
            }
            default: throw std::runtime_error("Unhandled enum value '" + model.toString() + "'");
        }

        return settings2D;
    }

} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Capturing and detecting feature points of the Zivid checkerboard" << std::endl;
        const auto detectionResult = Zivid::Calibration::detectCalibrationBoard(camera);

        if(!detectionResult.valid())
        {
            throw std::runtime_error("Calibration board not detected! " + detectionResult.statusDescription());
        }

        const auto featurePoints = detectionResult.featurePoints();

        std::cout << "Getting projector pixels (2D) corresponding to points (3D)" << std::endl;
        const auto projectorPixels = Zivid::Projection::pixelsFrom3DPoints(camera, featurePoints);

        std::cout << "Retrieving the projector resolution that the camera supports" << std::endl;
        const auto projectorResolution = Zivid::Projection::projectorResolution(camera);

        std::cout << "Creating a blank projector image with resolution: " << projectorResolution.toString()
                  << std::endl;
        const cv::Scalar backgroundColor{ 0, 0, 0, 255 };
        auto projectorImageOpenCV = cv::Mat{ static_cast<int>(projectorResolution.height()),
                                             static_cast<int>(projectorResolution.width()),
                                             CV_8UC4,
                                             backgroundColor };

        std::cout << "Drawing circles on the projector image for each grid point" << std::endl;
        const cv::Scalar circleColor{ 0, 255, 0, 255 };
        drawFilledCircles(projectorImageOpenCV, projectorPixels, 2, circleColor);

        std::cout << "Creating a Zivid::Image from the OpenCV image" << std::endl;
        const Zivid::Image<Zivid::ColorBGRA> projectorImage{ projectorResolution,
                                                             projectorImageOpenCV.datastart,
                                                             projectorImageOpenCV.dataend };

        const std::string projectorImageFile = "ProjectorImage.png";
        std::cout << "Saving the projector image to file: " << projectorImageFile << std::endl;
        projectorImage.save(projectorImageFile);

        std::cout << "Displaying the projector image" << std::endl;


        { // A Local Scope to handle the projected image lifetime

            auto projectedImageHandle = Zivid::Projection::showImage(camera, projectorImage);

            const auto settings2D = get2DCaptureSettings(camera);

            std::cout << "Capturing a 2D image with the projected image" << std::endl;
            const auto frame2D = projectedImageHandle.capture2D(settings2D);

            const std::string capturedImageFile = "CapturedImage.png";
            std::cout << "Saving the captured image: " << capturedImageFile << std::endl;
            frame2D.imageBGRA_SRGB().save(capturedImageFile);

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
