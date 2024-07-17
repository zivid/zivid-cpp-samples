/*
Illuminate checkerboard (Zivid Calibration Board) corners by getting checkerboard pose
from the API and transforming the desired points to projector pixel coordinates.

The checkerboard pose is determined first and then used to estimate the coordinates of corners
in the camera frame. These points are then passed to the API to get the corresponding projector pixels.
The projector pixel coordinates are then used to draw markers at the correct locations before displaying
the image using the projector.
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
    std::vector<cv::Matx41f> checkerboardGrid()
    {
        std::vector<cv::Matx41f> points;
        for(int x = 0; x < 7; x++)
        {
            for(int y = 0; y < 6; y++)
            {
                const float xPos = x * 30.0F;
                const float yPos = y * 30.0F;
                points.emplace_back(xPos, yPos, 0.0F, 1.0F);
            }
        }
        return points;
    }


    std::vector<Zivid::PointXYZ> transformGridToCalibrationBoard(
        const std::vector<cv::Matx41f> &grid,
        const Zivid::Matrix4x4 &transformCameraToCheckerboard)
    {
        std::vector<Zivid::PointXYZ> pointsInCameraFrame;
        const auto transformationMatrix = cv::Matx44f{ transformCameraToCheckerboard.data() };
        for(const auto &point : grid)
        {
            const auto transformedPoint = transformationMatrix * point;
            pointsInCameraFrame.emplace_back(transformedPoint(0, 0), transformedPoint(1, 0), transformedPoint(2, 0));
        }
        return pointsInCameraFrame;
    }


    void drawFilledCircles(
        cv::Mat image,
        const std::vector<Zivid::PointXY> &positions,
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

} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Capturing and estimating pose of the Zivid checkerboard in the camera frame" << std::endl;
        const auto detectionResult = Zivid::Calibration::detectCalibrationBoard(camera);
        if(!detectionResult.valid())
        {
            throw std::runtime_error("Calibration board not detected!");
        }

        std::cout << "Estimating checkerboard pose" << std::endl;
        const auto transformCameraToCheckerboard = detectionResult.pose().toMatrix();
        std::cout << transformCameraToCheckerboard << std::endl;

        std::cout << "Creating a grid of 7 x 6 points (3D) with 30 mm spacing to match checkerboard corners"
                  << std::endl;
        const auto grid = checkerboardGrid();

        std::cout << "Transforming the grid to the camera frame" << std::endl;
        const auto pointsInCameraFrame = transformGridToCalibrationBoard(grid, transformCameraToCheckerboard);

        std::cout << "Getting projector pixels (2D) corresponding to points (3D) in the camera frame" << std::endl;
        const auto projectorPixels = Zivid::Projection::pixelsFrom3DPoints(camera, pointsInCameraFrame);

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
