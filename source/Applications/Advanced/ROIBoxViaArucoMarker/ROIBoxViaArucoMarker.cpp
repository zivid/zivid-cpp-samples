/*
Filter the point cloud based on a ROI box given relative to the ArUco marker on a Zivid Calibration Board.
The ZFC file for this sample can be downloaded from https://support.zivid.com/en/latest/api-reference/samples/sample-data.html.

This sample depends on ArUco libraries in OpenCV with extra modules (https://github.com/opencv/opencv_contrib).
*/

#include <Zivid/Experimental/Calibration.h>
#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/aruco.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <thread>

namespace
{
    struct Line2d
    {
        float slope;
        float intercept;
    };

    float clamp(const float n, const float lower, const float upper)
    {
        return std::max(lower, std::min(n, upper));
    }

    Line2d fitLine(const cv::Point2f &point1, const cv::Point2f &point2)
    {
        // Fitting a line y=a*x + b to 2 points
        const auto slope{ (point2.y - point1.y) / (point2.x - point1.x) };
        const auto intercept{ point1.y - slope * point1.x };
        return { slope, intercept };
    };

    cv::Point2f estimate2DMarkerCenter(const std::vector<cv::Point2f> &markerCorners)
    {
        const auto &markerCorner0 = markerCorners[0];
        const auto &markerCorner1 = markerCorners[1];
        const auto &markerCorner2 = markerCorners[2];
        const auto &markerCorner3 = markerCorners[3];

        // Fitting line between two diagonal marker corners
        const auto backDiagonal{ fitLine(markerCorner2, markerCorner0) };
        const auto forwardDiagonal{ fitLine(markerCorner3, markerCorner1) };

        // Finding intersection of the two lines
        // a1*x + b1 = a2*x + b2
        // x = (b2-b1) / (a1-a2)
        // y = a1*x + b1
        const auto xCoordinate =
            (forwardDiagonal.intercept - backDiagonal.intercept) / (backDiagonal.slope - forwardDiagonal.slope);
        const auto yCoordinate = backDiagonal.slope * xCoordinate + backDiagonal.intercept;
        auto markerCenter = cv::Point2f(xCoordinate, yCoordinate);

        return markerCenter;
    }

    std::vector<cv::Point3f> estimate3DMarkerPoints(
        const Zivid::PointCloud &pointCloud,
        const std::vector<cv::Point2f> &markerPoints2D)
    {
        if(markerPoints2D.empty())
        {
            return {};
        }

        const float width = pointCloud.width();
        const float height = pointCloud.height();
        const auto points = pointCloud.copyPointsXYZ();

        std::vector<cv::Point3f> markerPoints3D;
        markerPoints3D.reserve(markerPoints2D.size());

        for(const auto &point2D : markerPoints2D)
        {
            // Estimating the 3D center/corners of the marker using bilinear interpolation
            // See https://en.wikipedia.org/wiki/Bilinear_interpolation
            const auto x = point2D.x;
            const auto y = point2D.y;

            // Getting pixel coordinates for the four known pixels next to the interpolation point
            const auto xRoundedDown = clamp(std::floor(x), 0, width - 1);
            const auto xRoundedUp = clamp(std::ceil(x), 0, width - 1);
            const auto yRoundedDown = clamp(std::floor(y), 0, height - 1);
            const auto yRoundedUp = clamp(std::ceil(y), 0, height - 1);

            // Getting 3D coordinates for the four points next to the interpolation point
            const auto q11 = points(yRoundedDown, xRoundedDown);
            const auto q12 = points(yRoundedUp, xRoundedDown);
            const auto q21 = points(yRoundedDown, xRoundedUp);
            const auto q22 = points(yRoundedUp, xRoundedUp);

            // Linear interpolation in x direction
            // f(x,y1) = (x2 - x)/(x2 - x1) * f(q11) + (x - x1)/(x2 - x1) * f(q21)
            const cv::Point3f fxy1{ (xRoundedUp - x) / (xRoundedUp - xRoundedDown) * q11.x
                                        + (x - xRoundedDown) / (xRoundedUp - xRoundedDown) * q21.x,
                                    (xRoundedUp - x) / (xRoundedUp - xRoundedDown) * q11.y
                                        + (x - xRoundedDown) / (xRoundedUp - xRoundedDown) * q21.y,
                                    (xRoundedUp - x) / (xRoundedUp - xRoundedDown) * q11.z
                                        + (x - xRoundedDown) / (xRoundedUp - xRoundedDown) * q21.z };
            // f(x,y2) = (x2 - x)/(x2 - x1) * f(q12) + (x - x1)/(x2 - x1) * f(q22)
            const cv::Point3f fxy2{ (xRoundedUp - x) / (xRoundedUp - xRoundedDown) * q12.x
                                        + (x - xRoundedDown) / (xRoundedUp - xRoundedDown) * q22.x,
                                    (xRoundedUp - x) / (xRoundedUp - xRoundedDown) * q12.y
                                        + (x - xRoundedDown) / (xRoundedUp - xRoundedDown) * q22.y,
                                    (xRoundedUp - x) / (xRoundedUp - xRoundedDown) * q12.z
                                        + (x - xRoundedDown) / (xRoundedUp - xRoundedDown) * q22.z };

            // Linear interpolation in y direction
            // f(x,y) = (y2 - y)/(y2 - y1) * f(x,y1) + (y - y1)/(y2 - y1) * f(x,y2)
            markerPoints3D.emplace_back(cv::Point3f{ (yRoundedUp - y) / (yRoundedUp - yRoundedDown) * fxy1.x
                                                         + (y - yRoundedDown) / (yRoundedUp - yRoundedDown) * fxy2.x,
                                                     (yRoundedUp - y) / (yRoundedUp - yRoundedDown) * fxy1.y
                                                         + (y - yRoundedDown) / (yRoundedUp - yRoundedDown) * fxy2.y,
                                                     (yRoundedUp - y) / (yRoundedUp - yRoundedDown) * fxy1.z
                                                         + (y - yRoundedDown) / (yRoundedUp - yRoundedDown) * fxy2.z });
        }

        return markerPoints3D;
    }

    Zivid::Matrix4x4 transformationMatrix(const cv::Matx33f &rotationMatrix, const cv::Vec3f &translationVector)
    {
        auto transformMatrix = Zivid::Matrix4x4{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

        for(size_t row = 0; row < rotationMatrix.rows; row++)
        {
            for(size_t col = 0; col < rotationMatrix.cols; col++)
            {
                transformMatrix(row, col) = rotationMatrix(row, col);
            }
        }

        transformMatrix(0, 3) = translationVector[0];
        transformMatrix(1, 3) = translationVector[1];
        transformMatrix(2, 3) = translationVector[2];

        return transformMatrix;
    }

    Zivid::Matrix4x4 estimateArUcoMarkerPose(
        const Zivid::PointCloud &pointCloud,
        const std::vector<cv::Point2f> &markerCorners)
    {
        // Extracting 2D corners and estimating 2D center
        const auto center2D = estimate2DMarkerCenter(markerCorners);

        // Estimating 3D corners and center from 2D data
        const auto corners3D = estimate3DMarkerPoints(pointCloud, markerCorners);
        const auto center3D = estimate3DMarkerPoints(pointCloud, { center2D })[0];

        // Extracting origin and calculating normal vectors for x-, y- and z-axis
        const auto origin = cv::Vec3f(center3D.x, center3D.y, center3D.z);

        const auto xAxis = cv::Vec3f(
            corners3D[1].x - corners3D[0].x, corners3D[1].y - corners3D[0].y, corners3D[1].z - corners3D[0].z);

        const auto yAxis = cv::Vec3f(
            corners3D[2].x - corners3D[1].x, corners3D[2].y - corners3D[1].y, corners3D[2].z - corners3D[1].z);

        const auto u = xAxis / cv::norm(xAxis, cv::NORM_L2);
        const auto v = yAxis / cv::norm(yAxis, cv::NORM_L2);
        const auto normal = u.cross(v);
        const auto unitNormal = normal / cv::norm(normal, cv::NORM_L2);

        auto rotationMatrix = cv::Mat(3, 3, CV_32F);
        for(int i = 0; i < 3; ++i)
        {
            rotationMatrix.at<float>(i, 0) = u[i];
            rotationMatrix.at<float>(i, 1) = v[i];
            rotationMatrix.at<float>(i, 2) = unitNormal[i];
        }

        return transformationMatrix(rotationMatrix, origin);
    }

    cv::Mat pointCloudToGray(const Zivid::PointCloud &pointCloud)
    {
        const auto rgba = cv::Mat(pointCloud.height(), pointCloud.width(), CV_8UC4);
        pointCloud.copyData(reinterpret_cast<Zivid::ColorRGBA *>(rgba.data));
        auto bgra = cv::Mat(pointCloud.height(), pointCloud.width(), CV_8UC4);
        cv::Mat gray;
        cv::cvtColor(rgba, gray, cv::COLOR_RGBA2GRAY);
        return gray;
    }

    std::vector<Eigen::Vector3f> zividToEigen(const std::vector<Zivid::PointXYZ> &zividPoints)
    {
        std::vector<Eigen::Vector3f> eigenPoints(zividPoints.size());
        for(std::size_t i = 0; i < zividPoints.size(); i++)
        {
            eigenPoints[i] = Eigen::Vector3f{ zividPoints[i].x, zividPoints[i].y, zividPoints[i].z };
        }

        return eigenPoints;
    }

    Eigen::Affine3f zividToEigen(const Zivid::Matrix4x4 &zividMatrix)
    {
        Eigen::Matrix4f eigenMatrix;
        for(std::size_t row = 0; row < Zivid::Matrix4x4::rows; row++)
        {
            for(std::size_t column = 0; column < Zivid::Matrix4x4::cols; column++)
            {
                eigenMatrix(row, column) = zividMatrix(row, column);
            }
        }
        Eigen::Affine3f eigenTransform{ eigenMatrix };
        return eigenTransform;
    }

    std::vector<Zivid::PointXYZ> eigenToZivid(const std::vector<Eigen::Vector3f> &eigenPoints)
    {
        std::vector<Zivid::PointXYZ> zividPoints(eigenPoints.size());
        for(std::size_t i = 0; i < eigenPoints.size(); i++)
        {
            zividPoints[i] = Zivid::PointXYZ{ eigenPoints[i](0), eigenPoints[i](1), eigenPoints[i](2) };
        }

        return zividPoints;
    }

    std::vector<Zivid::PointXYZ> transformPoints(
        const std::vector<Zivid::PointXYZ> &zividPoints,
        const Zivid::Matrix4x4 &zividTransform)
    {
        std::vector<Eigen::Vector3f> eigenPoints = zividToEigen(zividPoints);
        Eigen::Affine3f eigenTransform = zividToEigen(zividTransform);

        std::vector<Eigen::Vector3f> transformedEigenPoints(eigenPoints.size());
        for(std::size_t i = 0; i < transformedEigenPoints.size(); i++)
        {
            transformedEigenPoints[i] = eigenTransform.rotation() * eigenPoints[i] + eigenTransform.translation();
        }

        std::vector<Zivid::PointXYZ> transformedZividPoints = eigenToZivid(transformedEigenPoints);

        return transformedZividPoints;
    }

    void visualizeZividPointCloud(const Zivid::Frame &frame)
    {
        Zivid::Visualization::Visualizer visualizer;

        visualizer.showMaximized();
        visualizer.show(frame);
        visualizer.resetToFit();

        std::cout << "Running visualizer. Blocking until window closes." << std::endl;
        visualizer.run();
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        const auto fileCamera = std::string(ZIVID_SAMPLE_DATA_DIR) + "/BinWithCalibrationBoard.zfc";

        std::cout << "Creating virtual camera using file: " << fileCamera << std::endl;
        auto camera = zivid.createFileCamera(fileCamera);

        auto settings = Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} } };

        const auto originalFrame = camera.capture(settings);
        auto pointCloud = originalFrame.pointCloud();

        std::cout << "Displaying the original point cloud" << std::endl;
        visualizeZividPointCloud(originalFrame);

        std::cout << "Configuring ROI box based on bin size and checkerboard placement" << std::endl;
        const float roiBoxLength = 545.F;
        const float roiBoxWidth = 345.F;
        const float roiBoxHeight = 150.F;

        // Coordinates are relative to the ArUco marker origin which lies in the center of the ArUco marker.
        // Positive x-axis is "East", y-axis is "South" and z-axis is "Down".
        const Zivid::PointXYZ roiBoxLowerRightCornerInArUcoFrame{ 240.F, 30.F, 5.F };
        const Zivid::PointXYZ roiBoxUpperRightCornerInArUcoFrame{ roiBoxLowerRightCornerInArUcoFrame.x,
                                                                  roiBoxLowerRightCornerInArUcoFrame.y - roiBoxWidth,
                                                                  roiBoxLowerRightCornerInArUcoFrame.z };
        const Zivid::PointXYZ roiBoxLowerLeftCornerInArUcoFrame{ roiBoxLowerRightCornerInArUcoFrame.x - roiBoxLength,
                                                                 roiBoxLowerRightCornerInArUcoFrame.y,
                                                                 roiBoxLowerRightCornerInArUcoFrame.z };

        const Zivid::PointXYZ pointOInArUcoFrame = roiBoxLowerRightCornerInArUcoFrame;
        const Zivid::PointXYZ pointAInArUcoFrame = roiBoxUpperRightCornerInArUcoFrame;
        const Zivid::PointXYZ pointBInArUcoFrame = roiBoxLowerLeftCornerInArUcoFrame;

        std::cout << "Configuring ArUco marker" << std::endl;
        const auto markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        cv::Ptr<cv::aruco::DetectorParameters> detectorParameters = cv::aruco::DetectorParameters::create();
        detectorParameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

        std::cout << "Converting to OpenCV image format" << std::endl;
        const auto grayImage = pointCloudToGray(pointCloud);

        std::cout << "Detecting ArUco Marker" << std::endl;
        cv::aruco::detectMarkers(grayImage, markerDictionary, markerCorners, markerIds, detectorParameters);

        if(markerIds.empty())
        {
            std::cout << "No ArUco markers detected" << std::endl;
            return EXIT_FAILURE;
        }

        std::cout << "Estimating pose of detected ArUco marker" << std::endl;
        const auto transformMarkerToCamera = estimateArUcoMarkerPose(pointCloud, markerCorners[0]);

        std::cout << "Transforming the ROI base frame points to the camera frame" << std::endl;
        const auto roiPointsInCameraFrame = transformPoints(
            std::vector<Zivid::PointXYZ>{ pointOInArUcoFrame, pointAInArUcoFrame, pointBInArUcoFrame },
            transformMarkerToCamera);

        std::cout << "Setting the ROI" << std::endl;
        settings.set(Zivid::Settings::RegionOfInterest{
            Zivid::Settings::RegionOfInterest::Box::Enabled::yes,
            Zivid::Settings::RegionOfInterest::Box::PointO{ roiPointsInCameraFrame[0] },
            Zivid::Settings::RegionOfInterest::Box::PointA{ roiPointsInCameraFrame[1] },
            Zivid::Settings::RegionOfInterest::Box::PointB{ roiPointsInCameraFrame[2] },
            Zivid::Settings::RegionOfInterest::Box::Extents{ -10, roiBoxHeight } });

        const auto roiFrame = camera.capture(settings);

        std::cout << "Displaying the ROI-filtered point cloud" << std::endl;
        visualizeZividPointCloud(roiFrame);
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
