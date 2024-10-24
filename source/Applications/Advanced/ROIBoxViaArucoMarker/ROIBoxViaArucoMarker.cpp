/*
Filter the point cloud based on a ROI box given relative to the ArUco marker on a Zivid Calibration Board.

The ZFC file for this sample can be downloaded from https://support.zivid.com/en/latest/api-reference/samples/sample-data.html.
*/

#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <thread>

namespace
{
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
        const auto markerDictionary = Zivid::Calibration::MarkerDictionary::aruco4x4_50;
        std::vector<int> markerId = { 1 };

        std::cout << "Detecting ArUco marker" << std::endl;
        const auto detectionResult = Zivid::Calibration::detectMarkers(originalFrame, markerId, markerDictionary);

        if(!detectionResult.valid())
        {
            std::cout << "No ArUco markers detected" << std::endl;
            return EXIT_FAILURE;
        }

        std::cout << "Estimating pose of detected ArUco marker" << std::endl;
        const auto cameraToMarkerTransform = detectionResult.detectedMarkers()[0].pose().toMatrix();

        std::cout << "Transforming the ROI base frame points to the camera frame" << std::endl;
        const auto roiPointsInCameraFrame = transformPoints(
            std::vector<Zivid::PointXYZ>{ pointOInArUcoFrame, pointAInArUcoFrame, pointBInArUcoFrame },
            cameraToMarkerTransform);

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
