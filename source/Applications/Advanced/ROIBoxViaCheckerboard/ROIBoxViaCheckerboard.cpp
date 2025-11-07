/*
Filter the point cloud based on a ROI box given relative to the Zivid Calibration Board.

The ZFC file for this sample can be downloaded from https://support.zivid.com/en/latest/api-reference/samples/sample-data.html.
*/

#include <Zivid/Calibration/Detector.h>
#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <thread>

namespace
{
    void visualizeZividPointCloud(const Zivid::Frame &frame)
    {
        Zivid::Visualization::Visualizer visualizer;

        visualizer.showMaximized();
        visualizer.show(frame);
        visualizer.resetToFit();

        std::cout << "Running visualizer. Blocking until window closes." << std::endl;
        visualizer.run();
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
        Eigen::Affine3f affineTransform = zividToEigen(zividTransform);

        std::vector<Eigen::Vector3f> transformedEigenPoints(eigenPoints.size());
        for(std::size_t i = 0; i < transformedEigenPoints.size(); i++)
        {
            transformedEigenPoints[i] = affineTransform.rotation() * eigenPoints[i] + affineTransform.translation();
        }

        std::vector<Zivid::PointXYZ> transformedZividPoints = eigenToZivid(transformedEigenPoints);

        return transformedZividPoints;
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

        Zivid::Settings2D settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } };
        Zivid::Settings settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} },
                                  Zivid::Settings::Color{ settings2D } };

        const auto originalFrame = camera.capture2D3D(settings);
        auto pointCloud = originalFrame.pointCloud();

        std::cout << "Displaying the original point cloud" << std::endl;
        visualizeZividPointCloud(originalFrame);

        std::cout << "Configuring ROI box based on bin size and checkerboard placement" << std::endl;
        const float roiBoxLength = 545.F;
        const float roiBoxWidth = 345.F;
        const float roiBoxHeight = 150.F;

        // Coordinates are relative to the checkerboard origin which lies in the intersection between the four checkers
        // in the top-left corner of the checkerboard: Positive x-axis is "East", y-axis is "South" and z-axis is "Down"
        const Zivid::PointXYZ roiBoxLowerRightCornerInCheckerboardFrame{ 240.F, 260.F, 5.F };
        const Zivid::PointXYZ roiBoxUpperRightCornerInCheckerboardFrame{ roiBoxLowerRightCornerInCheckerboardFrame.x,
                                                                         roiBoxLowerRightCornerInCheckerboardFrame.y
                                                                             - roiBoxWidth,
                                                                         roiBoxLowerRightCornerInCheckerboardFrame.z };
        const Zivid::PointXYZ roiBoxLowerLeftCornerInCheckerboardFrame{ roiBoxLowerRightCornerInCheckerboardFrame.x
                                                                            - roiBoxLength,
                                                                        roiBoxLowerRightCornerInCheckerboardFrame.y,
                                                                        roiBoxLowerRightCornerInCheckerboardFrame.z };

        const Zivid::PointXYZ pointOInCheckerboardFrame = roiBoxLowerRightCornerInCheckerboardFrame;
        const Zivid::PointXYZ pointAInCheckerboardFrame = roiBoxUpperRightCornerInCheckerboardFrame;
        const Zivid::PointXYZ pointBInCheckerboardFrame = roiBoxLowerLeftCornerInCheckerboardFrame;

        std::cout << "Detecting and estimating pose of the Zivid checkerboard in the camera frame" << std::endl;
        const auto detectionResult = Zivid::Calibration::detectCalibrationBoard(originalFrame);

        if(!detectionResult.valid())
        {
            std::cout << "Detection failed. " << detectionResult.statusDescription() << std::endl;
            return EXIT_FAILURE;
        }

        const auto transformCameraToCheckerboard = detectionResult.pose().toMatrix();

        std::cout << "Transforming the ROI base frame points to the camera frame" << std::endl;
        const auto roiPointsInCameraFrame = transformPoints(
            std::vector<Zivid::PointXYZ>{
                pointOInCheckerboardFrame, pointAInCheckerboardFrame, pointBInCheckerboardFrame },
            transformCameraToCheckerboard);

        std::cout << "Setting the ROI" << std::endl;
        settings.set(
            Zivid::Settings::RegionOfInterest{
                Zivid::Settings::RegionOfInterest::Box::Enabled::yes,
                Zivid::Settings::RegionOfInterest::Box::PointO{ roiPointsInCameraFrame[0] },
                Zivid::Settings::RegionOfInterest::Box::PointA{ roiPointsInCameraFrame[1] },
                Zivid::Settings::RegionOfInterest::Box::PointB{ roiPointsInCameraFrame[2] },
                Zivid::Settings::RegionOfInterest::Box::Extents{ -10, roiBoxHeight } });

        const auto roiFrame = camera.capture2D3D(settings);

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
