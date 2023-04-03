/*
Transform single data point or entire point cloud from camera frame to robot base frame using Hand-Eye calibration
matrix.

This example shows how to utilize the result of Hand-Eye calibration to transform either (picking) point coordinates
or the entire point cloud from the camera frame to the robot base frame.

For both Eye-To-Hand and Eye-In-Hand, there is a Zivid gem placed approx. 500 mm away from the robot base (see below).
The (picking) point is the Zivid gem centroid, defined as image coordinates in the camera frame and hard-coded
in this code example. Open the ZDF files in Zivid Studio to inspect the gem's 2D and corresponding 3D coordinates.

Eye-To-Hand
- ZDF file: ZividGemEyeToHand.zdf
- 2D image coordinates: (1035,255)
- Corresponding 3D coordinates: (37.77 -145.92 1227.1)
- Corresponding 3D coordinates (robot base frame): (-12.4  514.37 -21.79)

Eye-In-Hand:
- ZDF file: ZividGemEyeInHand.zdf
- 2D image coordinates: (1460,755)
- Corresponding 3D coordinates (camera frame): (83.95  28.84 305.7)
- Corresponding 3D coordinates (robot base frame): (531.03  -5.44 164.6)

For verification, check that the Zivid gem centroid 3D coordinates are the same as above after the transformation.

The YAML files for this sample can be found under the main instructions for Zivid samples.
*/

#include <Zivid/Zivid.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <iostream>

namespace
{
    enum class Command
    {
        transformSinglePoint,
        transformPointCloud,
        unknown
    };

    enum class RobotCameraConfiguration
    {
        eyeToHand,
        eyeInHand,
        unknown
    };

    std::string getInput()
    {
        std::string command;
        std::getline(std::cin, command);
        return command;
    }

    Command enterCommand()
    {
        std::cout << "Enter command, s (to transform single point) or p (to transform point cloud): ";
        const auto command = getInput();

        if(command == "S" || command == "s")
        {
            return Command::transformSinglePoint;
        }
        if(command == "P" || command == "p")
        {
            return Command::transformPointCloud;
        }
        return Command::unknown;
    }

    RobotCameraConfiguration enterRobotCameraConfiguration()
    {
        std::cout << "Enter type of calibration, eth (for eye-to-hand) or eih (for eye-in-hand): ";
        const auto command = getInput();

        if(command == "eth" || command == "ETH")
        {
            return RobotCameraConfiguration::eyeToHand;
        }
        if(command == "eih" || command == "EIH")
        {
            return RobotCameraConfiguration::eyeInHand;
        }
        return RobotCameraConfiguration::unknown;
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

    Zivid::Matrix4x4 eigenToZivid(const Eigen::Affine3f &eigenTransform)
    {
        Eigen::Matrix4f eigenMatrix = eigenTransform.matrix();
        Zivid::Matrix4x4 zividMatrix;
        for(Eigen::Index row = 0; row < eigenMatrix.rows(); row++)
        {
            for(Eigen::Index column = 0; column < eigenMatrix.cols(); column++)
            {
                zividMatrix(row, column) = eigenMatrix(row, column);
            }
        }
        return zividMatrix;
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::string fileName;
        size_t imageCoordinateX = 0;
        size_t imageCoordinateY = 0;
        Zivid::Matrix4x4 transformBaseToCamera;

        bool loopContinue = true;
        while(loopContinue)
        {
            switch(enterRobotCameraConfiguration())
            {
                case RobotCameraConfiguration::eyeToHand:
                {
                    fileName = "/ZividGemEyeToHand.zdf";

                    // The (picking) point is defined as image coordinates in camera frame. It is hard-coded for the
                    // ZividGemEyeToHand.zdf (1035,255) X: 37.77 Y: -145.92 Z: 1227.1
                    imageCoordinateX = 1035;
                    imageCoordinateY = 255;

                    const auto eyeToHandTransformFile = "/EyeToHandTransform.yaml";

                    std::cout << "Reading camera pose in robot base frame (result of eye-to-hand calibration"
                              << std::endl;
                    transformBaseToCamera.load(std::string(ZIVID_SAMPLE_DATA_DIR) + eyeToHandTransformFile);

                    loopContinue = false;
                    break;
                }
                case RobotCameraConfiguration::eyeInHand:
                {
                    fileName = "/ZividGemEyeInHand.zdf";

                    // The (picking) point is defined as image coordinates in camera frame. It is hard-coded for the
                    // ZividGemEyeInHand.zdf (1460,755) X: 83.95 Y: 28.84 Z: 305.7
                    imageCoordinateX = 1357;
                    imageCoordinateY = 666;

                    const auto eyeInHandTransformFile = "/EyeInHandTransform.yaml";
                    const auto robotTransformFile = "/RobotTransform.yaml";

                    std::cout << "Reading camera pose in end-effector frame (result of eye-in-hand calibration)"
                              << std::endl;
                    Zivid::Matrix4x4 transformEndEffectorToCamera(
                        std::string(ZIVID_SAMPLE_DATA_DIR) + eyeInHandTransformFile);

                    std::cout << "Reading end-effector pose in robot base frame" << std::endl;
                    Zivid::Matrix4x4 transformBaseToEndEffector(
                        std::string(ZIVID_SAMPLE_DATA_DIR) + robotTransformFile);

                    std::cout << "Computing camera pose in robot base frame" << std::endl;
                    transformBaseToCamera = eigenToZivid(
                        zividToEigen(transformBaseToEndEffector) * zividToEigen(transformEndEffectorToCamera));

                    loopContinue = false;
                    break;
                }
                case RobotCameraConfiguration::unknown:
                {
                    std::cout << "Entered unknown Hand-Eye calibration type" << std::endl;
                    break;
                }
            }
        }

        const auto dataFile = std::string(ZIVID_SAMPLE_DATA_DIR) + fileName;
        std::cout << "Reading ZDF frame from file: " << dataFile << std::endl;
        const auto frame = Zivid::Frame(dataFile);
        auto pointCloud = frame.pointCloud();

        loopContinue = true;
        while(loopContinue)
        {
            switch(enterCommand())
            {
                case Command::transformSinglePoint:
                {
                    std::cout << "Transforming single point" << std::endl;

                    const auto xyz = pointCloud.copyPointsXYZW();

                    const Eigen::Vector4f pointInCameraFrame(
                        xyz(imageCoordinateY, imageCoordinateX).x,
                        xyz(imageCoordinateY, imageCoordinateX).y,
                        xyz(imageCoordinateY, imageCoordinateX).z,
                        xyz(imageCoordinateY, imageCoordinateX).w);

                    std::cout << "Point coordinates in camera frame: " << pointInCameraFrame.x() << " "
                              << pointInCameraFrame.y() << " " << pointInCameraFrame.z() << std::endl;

                    // Converting to Eigen matrix for easier computation
                    const Eigen::Affine3f transformBaseToCameraEigen = zividToEigen(transformBaseToCamera);

                    std::cout << "Transforming (picking) point from camera to robot base frame" << std::endl;
                    const Eigen::Vector4f pointInBaseFrame = transformBaseToCameraEigen * pointInCameraFrame;

                    std::cout << "Point coordinates in robot base frame: " << pointInBaseFrame.x() << " "
                              << pointInBaseFrame.y() << " " << pointInBaseFrame.z() << std::endl;

                    loopContinue = false;
                    break;
                }
                case Command::transformPointCloud:
                {
                    std::cout << "Transforming point cloud" << std::endl;

                    pointCloud.transform(transformBaseToCamera);

                    const auto saveFile = "ZividGemTransformed.zdf";
                    std::cout << "Saving frame to file: " << saveFile << std::endl;
                    frame.save(saveFile);

                    loopContinue = false;
                    break;
                }
                case Command::unknown:
                {
                    std::cout << "Entered unknown command" << std::endl;
                    break;
                }
            }
        }
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
