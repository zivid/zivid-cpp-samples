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
#include <opencv2/core/core.hpp>

#include <cmath>
#include <iostream>

namespace
{
    enum class Command
    {
        cmdTransformSinglePoint,
        cmdTransformPointCloud,
        cmdUnknown
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
            return Command::cmdTransformSinglePoint;
        }
        if(command == "P" || command == "p")
        {
            return Command::cmdTransformPointCloud;
        }
        return Command::cmdUnknown;
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

    Eigen::MatrixXf cvToEigen(const cv::Mat &cvMat)
    {
        if(cvMat.dims != 2)
        {
            throw std::invalid_argument("Invalid matrix dimensions. Expected 2D.");
        }

        Eigen::MatrixXf eigenMat(cvMat.rows, cvMat.cols);
        for(int i = 0; i < cvMat.rows; i++)
        {
            for(int j = 0; j < cvMat.cols; j++)
            {
                eigenMat(i, j) = cvMat.at<float>(i, j);
            }
        }
        return eigenMat;
    }

    Zivid::Matrix4x4 cvToZivid(const cv::Mat &cvMat)
    {
        const auto zividMat = Zivid::Matrix4x4(cvMat.ptr<float>(0), cvMat.ptr<float>(0) + 16);
        return zividMat;
    }

    cv::Mat readTransform(const std::string &transformFile)
    {
        auto fileStorage = cv::FileStorage();

        if(!fileStorage.open(transformFile, cv::FileStorage::Mode::READ))
        {
            throw std::invalid_argument("Could not open " + transformFile);
        }
        try
        {
            const auto poseStateNode = fileStorage["PoseState"];

            if(poseStateNode.empty())
            {
                throw std::invalid_argument("PoseState not found in file " + transformFile);
            }

            const auto rows = poseStateNode.mat().rows;
            const auto cols = poseStateNode.mat().cols;
            if(rows != 4 || cols != 4)
            {
                throw std::invalid_argument(
                    "Expected 4x4 matrix in " + transformFile + ", but got " + std::to_string(cols) + "x"
                    + std::to_string(rows));
            }

            auto poseState = poseStateNode.mat();
            fileStorage.release();
            return poseState;
        }
        catch(...)
        {
            fileStorage.release();
            throw;
        }
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
        cv::Mat transformBaseToCameraCV;

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
                    transformBaseToCameraCV =
                        readTransform(std::string(ZIVID_SAMPLE_DATA_DIR) + eyeToHandTransformFile);

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
                    const cv::Mat transformEndEffectorToCamera =
                        readTransform(std::string(ZIVID_SAMPLE_DATA_DIR) + eyeInHandTransformFile);

                    std::cout << "Reading end-effector pose in robot base frame" << std::endl;
                    const cv::Mat transformBaseToEndEffector =
                        readTransform(std::string(ZIVID_SAMPLE_DATA_DIR) + robotTransformFile);

                    std::cout << "Computing camera pose in robot base frame" << std::endl;
                    transformBaseToCameraCV = transformBaseToEndEffector * transformEndEffectorToCamera;

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
                case Command::cmdTransformSinglePoint:
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
                    const Eigen::MatrixXf transformBaseToCamera = cvToEigen(transformBaseToCameraCV);

                    std::cout << "Transforming (picking) point from camera to robot base frame" << std::endl;
                    const Eigen::Vector4f pointInBaseFrame = transformBaseToCamera * pointInCameraFrame;

                    std::cout << "Point coordinates in robot base frame: " << pointInBaseFrame.x() << " "
                              << pointInBaseFrame.y() << " " << pointInBaseFrame.z() << std::endl;

                    loopContinue = false;
                    break;
                }
                case Command::cmdTransformPointCloud:
                {
                    std::cout << "Transforming point cloud" << std::endl;

                    const auto transformBaseToCamera = cvToZivid(transformBaseToCameraCV);
                    pointCloud.transform(transformBaseToCamera);

                    const auto saveFile = "ZividGemTransformed.zdf";
                    std::cout << "Saving frame to file: " << saveFile << std::endl;
                    frame.save(saveFile);

                    loopContinue = false;
                    break;
                }
                case Command::cmdUnknown:
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
}
