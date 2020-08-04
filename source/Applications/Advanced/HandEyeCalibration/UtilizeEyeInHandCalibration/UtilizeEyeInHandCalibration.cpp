/*
This example shows how to utilize the result of Eye-in-Hand calibration to transform either a (picking) point
coordinates or the entire point cloud from the camera frame to the robot base frame. The YAML files for this sample can
be found under the main instructions for Zivid samples.
*/

#include <Zivid/Zivid.h>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

#include <cmath>
#include <iostream>

namespace
{
    enum class CommandType
    {
        cmdTransformSinglePoint,
        cmdTransformPointCloud,
        cmdUnknown
    };

    std::string getInput()
    {
        std::string command;
        std::getline(std::cin, command);
        return command;
    }

    CommandType enterCommand()
    {
        std::cout << "Enter command, s (to transform single point) or p (to transform point cloud): ";
        const auto command = getInput();

        if(command == "S" || command == "s")
        {
            return CommandType::cmdTransformSinglePoint;
        }
        if(command == "P" || command == "p")
        {
            return CommandType::cmdTransformPointCloud;
        }
        return CommandType::cmdUnknown;
    }

    Eigen::MatrixXf cvToEigen(const cv::Mat &cvMat)
    {
        if(cvMat.dims > 2)
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
                throw std::invalid_argument("Expected 4x4 matrix in " + transformFile + ", but got "
                                            + std::to_string(cols) + "x" + std::to_string(rows));
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

        std::cout << "Reading camera pose in end-effector frame (result of eye-in-hand calibration)" << std::endl;
        const cv::Mat eyeInHandTransformation =
            readTransform(std::string(ZIVID_SAMPLE_DATA_DIR) + "/EyeInHandTransform.yaml");

        std::cout << "Reading end-effector pose in robot base frame" << std::endl;
        const cv::Mat endEffectorPose = readTransform(std::string(ZIVID_SAMPLE_DATA_DIR) + "/RobotTransform.yaml");

        const auto dataFile = std::string(ZIVID_SAMPLE_DATA_DIR) + "/ZividGem.zdf";
        std::cout << "Reading ZDF frame from file: " << dataFile << std::endl;
        const Zivid::Frame frame = Zivid::Frame(dataFile);
        auto pointCloud = frame.pointCloud();

        std::cout << "Computing camera pose in robot base frame" << std::endl;
        const cv::Mat transformBaseToCameraCV = endEffectorPose * eyeInHandTransformation;

        switch(enterCommand())
        {
            case CommandType::cmdTransformSinglePoint:
            {
                std::cout << "Transforming single point" << std::endl;

                // The (picking) point is defined as image coordinates in camera frame. It is hard-coded for the
                // ZividGem.zdf
                const size_t imageCoordinateX = 1357;
                const size_t imageCoordinateY = 666;
                const auto xyz = pointCloud.copyPointsXYZW();

                const Eigen::Vector4f pointInCameraFrame(xyz(imageCoordinateY, imageCoordinateX).x,
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
                break;
            }
            case CommandType::cmdTransformPointCloud:
            {
                std::cout << "Transforming point cloud" << std::endl;
                const auto transformationMatrix = cvToZivid(transformBaseToCameraCV);
                pointCloud.transform(transformationMatrix);

                const auto *saveFile = "ZividGemTransformed.zdf";
                std::cout << "Saving frame to file: " << saveFile << std::endl;
                frame.save(saveFile);
                break;
            }
            case CommandType::cmdUnknown:
            {
                std::cout << "Error: Unknown command" << std::endl;
                break;
            }
        }
    }

    catch(const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        if(std::cin.get() == '\n')
        {
            return EXIT_FAILURE;
        }
    }
}
