/*
This example shows how to utilize the result of Eye-in-Hand calibration to transform (picking) point
coordinates from the camera frame to the robot base frame.
*/

#include <Zivid/Zivid.h>

#include <Eigen/Core>

#include <opencv2/core/core.hpp>

#include <cmath>
#include <iostream>

namespace
{
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

        // define (picking) point in camera frame
        const Eigen::Vector4f pointInCameraFrame(81.2F, 18.0F, 594.6F, 1.0F);
        std::cout << "Point coordinates in camera frame: " << pointInCameraFrame.x() << " " << pointInCameraFrame.y()
                  << " " << pointInCameraFrame.z() << std::endl;

        std::cout << "Reading camera pose in end-effector frame (result of eye-in-hand calibration)" << std::endl;
        const cv::Mat eyeInHandTransformation =
            readTransform(std::string(ZIVID_SAMPLE_DATA_DIR) + "/EyeInHandTransform.yaml");

        std::cout << "Reading end-effector pose in robot base frame" << std::endl;
        const cv::Mat endEffectorPose = readTransform(std::string(ZIVID_SAMPLE_DATA_DIR) + "/RobotTransform.yaml");

        std::cout << "Converting to Eigen matrices for easier computation" << std::endl;
        const Eigen::MatrixXf transformEndEffectorToCamera = cvToEigen(eyeInHandTransformation);
        const Eigen::MatrixXf transformBaseToEndEffector = cvToEigen(endEffectorPose);

        std::cout << "Computing camera pose in robot base frame" << std::endl;
        const Eigen::MatrixXf transformBaseToCamera = transformBaseToEndEffector * transformEndEffectorToCamera;

        std::cout << "Computing (picking) point in robot base frame" << std::endl;
        const Eigen::Vector4f pointInBaseFrame = transformBaseToCamera * pointInCameraFrame;
        std::cout << "Point coordinates in robot base frame: " << pointInBaseFrame.x() << " " << pointInBaseFrame.y()
                  << " " << pointInBaseFrame.z() << std::endl;
    }

    catch(const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
