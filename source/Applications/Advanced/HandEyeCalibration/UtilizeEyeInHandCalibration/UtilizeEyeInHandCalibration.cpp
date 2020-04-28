/*
Utilize the result of eye-in-hand calibration to transform (picking) point
coordinates from the camera frame to the robot base frame.
*/

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

#include <cmath>
#include <iostream>

Eigen::MatrixXd cvToEigen(const cv::Mat &cvMat);
cv::Mat readTransform(const std::string &fileName);

int main()
{
    try
    {
        // define (picking) point in camera frame
        const Eigen::Vector4d pointInCameraFrame(81.2, 18.0, 594.6, 1);
        std::cout << "Point coordinates in camera frame: " << pointInCameraFrame.segment(0, 3).transpose() << std::endl;

        // Read camera pose in end-effector frame (result of eye-in-hand calibration)
        const auto eyeInHandTransformation = readTransform("handEyeTransform.yaml");

        // Read end-effector pose in robot base frame
        const auto endEffectorPose = readTransform("robotTransform.yaml");

        // convert to Eigen matrices for easier computation
        const auto transformEndEffectorToCamera = cvToEigen(eyeInHandTransformation);
        const auto transformBaseToEndEffector = cvToEigen(endEffectorPose);

        // Compute camera pose in robot base frame
        const auto transformBaseToCamera = transformBaseToEndEffector * transformEndEffectorToCamera;

        // compute (picking) point in robot base frame
        const auto pointInBaseFrame = transformBaseToCamera * pointInCameraFrame;
        std::cout << "Point coordinates in robot base frame: " << pointInBaseFrame.segment(0, 3).transpose()
                  << std::endl;
    }

    catch(const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}

Eigen::MatrixXd cvToEigen(const cv::Mat &cvMat)
{
    if(cvMat.dims > 2)
    {
        throw std::invalid_argument("Invalid matrix dimensions. Expected 2D.");
    }

    Eigen::MatrixXd eigenMat(cvMat.rows, cvMat.cols);

    for(int i = 0; i < cvMat.rows; i++)
    {
        for(int j = 0; j < cvMat.cols; j++)
        {
            eigenMat(i, j) = cvMat.at<double>(i, j);
        }
    }

    return eigenMat;
}

cv::Mat readTransform(const std::string &fileName)
{
    auto fileStorage = cv::FileStorage();

    if(!fileStorage.open(fileName, cv::FileStorage::Mode::READ))
    {
        throw std::invalid_argument("Could not open " + fileName);
    }
    try
    {
        const auto poseStateNode = fileStorage["PoseState"];

        if(poseStateNode.empty())
        {
            throw std::invalid_argument("PoseState not found in file " + fileName);
        }

        const auto rows = poseStateNode.mat().rows;
        const auto cols = poseStateNode.mat().cols;
        if(rows != 4 || cols != 4)
        {
            throw std::invalid_argument("Expected 4x4 matrix in " + fileName + ", but got " + std::to_string(cols) + "x"
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