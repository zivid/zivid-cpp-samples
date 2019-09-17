/*
Undistort an RGB image from a ZDF point cloud using Zivid camera intrinsics.
*/

#include <Zivid/CloudVisualizer.h>
#include <Zivid/Zivid.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

cv::Mat pointCloudToRGB(const Zivid::PointCloud &);
std::tuple<cv::Mat, cv::Mat> reformatCameraIntrinsics(const Zivid::CameraIntrinsics &);
void displayRGB(const cv::Mat &, const std::string &);

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to the camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Configuring the camera settings" << std::endl;
        camera << Zivid::Settings::Iris{ 21 } << Zivid::Settings::ExposureTime{ std::chrono::microseconds{ 20000 } }
               << Zivid::Settings::Gain{ 1 } << Zivid::Settings::Brightness{ 1.0 };

        std::cout << "Capturing a frame" << std::endl;
        const auto frame = camera.capture();

        std::cout << "Setting up visualization" << std::endl;
        Zivid::CloudVisualizer vis;
        zivid.setDefaultComputeDevice(vis.computeDevice());

        std::cout << "Displaying the point cloud" << std::endl;
        vis.showMaximized();
        vis.show(frame);
        vis.resetToFit();

        std::cout << "Running the visualizer. Blocking until the window closes" << std::endl;
        vis.run();

        std::cout << "Converting ZDF point cloud to OpenCV format" << std::endl;

        const auto pointCloud = frame.getPointCloud();
        const auto rgb = pointCloudToRGB(pointCloud);

        std::cout << "Undistorting the RGB image" << std::endl;

        const auto cameraIntrinsticsCV = reformatCameraIntrinsics(camera.intrinsics());
        const auto distortionCoefficients = std::get<0>(cameraIntrinsticsCV);
        const auto cameraMatrix = std::get<1>(cameraIntrinsticsCV);

        const auto size = rgb.size();
        const auto optimalCameraMatrix =
            cv::getOptimalNewCameraMatrix(cameraMatrix, distortionCoefficients, size, 1, size);

        cv::Mat rgbUndistorted;
        cv::Mat rgbUndistortedFull;

        cv::undistort(rgb, rgbUndistorted, cameraMatrix, distortionCoefficients);
        cv::undistort(rgb, rgbUndistortedFull, cameraMatrix, distortionCoefficients, optimalCameraMatrix);

        std::cout << "Displaying and saving the RGB image" << std::endl;

        displayRGB(rgb, "Distorted RGB image");
        cv::imwrite("Distorted RGB image.jpg", rgb);

        std::cout << "Displaying and saving the undistorted RGB image" << std::endl;

        displayRGB(rgbUndistorted, "Undistorted RGB image");
        cv::imwrite("Undistorted RGB image.jpg", rgbUndistorted);

        std::cout << "Displaying and saving the Undistorted RGB image - full" << std::endl;

        displayRGB(rgbUndistortedFull, "Undistorted RGB image - full");
        cv::imwrite("Undistorted RGB image - full.jpg", rgbUndistorted);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}

cv::Mat pointCloudToRGB(const Zivid::PointCloud &pointCloud)
{
    cv::Mat rgb(static_cast<int>(pointCloud.height()),
                static_cast<int>(pointCloud.width()),
                CV_8UC3,
                cv::Scalar(0, 0, 0));

    const auto height = pointCloud.height();
    const auto width = pointCloud.width();

    for(int i = 0; i < height; i++)
    {
        for(int j = 0; j < width; j++)
        {
            cv::Vec3b &color = rgb.at<cv::Vec3b>(i, j);
            color[0] = pointCloud(i, j).blue();
            color[1] = pointCloud(i, j).green();
            color[2] = pointCloud(i, j).red();
        }
    }

    return rgb;
}

std::tuple<cv::Mat, cv::Mat> reformatCameraIntrinsics(const Zivid::CameraIntrinsics &cameraIntrinsics)
{
    cv::Mat distortionCoefficients(cv::Size(1, 5), CV_64FC1, cv::Scalar(0));
    cv::Mat cameraMatrix(cv::Size(3, 3), CV_64FC1, cv::Scalar(0));

    distortionCoefficients.at<double>(0, 0) = cameraIntrinsics.distortion().k1().value();
    distortionCoefficients.at<double>(0, 1) = cameraIntrinsics.distortion().k2().value();
    distortionCoefficients.at<double>(0, 2) = cameraIntrinsics.distortion().p1().value();
    distortionCoefficients.at<double>(0, 3) = cameraIntrinsics.distortion().p2().value();
    distortionCoefficients.at<double>(0, 4) = cameraIntrinsics.distortion().k3().value();

    cameraMatrix.at<double>(0, 0) = cameraIntrinsics.cameraMatrix().fx().value();
    cameraMatrix.at<double>(0, 2) = cameraIntrinsics.cameraMatrix().cx().value();
    cameraMatrix.at<double>(1, 1) = cameraIntrinsics.cameraMatrix().fy().value();
    cameraMatrix.at<double>(1, 2) = cameraIntrinsics.cameraMatrix().cy().value();
    cameraMatrix.at<double>(2, 2) = 1;

    return std::make_tuple(distortionCoefficients, cameraMatrix);
}

void displayRGB(const cv::Mat &rgb, const std::string &rgbName)
{
    cv::namedWindow(rgbName, cv::WINDOW_AUTOSIZE);
    cv::imshow(rgbName, rgb);
    cv::waitKey(0);
}