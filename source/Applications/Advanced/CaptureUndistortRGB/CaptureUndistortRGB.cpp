/*
Undistort a BGR image from a ZDF point cloud using Zivid camera intrinsics.
*/

#include <Zivid/CloudVisualizer.h>
#include <Zivid/Zivid.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

cv::Mat pointCloudToBGR(const Zivid::PointCloud &pointCloud);
cv::Mat imageToBGR(const Zivid::Image<Zivid::RGBA8> &image);
std::tuple<cv::Mat, cv::Mat> reformatCameraIntrinsics(const Zivid::CameraIntrinsics &cameraIntrinsics);
void displayBGR(const cv::Mat &bgr, const std::string &bgrName);
std::string getInput();
cv::Mat getImage2D(Zivid::Camera &camera);
cv::Mat getImage3D(Zivid::Camera &camera, Zivid::Application &zivid);

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to the camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << R"(Enter "2d" or "3d" to select mode, then press Enter/Return to confirm)" << std::endl;
        const auto command = getInput();
        bool use2D = false;
        if(command == "2d" || command == "2D")
        {
            use2D = true;
        }

        const auto bgr = (use2D ? getImage2D(camera) : getImage3D(camera, zivid));

        std::cout << "Undistorting the BGR image" << std::endl;

        const auto cameraIntrinsticsCV = reformatCameraIntrinsics(camera.intrinsics());
        const auto &distortionCoefficients = std::get<0>(cameraIntrinsticsCV);
        const auto &cameraMatrix = std::get<1>(cameraIntrinsticsCV);

        const auto size = bgr.size();
        const auto optimalCameraMatrix =
            cv::getOptimalNewCameraMatrix(cameraMatrix, distortionCoefficients, size, 1, size);

        cv::Mat bgrUndistorted;
        cv::Mat bgrUndistortedFull;

        cv::undistort(bgr, bgrUndistorted, cameraMatrix, distortionCoefficients);
        cv::undistort(bgr, bgrUndistortedFull, cameraMatrix, distortionCoefficients, optimalCameraMatrix);

        std::cout << "Displaying and saving the BGR image" << std::endl;

        displayBGR(bgr, "Distorted BGR image");
        cv::imwrite("ImageDistorted.jpg", bgr);

        std::cout << "Displaying and saving the undistorted BGR image" << std::endl;

        displayBGR(bgrUndistorted, "Undistorted BGR image");
        cv::imwrite("ImageUndistorted.jpg", bgrUndistorted);

        std::cout << "Displaying and saving the Undistorted BGR image - full" << std::endl;

        displayBGR(bgrUndistortedFull, "Undistorted BGR image - full");
        cv::imwrite("ImageUndistortedFull.jpg", bgrUndistorted);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}

cv::Mat getImage3D(Zivid::Camera &camera, Zivid::Application &zivid)
{
    std::cout << "3D mode" << std::endl;

    std::cout << "Configuring the camera settings" << std::endl;
    camera << Zivid::Settings::Iris{ 21 } << Zivid::Settings::ExposureTime{ std::chrono::microseconds{ 20000 } }
           << Zivid::Settings::Gain{ 1 } << Zivid::Settings::Brightness{ 1.0 };

    std::cout << "Capturing a 3D frame" << std::endl;
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

    return pointCloudToBGR(pointCloud);
}

cv::Mat getImage2D(Zivid::Camera &camera)
{
    std::cout << "2D mode" << std::endl;

    std::cout << "Configuring the camera settings" << std::endl;
    Zivid::Settings2D settings = Zivid::Settings2D();
    settings.set(Zivid::Settings2D::ExposureTime{ std::chrono::microseconds{ 10000 } });
    settings.set(Zivid::Settings2D::Gain{ 2.0 });
    settings.set(Zivid::Settings2D::Iris{ 27 });
    settings.set(Zivid::Settings2D::Brightness{ 1.0 });

    std::cout << "Capturing a 2D frame" << std::endl;
    const auto frame = camera.capture2D(settings);

    std::cout << "Getting RGBA8 image from 2D frame" << std::endl;
    const auto image = frame.image<Zivid::RGBA8>();

    std::cout << "Converting RGBA8 image to OpenCV format" << std::endl;

    return imageToBGR(image);
}

std::string getInput()
{
    std::string command;
    std::getline(std::cin, command);
    return command;
}

cv::Mat imageToBGR(const Zivid::Image<Zivid::RGBA8> &image)
{
    // The cast for image.dataPtr() is required because the cv::Mat constructor requires non-const void *.
    // It does not actually mutate the data, it only adds an OpenCV header to the matrix. We then protect
    // our own instance with const.
    const cv::Mat rgbaMat(image.height(),
                          image.width(),
                          CV_8UC4, // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
                          const_cast<void *>(static_cast<const void *>(image.dataPtr())));
    cv::Mat bgr;
    cv::cvtColor(rgbaMat, bgr, cv::COLOR_RGBA2BGR);

    return bgr;
}

cv::Mat pointCloudToBGR(const Zivid::PointCloud &pointCloud)
{
    // NOLINTNEXTLINE(hicpp-signed-bitwise)
    cv::Mat bgr(pointCloud.height(), pointCloud.width(), CV_8UC3, cv::Scalar(0, 0, 0));

    const auto height = pointCloud.height();
    const auto width = pointCloud.width();

    for(size_t i = 0; i < height; i++)
    {
        for(size_t j = 0; j < width; j++)
        {
            auto &color = bgr.at<cv::Vec3b>(i, j);
            color[0] = pointCloud(i, j).blue();
            color[1] = pointCloud(i, j).green();
            color[2] = pointCloud(i, j).red();
        }
    }

    return bgr;
}

std::tuple<cv::Mat, cv::Mat> reformatCameraIntrinsics(const Zivid::CameraIntrinsics &cameraIntrinsics)
{
    cv::Mat distortionCoefficients(1, 5, CV_64FC1, cv::Scalar(0)); // NOLINT(hicpp-signed-bitwise)
    cv::Mat cameraMatrix(3, 3, CV_64FC1, cv::Scalar(0));           // NOLINT(hicpp-signed-bitwise)

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

void displayBGR(const cv::Mat &bgr, const std::string &bgrName)
{
    cv::namedWindow(bgrName, cv::WINDOW_AUTOSIZE);
    cv::imshow(bgrName, bgr);
    cv::waitKey(0);
}
