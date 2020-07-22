/*
This example shows how to use camera intrinsics to undistort an RGB image.

The example will prompt the user for whether to capture an image (2D) or a point cloud (3D).
In both instances it will operate on an RGB image. However, in the 3D case it will extract
the RGB image from the point cloud. The 2D variant is faster.
*/

#include <Zivid/Experimental/Calibration.h>
#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

namespace
{
    struct CameraIntrinsicsCV
    {
        cv::Mat distortionCoefficients;
        cv::Mat cameraMatrix;
    };

    Zivid::Settings makeSettings(const std::chrono::microseconds exposureTime,
                                 const double aperture,
                                 const double gain,
                                 const double brightness)
    {
        Zivid::Settings settings = Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{
            Zivid::Settings::Acquisition::ExposureTime{ exposureTime },
            Zivid::Settings::Acquisition::Aperture{ aperture },
            Zivid::Settings::Acquisition::Gain{ gain },
            Zivid::Settings::Acquisition::Brightness{ brightness },
        } } };
        std::cout << settings << std::endl;
        return settings;
    }

    Zivid::Settings2D makeSettings2D(const std::chrono::microseconds exposureTime,
                                     const double aperture,
                                     const double gain,
                                     const double brightness)
    {
        Zivid::Settings2D settings2D =
            Zivid::Settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{
                Zivid::Settings2D::Acquisition::ExposureTime{ exposureTime },
                Zivid::Settings2D::Acquisition::Aperture{ aperture },
                Zivid::Settings2D::Acquisition::Gain{ gain },
                Zivid::Settings2D::Acquisition::Brightness{ brightness },
            } } };
        return settings2D;
    }

    cv::Mat imageToBGR(const Zivid::Image<Zivid::ColorRGBA> &image)
    {
        // The cast for image.data() is required because the cv::Mat constructor requires non-const void *.
        // It does not actually mutate the data, it only adds an OpenCV header to the matrix. We then protect
        // our own instance with const.
        const cv::Mat rgbaMat(image.height(),
                              image.width(),
                              CV_8UC4, // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
                              const_cast<void *>(static_cast<const void *>(image.data())));
        cv::Mat bgr;
        cv::cvtColor(rgbaMat, bgr, cv::COLOR_RGBA2BGR);

        return bgr;
    }

    cv::Mat getImage3D(Zivid::Camera &camera, const Zivid::Settings &settings)
    {
        std::cout << "3D mode" << std::endl;

        std::cout << "Capturing frame" << std::endl;
        const auto frame = camera.capture(settings);

        std::cout << "Setting up visualization" << std::endl;
        Zivid::Visualization::Visualizer visualizer;

        std::cout << "Visualizing point cloud" << std::endl;
        visualizer.showMaximized();
        visualizer.show(frame);
        visualizer.resetToFit();

        std::cout << "Running visualizer. Blocking until window closes" << std::endl;
        visualizer.run();

        std::cout << "Converting to OpenCV BGR image" << std::endl;
        const auto image = frame.pointCloud().copyImageRGBA();

        return imageToBGR(image);
    }

    cv::Mat getImage2D(Zivid::Camera &camera, const Zivid::Settings2D &settings)
    {
        std::cout << "2D mode" << std::endl;

        std::cout << "Capturing 2D frame" << std::endl;
        const auto frame2D = camera.capture(settings);

        std::cout << "Getting RGBA image" << std::endl;
        const auto image = frame2D.imageRGBA();

        std::cout << "Converting to OpenCV BGR image" << std::endl;

        return imageToBGR(image);
    }

    std::string getInput()
    {
        std::string command;
        std::getline(std::cin, command);
        return command;
    }

    CameraIntrinsicsCV reformatCameraIntrinsics(const Zivid::CameraIntrinsics &cameraIntrinsics)
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

        CameraIntrinsicsCV cameraIntrinsicsCV;
        cameraIntrinsicsCV.distortionCoefficients = distortionCoefficients;
        cameraIntrinsicsCV.cameraMatrix = cameraMatrix;

        return cameraIntrinsicsCV;
    }

    void displayBGR(const cv::Mat &bgr, const std::string &bgrName)
    {
        cv::namedWindow(bgrName, cv::WINDOW_AUTOSIZE);
        cv::imshow(bgrName, bgr);
        cv::waitKey(0);
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << R"(Enter "2d" or "3d" to select mode, then press Enter/Return to confirm)" << std::endl;
        const auto command = getInput();
        bool use2D = false;
        if(command == "2d" || command == "2D")
        {
            use2D = true;
        }

        const auto exposureTime = std::chrono::microseconds{ 20000 };
        const auto aperture = 5.66;
        const auto gain = 1.0;
        const auto brightness = 1.0;

        const auto bgr = use2D ? getImage2D(camera, makeSettings2D(exposureTime, aperture, gain, brightness))
                               : getImage3D(camera, makeSettings(exposureTime, aperture, gain, brightness));

        std::cout << "Undistorting BGR image" << std::endl;

        const auto cameraIntrinsticsCV = reformatCameraIntrinsics(Zivid::Experimental::Calibration::intrinsics(camera));
        const auto distortionCoefficients = cameraIntrinsticsCV.distortionCoefficients;
        const auto cameraMatrix = cameraIntrinsticsCV.cameraMatrix;

        const auto size = bgr.size();
        const auto optimalCameraMatrix =
            cv::getOptimalNewCameraMatrix(cameraMatrix, distortionCoefficients, size, 1, size);

        cv::Mat bgrUndistorted;
        cv::Mat bgrUndistortedFull;

        cv::undistort(bgr, bgrUndistorted, cameraMatrix, distortionCoefficients);
        cv::undistort(bgr, bgrUndistortedFull, cameraMatrix, distortionCoefficients, optimalCameraMatrix);

        const auto *imageDistortedFile = "ImageDistorted.jpg";
        displayBGR(bgr, "Distorted BGR image");
        std::cout << "Visualizing and saving BGR image to file: " << imageDistortedFile << std::endl;
        cv::imwrite(imageDistortedFile, bgr);

        const auto *imageUndistorted = "ImageUnistorted.jpg";
        displayBGR(bgrUndistorted, "Undistorted BGR image");
        std::cout << "Visualizing and saving undistorted BGR image to file: " << imageUndistorted << std::endl;
        cv::imwrite(imageUndistorted, bgrUndistorted);

        const auto *imageUndistortedFull = "ImageUnistortedFull.jpg";
        displayBGR(bgrUndistortedFull, "Undistorted BGR image - full");
        std::cout << "Visualizing and saving undistorted BGR image (full) to file: " << imageUndistortedFull
                  << std::endl;
        cv::imwrite(imageUndistortedFull, bgrUndistorted);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        if(std::cin.get() == '\n')
        {
            return EXIT_FAILURE;
        }
    }
}
