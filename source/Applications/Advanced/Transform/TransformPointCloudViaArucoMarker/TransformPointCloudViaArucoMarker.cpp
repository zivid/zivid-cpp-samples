/*
Transform a point cloud from camera to ArUco marker coordinate frame by estimating the marker's pose from the point cloud.

The ZDF file for this sample can be found under the main instructions for Zivid samples.
*/

#include <Zivid/Zivid.h>

#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cmath>
#include <iostream>

template<>
struct cv::DataType<Zivid::ColorBGRA_SRGB>
{
    using channel_type = Zivid::ColorBGRA::ValueType;
};

template<>
struct cv::traits::Type<Zivid::ColorBGRA_SRGB>
{
    static constexpr auto value = CV_MAKETYPE(DataDepth<cv::DataType<Zivid::ColorBGRA_SRGB>::channel_type>::value, 4);
};

namespace
{
    cv::Mat pointCloudToColorBGRA_SRGB(const Zivid::PointCloud &pointCloud)
    {
        auto bgra = cv::Mat(pointCloud.height(), pointCloud.width(), CV_8UC4);
        pointCloud.copyData(&(*bgra.begin<Zivid::ColorBGRA_SRGB>()));

        return bgra;
    }

    void displayBGR(const cv::Mat &bgr, const std::string &bgrName)
    {
        cv::namedWindow(bgrName, cv::WINDOW_AUTOSIZE);
        cv::imshow(bgrName, bgr);
        cv::waitKey(CI_WAITKEY_TIMEOUT_IN_MS);
    }

    cv::Mat drawDetectedMarker(
        const cv::Mat &bgraImage,
        const Zivid::Calibration::DetectionResultFiducialMarkers &detectionResult)
    {
        const auto detectedMarkerCorners = detectionResult.detectedMarkers()[0].cornersInPixelCoordinates();
        std::vector<cv::Point2f> markerCorners;
        markerCorners.reserve(detectedMarkerCorners.size());
        for(const auto &corner : detectedMarkerCorners)
        {
            markerCorners.emplace_back(corner.x, corner.y);
        }

        cv::Mat bgr;
        cv::cvtColor(bgraImage, bgr, cv::COLOR_BGRA2BGR);
        for(size_t i = 0; i < markerCorners.size(); ++i)
        {
            cv::line(bgr, markerCorners[i], markerCorners[(i + 1) % markerCorners.size()], cv::Scalar(0, 255, 0), 2);
        }

        return bgr;
    }

} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        const auto arucoMarkerFile = std::string(ZIVID_SAMPLE_DATA_DIR) + "/CalibrationBoardInCameraOrigin.zdf";
        std::cout << "Reading ZDF frame from file: " << arucoMarkerFile << std::endl;
        const auto frame = Zivid::Frame(arucoMarkerFile);
        auto pointCloud = frame.pointCloud();

        std::cout << "Configuring ArUco marker" << std::endl;
        const auto markerDictionary = Zivid::Calibration::MarkerDictionary::aruco4x4_50;
        std::vector<int> markerId = { 1 };

        std::cout << "Detecting ArUco marker" << std::endl;
        const auto detectionResult = Zivid::Calibration::detectMarkers(frame, markerId, markerDictionary);

        if(!detectionResult.valid())
        {
            std::cout << "No ArUco markers detected" << std::endl;
            return EXIT_FAILURE;
        }

        std::cout << "Converting to OpenCV image format" << std::endl;
        const auto bgraImage = pointCloudToColorBGRA_SRGB(pointCloud);

        std::cout << "Displaying detected ArUco marker" << std::endl;
        const auto bgr = drawDetectedMarker(bgraImage, detectionResult);
        displayBGR(bgr, "ArucoMarkerDetected");

        const auto bgrImageFile = "ArucoMarkerDetected.png";
        std::cout << "Saving 2D color image with detected ArUco marker to file: " << bgrImageFile << std::endl;
        cv::imwrite(bgrImageFile, bgr);

        std::cout << "Estimating pose of detected ArUco marker" << std::endl;
        const auto transformCameraToMarker = detectionResult.detectedMarkers()[0].pose().toMatrix();
        std::cout << "ArUco marker pose in camera frame:" << std::endl;
        std::cout << transformCameraToMarker << std::endl;
        std::cout << "Camera pose in ArUco marker frame:" << std::endl;
        const auto markerToCameraTransform = transformCameraToMarker.inverse();
        std::cout << markerToCameraTransform << std::endl;

        const auto transformFile = "ArUcoMarkerToCameraTransform.yaml";
        std::cout << "Saving a YAML file with Inverted ArUco marker pose to file: " << transformFile << std::endl;
        markerToCameraTransform.save(transformFile);

        std::cout << "Transforming point cloud from camera frame to ArUco marker frame" << std::endl;
        pointCloud.transform(markerToCameraTransform);

        const auto arucoMarkerTransformedFile = "CalibrationBoardInArucoMarkerOrigin.zdf";
        std::cout << "Saving transformed point cloud to file: " << arucoMarkerTransformedFile << std::endl;
        frame.save(arucoMarkerTransformedFile);
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
