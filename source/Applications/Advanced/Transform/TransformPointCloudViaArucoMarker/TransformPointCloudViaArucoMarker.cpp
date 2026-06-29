/*
Transform a point cloud from camera to ArUco marker coordinate frame by estimating the marker's pose from the point cloud.

The ZDF file for this sample can be found under the main instructions for Zivid samples.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.

For more information on transforming point clouds, check out this tutorial:
https://support.zivid.com/en/latest/camera/academy/applications/transformations.html
*/

#include <Zivid/Experimental/Calibration.h>
#include <Zivid/Zivid.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cmath>
#include <iostream>

template<>
struct cv::DataType<Zivid::ColorBGRA_SRGB>
{
    using channel_type = Zivid::ColorBGRA_SRGB::ValueType;
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

    struct CoordinateSystemPoints
    {
        cv::Point2d originPoint;
        cv::Point2d xAxisPoint;
        cv::Point2d yAxisPoint;
        cv::Point2d zAxisPoint;
    };

    void coordinateSystemLine(
        const cv::Mat &img,
        const cv::Point2d &firstPoint,
        const cv::Point2d &secondPoint,
        const cv::Scalar &lineColor)
    {
        int lineThickness = 4;
        int lineType = cv::LineTypes::LINE_8;
        line(img, firstPoint, secondPoint, lineColor, lineThickness, lineType);
    }

    cv::Matx33d zividPoseToOpenCVRotation(const Zivid::Matrix4x4 &matrix)
    {
        cv::Matx33d cvMat;
        for(std::size_t row = 0; row < cv::Matx33d::rows; row++)
        {
            for(std::size_t column = 0; column < cv::Matx33d::cols; column++)
            {
                cvMat(row, column) = matrix(row, column);
            }
        }
        return cvMat;
    }

    cv::Matx33d zividCameraMatrixToOpenCVCameraMatrix(const Zivid::CameraIntrinsics::CameraMatrix &cameraMatrix)
    {
        return { cameraMatrix.fx().value(),
                 0.0,
                 cameraMatrix.cx().value(),
                 0.0,
                 cameraMatrix.fy().value(),
                 cameraMatrix.cy().value(),
                 0.0,
                 0.0,
                 1 };
    }

    std::vector<double> zividDistortionCoefficientsToOpenCVDistortionCoefficients(
        const Zivid::CameraIntrinsics::Distortion &distortionCoeffs)
    {
        return { distortionCoeffs.k1().value(),
                 distortionCoeffs.k2().value(),
                 distortionCoeffs.p1().value(),
                 distortionCoeffs.p2().value(),
                 distortionCoeffs.k3().value() };
    }

    cv::Point3d movePoint(
        const cv::Point3d &originInCameraFrame,
        const cv::Point3d &offsetInMarkerFrame,
        const Zivid::Matrix4x4 &markerPose)
    {
        const cv::Matx33d rotationMatrix = zividPoseToOpenCVRotation(markerPose);
        const auto offsetRotated = rotationMatrix * offsetInMarkerFrame;
        return {
            originInCameraFrame.x + offsetRotated.x,
            originInCameraFrame.y + offsetRotated.y,
            originInCameraFrame.z + offsetRotated.z,
        };
    }

    CoordinateSystemPoints
    getCoordinateSystemPoints(const Zivid::Frame &frame, const Zivid::Matrix4x4 &markerPose, float sizeOfAxis)
    {
        const auto intrinsics = Zivid::Experimental::Calibration::estimateIntrinsics(frame);
        const auto cvCameraMatrix = zividCameraMatrixToOpenCVCameraMatrix(intrinsics.cameraMatrix());
        const auto cvDistCoeffs = zividDistortionCoefficientsToOpenCVDistortionCoefficients(intrinsics.distortion());
        const cv::Point3d originPosition = { markerPose.at(0, 3), markerPose.at(1, 3), markerPose.at(2, 3) };
        const cv::Point3d xAxisDirection = movePoint(originPosition, { sizeOfAxis, 0.0, 0.0 }, markerPose);
        const cv::Point3d yAxisDirection = movePoint(originPosition, { 0.0, sizeOfAxis, 0.0 }, markerPose);
        const cv::Point3d zAxisDirection = movePoint(originPosition, { 0.0, 0.0, sizeOfAxis }, markerPose);

        std::vector<cv::Point3d> pointsToProject{ originPosition, xAxisDirection, yAxisDirection, zAxisDirection };
        std::vector<cv::Point2d> projectedPoints;
        projectedPoints.reserve(4);
        const cv::Vec3d tvec{ 0, 0, 0 };
        const cv::Vec3d rvec{ 0, 0, 0 };
        cv::projectPoints(pointsToProject, rvec, tvec, cvCameraMatrix, cvDistCoeffs, projectedPoints);

        return { projectedPoints.at(0), projectedPoints.at(1), projectedPoints.at(2), projectedPoints.at(3) };
    }

    void drawCoordinateSystem(const Zivid::Frame &frame, const Zivid::Matrix4x4 &markerPose, const cv::Mat &bgrImage)
    {
        const float sizeOfAxis = 30.0; // each axis has 30[mm] of length

        std::cout << "Acquiring frame points" << std::endl;
        auto framePoints = getCoordinateSystemPoints(frame, markerPose, sizeOfAxis);

        std::cout << "Drawing Z axis" << std::endl;
        coordinateSystemLine(bgrImage, framePoints.originPoint, framePoints.zAxisPoint, cv::Scalar(255, 0, 0));

        std::cout << "Drawing Y axis" << std::endl;
        coordinateSystemLine(bgrImage, framePoints.originPoint, framePoints.yAxisPoint, cv::Scalar(0, 255, 0));

        std::cout << "Drawing X axis" << std::endl;
        coordinateSystemLine(bgrImage, framePoints.originPoint, framePoints.xAxisPoint, cv::Scalar(0, 0, 255));
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
        const auto cameraToMarkerTransform = detectionResult.detectedMarkers()[0].pose().toMatrix();
        std::cout << "ArUco marker pose in camera frame:" << std::endl;
        std::cout << cameraToMarkerTransform << std::endl;
        std::cout << "Camera pose in ArUco marker frame:" << std::endl;
        const auto markerToCameraTransform = cameraToMarkerTransform.inverse();
        std::cout << markerToCameraTransform << std::endl;

        std::cout << "Visualizing ArUco marker with coordinate system" << std::endl;
        cv::Mat bgrCoordinateSystem;
        cv::cvtColor(bgraImage, bgrCoordinateSystem, cv::COLOR_BGRA2BGR);
        drawCoordinateSystem(frame, cameraToMarkerTransform, bgrCoordinateSystem);
        displayBGR(bgrCoordinateSystem, "ArUco marker transformation frame");

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
