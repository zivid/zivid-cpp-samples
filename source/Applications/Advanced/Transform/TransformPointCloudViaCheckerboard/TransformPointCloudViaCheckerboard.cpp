/*
Transform a point cloud from camera to checkerboard (Zivid Calibration Board) coordinate frame by getting checkerboard pose from the API.

The ZDF file for this sample can be found under the main instructions for Zivid samples.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.
*/

#include <Zivid/Calibration/Detector.h>
#include <Zivid/Experimental/Calibration.h>
#include <Zivid/Zivid.h>

#include <opencv2/calib3d.hpp>
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
    struct CoordinateSystemPoints
    {
        cv::Point2d originPoint;
        cv::Point2d xAxisPoint;
        cv::Point2d yAxisPoint;
        cv::Point2d zAxisPoint;
    };

    cv::Mat pointCloudToColorBGRA_SRGB(const Zivid::PointCloud &pointCloud)
    {
        auto bgra = cv::Mat(pointCloud.height(), pointCloud.width(), CV_8UC4);
        pointCloud.copyData(&(*bgra.begin<Zivid::ColorBGRA_SRGB>()));

        return bgra;
    }

    void displayBGRA(const cv::Mat &bgra, const std::string &bgraName)
    {
        cv::namedWindow(bgraName, cv::WINDOW_AUTOSIZE);
        cv::imshow(bgraName, bgra);
        cv::waitKey(CI_WAITKEY_TIMEOUT_IN_MS);
    }

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
        const cv::Point3d &offsetInBoardFrame,
        const Zivid::Matrix4x4 &checkerBoardPose)
    {
        const cv::Matx33d rotationMatrix = zividPoseToOpenCVRotation(checkerBoardPose);
        const auto offsetRotated = rotationMatrix * offsetInBoardFrame;
        return {
            originInCameraFrame.x + offsetRotated.x,
            originInCameraFrame.y + offsetRotated.y,
            originInCameraFrame.z + offsetRotated.z,
        };
    }

    CoordinateSystemPoints
    getCoordinateSystemPoints(const Zivid::Frame &frame, const Zivid::Matrix4x4 &checkerboardPose, float size_of_axis)
    {
        const auto intrinsics = Zivid::Experimental::Calibration::estimateIntrinsics(frame);
        const auto cvCameraMatrix = zividCameraMatrixToOpenCVCameraMatrix(intrinsics.cameraMatrix());
        const auto cvDistCoeffs = zividDistortionCoefficientsToOpenCVDistortionCoefficients(intrinsics.distortion());
        const cv::Point3d originPosition = { checkerboardPose.at(0, 3),
                                             checkerboardPose.at(1, 3),
                                             checkerboardPose.at(2, 3) };
        const cv::Point3d xAxisDirection = movePoint(originPosition, { size_of_axis, 0.0, 0.0 }, checkerboardPose);
        const cv::Point3d yAxisDirection = movePoint(originPosition, { 0.0, size_of_axis, 0.0 }, checkerboardPose);
        const cv::Point3d zAxizDirection = movePoint(originPosition, { 0.0, 0.0, size_of_axis }, checkerboardPose);

        std::vector<cv::Point3d> pointsToProject{ originPosition, xAxisDirection, yAxisDirection, zAxizDirection };
        std::vector<cv::Point2d> projectedPoints;
        projectedPoints.reserve(4);
        const cv::Vec3d tvec{ 0, 0, 0 };
        const cv::Vec3d rvec{ 0, 0, 0 };
        cv::projectPoints(pointsToProject, rvec, tvec, cvCameraMatrix, cvDistCoeffs, projectedPoints);

        return { projectedPoints.at(0), projectedPoints.at(1), projectedPoints.at(2), projectedPoints.at(3) };
    }

    void
    drawCoordinateSystem(const Zivid::Frame &frame, const Zivid::Matrix4x4 &checkerboardPose, const cv::Mat &bgrImage)
    {
        const float sizeOfAxis = 30.0; // each axis have 30[mm] of length

        std::cout << "Aquiring frame points" << std::endl;
        auto framePoints = getCoordinateSystemPoints(frame, checkerboardPose, sizeOfAxis);

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

        const auto calibrationBoardFile = std::string(ZIVID_SAMPLE_DATA_DIR) + "/CalibrationBoardInCameraOrigin.zdf";
        std::cout << "Reading ZDF frame from file: " << calibrationBoardFile << std::endl;
        const auto frame = Zivid::Frame(calibrationBoardFile);
        auto pointCloud = frame.pointCloud();

        std::cout << "Detecting and estimating pose of the Zivid checkerboard in the camera frame" << std::endl;
        const auto detectionResult = Zivid::Calibration::detectCalibrationBoard(frame);
        const auto cameraToCheckerboardTransform = detectionResult.pose().toMatrix();
        std::cout << cameraToCheckerboardTransform << std::endl;
        std::cout << "Camera pose in checkerboard frame:" << std::endl;
        const auto checkerboardToCameraTransform = cameraToCheckerboardTransform.inverse();
        std::cout << checkerboardToCameraTransform << std::endl;

        const auto transformFile = "CheckerboardToCameraTransform.yaml";
        std::cout << "Saving camera pose in checkerboard frame to file: " << transformFile << std::endl;
        checkerboardToCameraTransform.save(transformFile);

        std::cout << "Transforming point cloud from camera frame to checkerboard frame" << std::endl;
        pointCloud.transform(checkerboardToCameraTransform);

        std::cout << "Converting to OpenCV image format" << std::endl;
        const auto bgraImage = pointCloudToColorBGRA_SRGB(pointCloud);

        std::cout << "Visualizing checkerboard with coordinate system" << std::endl;
        drawCoordinateSystem(frame, cameraToCheckerboardTransform, bgraImage);
        displayBGRA(bgraImage, "Checkerboard transformation frame");

        const auto checkerboardTransformedFile = "CalibrationBoardInCheckerboardOrigin.zdf";
        std::cout << "Saving transformed point cloud to file: " << checkerboardTransformedFile << std::endl;
        frame.save(checkerboardTransformedFile);

        std::cout << "Reading applied transformation matrix to the point cloud:" << std::endl;
        const auto transformationMatrix = pointCloud.transformationMatrix();
        std::cout << transformationMatrix << std::endl;
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
