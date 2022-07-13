/*
Transform a point cloud from camera to checkerboard (Zivid Calibration Board) coordinate frame by getting checkerboard pose from the API.

The ZDF file for this sample can be found under the main instructions for Zivid samples.
*/

#include <Zivid/Experimental/Calibration.h>
#include <Zivid/Zivid.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cmath>
#include <iostream>

namespace
{
    struct CoordinateSystemPoints
    {
        cv::Point originPoint;
        cv::Point xAxisPoint;
        cv::Point yAxisPoint;
        cv::Point zAxisPoint;
    };

    cv::Mat pointCloudToColorBGR(const Zivid::PointCloud &pointCloud)
    {
        const auto rgb = cv::Mat(pointCloud.height(), pointCloud.width(), CV_8UC4);
        pointCloud.copyData(reinterpret_cast<Zivid::ColorRGBA *>(rgb.data));
        auto bgr = cv::Mat(pointCloud.height(), pointCloud.width(), CV_8UC4);
        cv::cvtColor(rgb, bgr, cv::COLOR_RGBA2BGR);

        return bgr;
    }

    void displayBGR(const cv::Mat &bgr, const std::string &bgrName)
    {
        cv::namedWindow(bgrName, cv::WINDOW_AUTOSIZE);
        cv::imshow(bgrName, bgr);
        cv::waitKey(0);
    }

    void coordinateSystemLine(
        const cv::Mat &img,
        const cv::Point &firstPoint,
        const cv::Point &secondPoint,
        const cv::Scalar &lineColor)
    {
        int lineThickness = 4;
        int lineType = cv::LineTypes::LINE_8;
        line(img, firstPoint, secondPoint, lineColor, lineThickness, lineType);
    }

    float euclideanDistance(const Zivid::PointXYZ &firstPoint, const cv::Point3f &secondPoint)
    {
        cv::Point3f
            vectorCoordinates(firstPoint.x - secondPoint.x, firstPoint.y - secondPoint.y, firstPoint.z - secondPoint.z);
        return cv::norm(vectorCoordinates);
    }

    void findPoint(
        const Zivid::Array2D<Zivid::PointXYZ> &points,
        size_t j,
        size_t i,
        const cv::Point3f &originPosition,
        float &distanceFromOrigin,
        cv::Point &point)
    {
        if(euclideanDistance(points(j, i), originPosition) < distanceFromOrigin)
        {
            point.x = i;
            point.y = j;
            distanceFromOrigin = euclideanDistance(points(j, i), originPosition);
        }
    }

    CoordinateSystemPoints getCoordinateSystemPoints(const Zivid::PointCloud &pointCloud, float size_of_axis)
    {
        const size_t width = pointCloud.width();
        const size_t height = pointCloud.height();
        const auto points = pointCloud.copyPointsXYZ();

        const cv::Point3f originPosition = { 0.0f, 0.0f, 0.0f };
        const cv::Point3f xAxisDirection = { size_of_axis, 0.0f, 0.0f };
        const cv::Point3f yAxisDirection = { 0.0f, size_of_axis, 0.0f };
        const cv::Point3f zAxizDirection = { 0.0f, 0.0f, size_of_axis };

        // the value 100, is hardcoded, we used 100 because is two order of magnitude higher than the
        // maximum spatial resolution (millimeters between points) we can achieve with your cameras
        float distanceFromOrigin = 100.F;
        float distanceFromXAxisPoint = 100.F;
        float distanceFromYAxisPoint = 100.F;
        float distanceFromZAxisPoint = 100.F;

        CoordinateSystemPoints frameCoordinates;

        for(size_t j = 0; j < height; j++)
        {
            for(size_t i = 0; i < width; i++)
            {
                findPoint(points, j, i, originPosition, distanceFromOrigin, frameCoordinates.originPoint);
                findPoint(points, j, i, xAxisDirection, distanceFromXAxisPoint, frameCoordinates.xAxisPoint);
                findPoint(points, j, i, yAxisDirection, distanceFromYAxisPoint, frameCoordinates.yAxisPoint);
                findPoint(points, j, i, zAxizDirection, distanceFromZAxisPoint, frameCoordinates.zAxisPoint);
            }
        }
        return frameCoordinates;
    }

    void drawCoordinateSystem(const Zivid::PointCloud &pointCloud, const cv::Mat &bgrImage)
    {
        const float sizeOfAxis = 30.F; // each axis have 30[mm] of length

        std::cout << "Aquiring frame points" << std::endl;
        auto framePoints = getCoordinateSystemPoints(pointCloud, sizeOfAxis);

        std::cout << "Drawing Y axis" << std::endl;
        coordinateSystemLine(bgrImage, framePoints.originPoint, framePoints.yAxisPoint, cv::Scalar(0, 255, 0));

        std::cout << "Drawing X axis" << std::endl;
        coordinateSystemLine(bgrImage, framePoints.originPoint, framePoints.xAxisPoint, cv::Scalar(0, 0, 255));

        std::cout << "Drawing Z axis" << std::endl;
        coordinateSystemLine(bgrImage, framePoints.originPoint, framePoints.zAxisPoint, cv::Scalar(255, 0, 0));
    }

    void writeTransform(const cv::Mat &transform, const std::string &transformFile)
    {
        cv::FileStorage fileStorageOut;
        if(!fileStorageOut.open(transformFile, cv::FileStorage::Mode::WRITE))
        {
            throw std::runtime_error("Could not open " + transformFile + " for writing");
        }
        fileStorageOut.write("PoseState", transform);
    }

    cv::Mat zivid4x4MatrixAsCV(const Zivid::Matrix4x4 &matrix)
    {
        return cv::Mat_<float>(4, 4, const_cast<float *>(matrix.data()));
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
        const auto detectionResult = Zivid::Calibration::detectFeaturePoints(frame.pointCloud());
        const auto transformCameraToCheckerboard = detectionResult.pose().toMatrix();
        std::cout << transformCameraToCheckerboard << std::endl;
        std::cout << "Camera pose in checkerboard frame:" << std::endl;
        const auto transformCheckerboardToCamera = transformCameraToCheckerboard.inverse();
        std::cout << transformCheckerboardToCamera << std::endl;

        const auto transformlFile = "CheckerboardToCameraTransform.yaml";
        std::cout << "Saving a YAML file with Inverted checkerboard pose to file: " << transformlFile << std::endl;
        writeTransform(zivid4x4MatrixAsCV(transformCheckerboardToCamera), transformlFile);

        std::cout << "Transforming point cloud from camera frame to Checkerboard frame" << std::endl;
        pointCloud.transform(transformCheckerboardToCamera);

        std::cout << "Converting to OpenCV image format" << std::endl;
        const auto bgrImage = pointCloudToColorBGR(pointCloud);

        std::cout << "Visualizing checkerboard with coordinate system" << std::endl;
        drawCoordinateSystem(pointCloud, bgrImage);
        displayBGR(bgrImage, "Checkerboard transformation frame");

        const auto checkerboardTransformedFile = "CalibrationBoardInCheckerboardOrigin.zdf";
        std::cout << "Saving transformed point cloud to file: " << checkerboardTransformedFile << std::endl;
        frame.save(checkerboardTransformedFile);
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
