/*
This example shows how to transform a point cloud from camera to ArUco frame using the marker's pose.
*/

#include <Zivid/Experimental/Calibration.h>
#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <aruco_nano.h>
#include <cmath>
#include <iostream>

namespace
{
    struct Line2d
    {
        double slope;
        double intercept;
    };

    int clamp(const int n, const int lower, const int upper)
    {
        return std::max(lower, std::min(n, upper));
    }

    Line2d fitLine(const cv::Point2d &point1, const cv::Point2d &point2)
    {
        // Fitting a line y=a*x + b to 2 points
        const auto slope{ (point2.y - point1.y) / (point2.x - point1.x) };
        const auto intercept{ point1.y - slope * point1.x };
        return { slope, intercept };
    };

    cv::Mat pointCloudToCvBGR(const Zivid::PointCloud &pointCloud)
    {
        auto rgb = cv::Mat(pointCloud.height(), pointCloud.width(), CV_8UC4); // NOLINT(hicpp-signed-bitwise)
        pointCloud.copyData(
            reinterpret_cast<Zivid::ColorRGBA *>(rgb.data)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        auto bgr = cv::Mat(pointCloud.height(), pointCloud.width(), CV_8UC4); // NOLINT(hicpp-signed-bitwise)
        cv::cvtColor(rgb, bgr, cv::COLOR_RGBA2BGR);

        return bgr;
    }

    std::vector<cv::Point2f> estimate2DMarkerCenters(const aruconano::Marker &marker)
    {
        if(marker.empty())
        {
            std::cout << "No markers detected";
            return {};
        }

        std::vector<cv::Point2f> markerCenters2D;
        markerCenters2D.reserve(marker.size());

        const auto markerCorner0 = marker[0];
        const auto markerCorner1 = marker[1];
        const auto markerCorner2 = marker[2];
        const auto markerCorner3 = marker[3];

        // Fitting line between two diagonal marker corners
        const auto backDiagonal{ fitLine(markerCorner2, markerCorner0) };
        const auto forwardDiagonal{ fitLine(markerCorner3, markerCorner1) };

        // Finding intersection of the two lines
        // a1*x + b1 = a2*x + b2
        // x = (b2-b1) / (a1-a2)
        // y = a1*x + b1
        const auto xCoordinate =
            (forwardDiagonal.intercept - backDiagonal.intercept) / (backDiagonal.slope - forwardDiagonal.slope);
        const auto yCoordinate = backDiagonal.slope * xCoordinate + backDiagonal.intercept;
        markerCenters2D.emplace_back(cv::Point2f(xCoordinate, yCoordinate));

        return markerCenters2D;
    }

    std::vector<cv::Point2f> get2DMarkerCorners(const aruconano::Marker &marker)
    {
        if(marker.empty())
        {
            std::cout << "No markers detected";
            return {};
        }

        std::vector<cv::Point2f> markerCorners2D;
        markerCorners2D.reserve(marker.size() * 4);

        markerCorners2D.emplace_back(marker[0]);
        markerCorners2D.emplace_back(marker[1]);
        markerCorners2D.emplace_back(marker[2]);
        markerCorners2D.emplace_back(marker[3]);

        return markerCorners2D;
    }

    std::vector<cv::Point3d> estimate3DMarkerPoints(const Zivid::PointCloud &pointCloud,
                                                    const std::vector<cv::Point2f> &markerPoints2D)
    {
        if(markerPoints2D.empty())
        {
            return {};
        }

        const int width = pointCloud.width();
        const int height = pointCloud.height();
        const auto points = pointCloud.copyPointsXYZ();

        std::vector<cv::Point3d> markerPoints3D;
        markerPoints3D.reserve(markerPoints2D.size());
        for(const auto &point2D : markerPoints2D)
        {
            // Estimating the 3D center/corners of the marker using bilinear interpolation
            // See https://en.wikipedia.org/wiki/Bilinear_interpolation
            const float x = point2D.x;
            const float y = point2D.y;

            // Getting pixel coordinates for the four known pixels next to the interpolation point
            const int xRoundedDown = clamp(static_cast<int>(std::floor(x)), 0, width - 1);
            const int xRoundedUp = clamp(static_cast<int>(std::ceil(x)), 0, width - 1);
            const int yRoundedDown = clamp(static_cast<int>(std::floor(y)), 0, height - 1);
            const int yRoundedUp = clamp(static_cast<int>(std::ceil(y)), 0, height - 1);

            // Getting 3D coordinates for the four points next to the interpolation point
            const auto q11 = points(yRoundedDown, xRoundedDown);
            const auto q12 = points(yRoundedUp, xRoundedDown);
            const auto q21 = points(yRoundedDown, xRoundedUp);
            const auto q22 = points(yRoundedUp, xRoundedUp);

            // Linear interpolation in x direction
            // f(x,y1) = (x2 - x)/(x2 - x1) * f(q11) + (x - x1)/(x2 - x1) * f(q21)
            const cv::Point3f fxy1{ (xRoundedUp - x) / (xRoundedUp - xRoundedDown) * q11.x
                                        + (x - xRoundedDown) / (xRoundedUp - xRoundedDown) * q21.x,
                                    (xRoundedUp - x) / (xRoundedUp - xRoundedDown) * q11.y
                                        + (x - xRoundedDown) / (xRoundedUp - xRoundedDown) * q21.y,
                                    (xRoundedUp - x) / (xRoundedUp - xRoundedDown) * q11.z
                                        + (x - xRoundedDown) / (xRoundedUp - xRoundedDown) * q21.z };
            // f(x,y2) = (x2 - x)/(x2 - x1) * f(q12) + (x - x1)/(x2 - x1) * f(q22)
            const cv::Point3f fxy2{ (xRoundedUp - x) / (xRoundedUp - xRoundedDown) * q12.x
                                        + (x - xRoundedDown) / (xRoundedUp - xRoundedDown) * q22.x,
                                    (xRoundedUp - x) / (xRoundedUp - xRoundedDown) * q12.y
                                        + (x - xRoundedDown) / (xRoundedUp - xRoundedDown) * q22.y,
                                    (xRoundedUp - x) / (xRoundedUp - xRoundedDown) * q12.z
                                        + (x - xRoundedDown) / (xRoundedUp - xRoundedDown) * q22.z };

            // Linear interpolation in y direction
            // f(x,y) = (y2 - y)/(y2 - y1) * f(x,y1) + (y - y1)/(y2 - y1) * f(x,y2)
            markerPoints3D.emplace_back(cv::Point3f{ (yRoundedUp - y) / (yRoundedUp - yRoundedDown) * fxy1.x
                                                         + (y - yRoundedDown) / (yRoundedUp - yRoundedDown) * fxy2.x,
                                                     (yRoundedUp - y) / (yRoundedUp - yRoundedDown) * fxy1.y
                                                         + (y - yRoundedDown) / (yRoundedUp - yRoundedDown) * fxy2.y,
                                                     (yRoundedUp - y) / (yRoundedUp - yRoundedDown) * fxy1.z
                                                         + (y - yRoundedDown) / (yRoundedUp - yRoundedDown) * fxy2.z });
        }

        return markerPoints3D;
    }

    Zivid::Matrix4x4 transformationMatrix(const cv::Matx33d &rotationMatrix, const cv::Vec3d &translationVector)
    {
        auto transformMatrix = Zivid::Matrix4x4{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

        for(size_t row = 0; row < rotationMatrix.rows; row++)
        {
            for(size_t col = 0; col < rotationMatrix.cols; col++)
            {
                transformMatrix(row, col) = rotationMatrix(row, col);
            }
        }

        transformMatrix(0, 3) = translationVector[0];
        transformMatrix(1, 3) = translationVector[1];
        transformMatrix(2, 3) = translationVector[2];

        return transformMatrix;
    }

    void displayArUcoMarkerPose(const Zivid::PointCloud &pointCloud,
                           const std::vector<cv::Point2f> &corners2D,
                           const std::vector<cv::Point2f> &center2D)
    {
        const auto bgr = pointCloudToCvBGR(pointCloud);

        auto poseCenter = center2D[0];
        auto xAxisPoint = cv::Point2f(((corners2D[3].x + corners2D[0].x) / 2), ((corners2D[3].y + corners2D[0].y) / 2));
        auto yAxisPoint = cv::Point2f(((corners2D[2].x + corners2D[3].x) / 2), ((corners2D[2].y + corners2D[3].y) / 2));

        cv::circle(bgr, poseCenter, 2, cv::Scalar(255, 0, 0), 2);
        cv::arrowedLine(bgr, poseCenter, xAxisPoint, cv::Scalar(0, 0, 255), 2);
        cv::arrowedLine(bgr, poseCenter, yAxisPoint, cv::Scalar(0, 255, 0), 2);

        cv::imshow("Pose of ArUco marker", bgr);
        cv::waitKey(0);
    }

    Zivid::Matrix4x4 getAndDisplayArUcoMarkerPose(const Zivid::PointCloud &pointCloud, const aruconano::Marker &marker)
    {
        // Extracting 2D corners and estimateing 2D center
        const auto corners2D = get2DMarkerCorners(marker);
        const auto center2D = estimate2DMarkerCenters(marker);

        // Estimating 3D corners and center from 2D data
        const auto corners3D = estimate3DMarkerPoints(pointCloud, corners2D);
        const auto center3D = estimate3DMarkerPoints(pointCloud, center2D);

        // Extracting origin and calcualting normal vectors for x-, y- and z-axis
        auto origin = cv::Vec3d{ center3D[0].x, center3D[0].y, center3D[0].z };

        auto xAxis = cv::Vec3d{ corners3D[2].x - corners3D[1].x,
                                corners3D[2].y - corners3D[1].y,
                                corners3D[2].z - corners3D[1].z };

        auto yAxis = cv::Vec3d{ corners3D[0].x - corners3D[1].x,
                                corners3D[0].y - corners3D[1].y,
                                corners3D[0].z - corners3D[1].z };

        auto u = xAxis / cv::norm(xAxis, cv::NORM_L2);
        auto v = yAxis / cv::norm(yAxis, cv::NORM_L2);
        auto normal = u.cross(v);
        auto unitNormal = normal / cv::norm(normal, cv::NORM_L2);

        auto rotationMatrix = cv::Mat(3, 3, CV_64F);
        for(int i = 0; i < 3; ++i)
        {
            rotationMatrix.at<double>(i, 0) = u[i];
            rotationMatrix.at<double>(i, 1) = v[i];
            rotationMatrix.at<double>(i, 2) = unitNormal[i];
        }

        std::cout << "Displaying ArUco marker's pose" << std::endl;
        displayArUcoMarkerPose(pointCloud, corners2D, center2D);

        return transformationMatrix(rotationMatrix, origin);
    }

    Zivid::Matrix4x4 invertAffineTransformation(const Zivid::Matrix4x4 &transform)
    {
        auto rotation = cv::Mat(3, 3, CV_32F);
        for(int i = 0; i < 3; ++i)
        {
            for(int j = 0; j < 3; ++j)
            {
                rotation.at<float>(i, j) = transform(i, j);
            }
        }

        auto translation = cv::Mat(3, 1, CV_32F);
        for(int i = 0; i < 3; ++i)
        {
            translation.at<float>(i, 0) = transform(i, 3);
        }

        cv::Mat newRotation = rotation.inv();
        cv::Mat newTranslation = -newRotation * translation;

        auto cvInverted = cv::Mat(4, 4, CV_32F, 0.0);
        for(int i = 0; i < 3; ++i)
        {
            for(int j = 0; j < 3; ++j)
            {
                cvInverted.at<float>(i, j) = newRotation.at<float>(i, j);
            }
        }
        for(int i = 0; i < 3; ++i)
        {
            cvInverted.at<float>(i, 3) = newTranslation.at<float>(i, 0);
        }
        cvInverted.at<float>(3, 3) = 1.0;

        auto *ptr = &cvInverted.at<float>(0, 0);
        auto invertedTransform = Zivid::Matrix4x4(ptr, ptr + 16);

        return invertedTransform;
    }

    void displayDetectedArUcoMarker(cv::Mat bgr, const std::vector<aruconano::Marker> markers)
    {
        for(const auto &marker : markers)
        {
            marker.draw(bgr);
        }
        cv::imshow("Detected ArUco marker", bgr);
        cv::waitKey(0);
    }

} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        const auto dataFile = std::string(ZIVID_SAMPLE_DATA_DIR) + "/ArucoSample.zdf";
        std::cout << "Reading ZDF frame from file: " << dataFile << std::endl;
        const Zivid::Frame frame = Zivid::Frame(dataFile);
        auto pointCloud = frame.pointCloud();

        std::cout << "Converting to OpenCV BGR image format" << std::endl;
        cv::Mat bgr = pointCloudToCvBGR(pointCloud);

        std::cout << "Detecting ArUco marker in 2D image" << std::endl;
        auto markers = aruconano::MarkerDetector::detect(bgr);

        std::cout << "Displaying detected ArUco marker in 2D image" << std::endl;
        displayDetectedArUcoMarker(bgr, markers);

        std::cout << "Estimating and displaying detected ArUco marker's pose" << std::endl;
        const auto transformMarkerToCamera = getAndDisplayArUcoMarkerPose(pointCloud, markers[0]);

        std::cout << "ArUco marker pose in camera frame" << std::endl;
        std::cout << transformMarkerToCamera << std::endl;

        auto transformCameraToMarker = invertAffineTransformation(transformMarkerToCamera);

        const auto *saveFile = "transformedArucoSample.zdf";
        std::cout << "Saving transformed point cloud to file " << saveFile << std::endl;
        pointCloud.transform(transformCameraToMarker);
        frame.save(saveFile);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
