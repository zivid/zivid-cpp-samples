/*
This example shows how to transform a point cloud from camera to ArUco Marker coordinate frame
by estimating the marker's pose from the point cloud. The ZDF file for this sample can be found
under the main instructions for Zivid samples.

This sample depends on ArUco libraries in OpenCV with extra modules (https://github.com/opencv/opencv_contrib).
*/

#include <Zivid/Experimental/Calibration.h>
#include <Zivid/Zivid.h>

#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/aruco.hpp>

#include <cmath>
#include <iostream>

namespace
{
    struct Line2d
    {
        double slope;
        double intercept;
    };

    float clamp(const float n, const float lower, const float upper)
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

    cv::Point2f estimate2DMarkerCenter(const std::vector<cv::Point2f> &markerCorners)
    {
        const auto markerCorner0 = markerCorners[0];
        const auto markerCorner1 = markerCorners[1];
        const auto markerCorner2 = markerCorners[2];
        const auto markerCorner3 = markerCorners[3];

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
        auto markerCenter = cv::Point2f(xCoordinate, yCoordinate);

        return markerCenter;
    }

    std::vector<cv::Point3d> estimate3DMarkerPoints(const Zivid::PointCloud &pointCloud,
                                                    const std::vector<cv::Point2f> &markerPoints2D)
    {
        if(markerPoints2D.empty())
        {
            return {};
        }

        const float width = pointCloud.width();
        const float height = pointCloud.height();
        const auto points = pointCloud.copyPointsXYZ();

        std::vector<cv::Point3d> markerPoints3D;
        markerPoints3D.reserve(markerPoints2D.size());

        for(const auto &point2D : markerPoints2D)
        {
            // Estimating the 3D center/corners of the marker using bilinear interpolation
            // See https://en.wikipedia.org/wiki/Bilinear_interpolation
            const auto x = point2D.x;
            const auto y = point2D.y;

            // Getting pixel coordinates for the four known pixels next to the interpolation point
            const auto xRoundedDown = clamp(std::floor(x), 0, width - 1);
            const auto xRoundedUp = clamp(std::ceil(x), 0, width - 1);
            const auto yRoundedDown = clamp(std::floor(y), 0, height - 1);
            const auto yRoundedUp = clamp(std::ceil(y), 0, height - 1);

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

    Zivid::Matrix4x4 estimateArUcoMarkerPose(const Zivid::PointCloud &pointCloud,
                                             const std::vector<cv::Point2f> &markerCorners)
    {
        // Extracting 2D corners and estimateing 2D center
        const auto center2D = estimate2DMarkerCenter(markerCorners);

        // Estimating 3D corners and center from 2D data
        const auto corners3D = estimate3DMarkerPoints(pointCloud, markerCorners);
        const auto center3D = estimate3DMarkerPoints(pointCloud, { center2D })[0];

        // Extracting origin and calculating normal vectors for x-, y- and z-axis
        const auto origin = cv::Vec3d(center3D.x, center3D.y, center3D.z);

        const auto xAxis = cv::Vec3d(corners3D[2].x - corners3D[1].x,
                                     corners3D[2].y - corners3D[1].y,
                                     corners3D[2].z - corners3D[1].z);

        const auto yAxis = cv::Vec3d(corners3D[0].x - corners3D[1].x,
                                     corners3D[0].y - corners3D[1].y,
                                     corners3D[0].z - corners3D[1].z);

        const auto u = xAxis / cv::norm(xAxis, cv::NORM_L2);
        const auto v = yAxis / cv::norm(yAxis, cv::NORM_L2);
        const auto normal = u.cross(v);
        const auto unitNormal = normal / cv::norm(normal, cv::NORM_L2);

        auto rotationMatrix = cv::Mat(3, 3, CV_64F);
        for(int i = 0; i < 3; ++i)
        {
            rotationMatrix.at<double>(i, 0) = u[i];
            rotationMatrix.at<double>(i, 1) = v[i];
            rotationMatrix.at<double>(i, 2) = unitNormal[i];
        }

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

        const cv::Mat newRotation = rotation.inv();
        const cv::Mat newTranslation = -newRotation * translation;

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

    cv::Mat pointCloudToColorBGR(const Zivid::PointCloud &pointCloud)
    {
        const auto rgb = cv::Mat(pointCloud.height(), pointCloud.width(), CV_8UC4); // NOLINT(hicpp-signed-bitwise)
        pointCloud.copyData(reinterpret_cast<Zivid::ColorRGBA *>(rgb.data));
        auto bgr = cv::Mat(pointCloud.height(), pointCloud.width(), CV_8UC4); // NOLINT(hicpp-signed-bitwise)
        cv::cvtColor(rgb, bgr, cv::COLOR_RGBA2BGR);

        return bgr;
    }

    cv::Mat colorBGRToGray(const cv::Mat &bgr)
    {
        cv::Mat gray;
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
        return gray;
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

        const auto arucoMarkerFile = std::string(ZIVID_SAMPLE_DATA_DIR) + "/ArucoMarkerInCameraOrigin.zdf";
        std::cout << "Reading ZDF frame from file: " << arucoMarkerFile << std::endl;
        const auto frame = Zivid::Frame(arucoMarkerFile);
        auto pointCloud = frame.pointCloud();

        std::cout << "Converting to OpenCV image format" << std::endl;
        const auto bgrImage = pointCloudToColorBGR(pointCloud);
        const auto grayImage = colorBGRToGray(bgrImage);

        std::cout << "Configuring ArUco marker" << std::endl;
        const auto markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        cv::Ptr<cv::aruco::DetectorParameters> detectorParameters = cv::aruco::DetectorParameters::create();
        detectorParameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

        std::cout << "Detecting ArUco Marker" << std::endl;
        cv::aruco::detectMarkers(grayImage, markerDictionary, markerCorners, markerIds, detectorParameters);

        std::cout << "Displaying detected ArUco marker" << std::endl;
        cv::aruco::drawDetectedMarkers(bgrImage, markerCorners);
        displayBGR(bgrImage, "ArucoMarkerDetected");

        if(markerIds.empty())
        {
            std::cout << "No ArUco markers detected" << std::endl;
            return EXIT_SUCCESS;
        }

        const auto *bgrImageFile = "ArucoMarkerDetected.png";
        std::cout << "Saving image with detected ArUco marker to file: " << bgrImageFile << std::endl;
        cv::imwrite(bgrImageFile, bgrImage);

        std::cout << "Estimating pose of detected ArUco marker" << std::endl;
        const auto transformMarkerToCamera = estimateArUcoMarkerPose(pointCloud, markerCorners[0]);

        std::cout << "ArUco marker pose in camera frame:" << std::endl;
        std::cout << transformMarkerToCamera << std::endl;

        const auto transformCameraToMarker = invertAffineTransformation(transformMarkerToCamera);

        std::cout << "Camera pose in ArUco marker frame:" << std::endl;
        std::cout << transformCameraToMarker << std::endl;

        std::cout << "Transforming point cloud from camera frame to ArUco marker frame" << std::endl;
        pointCloud.transform(transformCameraToMarker);

        const auto *arucoMarkerTransformedFile = "ArucoMarkerInMarkerOrigin.zdf";
        std::cout << "Saving transformed point cloud to file: " << arucoMarkerTransformedFile << std::endl;
        frame.save(arucoMarkerTransformedFile);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
