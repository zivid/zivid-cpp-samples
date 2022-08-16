/*
Filter the point cloud based on a ROI box given relative to the Zivid Calibration Board.

The ZDF file for this sample can be found under the main instructions for Zivid samples.
*/

#include <Zivid/Experimental/Calibration.h>
#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <thread>

namespace
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr roiBoxPointCloud(
        const Zivid::PointCloud &pointCloud,
        float roiBoxBottomLeftCornerX,
        float roiBoxBottomLeftCornerY,
        float roiBoxBottomLeftCornerZ,
        float boxDimensionInAxisX,
        float boxDimensionInAxisY,
        float boxDimensionInAxisZ)
    {
        const auto data = pointCloud.copyPointsXYZColorsRGBA();
        int height = data.height();
        int width = data.width();

        // Creating point cloud structure
        pcl::PointCloud<pcl::PointXYZRGB> maskedPointCloud(width, height);
        maskedPointCloud.is_dense = false;
        maskedPointCloud.points.resize(height * width);
        int marginOfBoxROI = 10;

        // Copying data points within the mask. Rest is set to NaN
        for(int i = 0; i < height; i++)
        {
            for(int j = 0; j < width; j++)
            {
                if(data(i, j).point.x > roiBoxBottomLeftCornerX - marginOfBoxROI
                   && data(i, j).point.x < roiBoxBottomLeftCornerX + boxDimensionInAxisX + marginOfBoxROI
                   && data(i, j).point.y < roiBoxBottomLeftCornerY + marginOfBoxROI
                   && data(i, j).point.y > roiBoxBottomLeftCornerY - boxDimensionInAxisY - marginOfBoxROI
                   && data(i, j).point.z < roiBoxBottomLeftCornerZ + marginOfBoxROI
                   && data(i, j).point.z > roiBoxBottomLeftCornerZ - boxDimensionInAxisZ - marginOfBoxROI)
                {
                    maskedPointCloud(j, i).r = data(i, j).color.r;
                    maskedPointCloud(j, i).g = data(i, j).color.g;
                    maskedPointCloud(j, i).b = data(i, j).color.b;
                    maskedPointCloud(j, i).x = data(i, j).point.x;
                    maskedPointCloud(j, i).y = data(i, j).point.y;
                    maskedPointCloud(j, i).z = data(i, j).point.z;
                }
                else
                {
                    maskedPointCloud(j, i).r = 0;
                    maskedPointCloud(j, i).g = 0;
                    maskedPointCloud(j, i).b = 0;
                    maskedPointCloud(j, i).x = NAN;
                    maskedPointCloud(j, i).y = NAN;
                    maskedPointCloud(j, i).z = NAN;
                }
            }
        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZRGB>);
        *cloudPTR = maskedPointCloud;
        return cloudPTR;
    }

    cv::Mat pointCloudToCvZ(const pcl::PointCloud<pcl::PointXYZRGB> &pointCloud)
    {
        // Getting min and max values for X, Y, Z images
        float zMax = -1;
        float zMin = 1000000;
        for(size_t i = 0; i < pointCloud.height; i++)
        {
            for(size_t j = 0; j < pointCloud.width; j++)
            {
                zMax = std::max(zMax, pointCloud(j, i).z);
                zMin = std::min(zMin, pointCloud(j, i).z);
            }
        }

        // Filling in OpenCV matrix with the cloud data
        cv::Mat z(pointCloud.height, pointCloud.width, CV_8UC1, cv::Scalar(0));
        for(size_t i = 0; i < pointCloud.height; i++)
        {
            for(size_t j = 0; j < pointCloud.width; j++)
            {
                if(std::isnan(pointCloud(j, i).z))
                {
                    z.at<uint8_t>(i, j) = 0;
                }
                else
                {
                    // If few points are captured resulting in zMin == zMax, this will throw an division-by-zero
                    // exception.
                    z.at<uint8_t>(i, j) = static_cast<uint8_t>((255.0F * (pointCloud(j, i).z - zMin) / (zMax - zMin)));
                }
            }
        }

        // Applying color map
        cv::Mat zColorMap;
        cv::applyColorMap(z, zColorMap, cv::COLORMAP_VIRIDIS);

        // Setting invalid points (nan) to black
        for(size_t i = 0; i < pointCloud.height; i++)
        {
            for(size_t j = 0; j < pointCloud.width; j++)
            {
                if(std::isnan(pointCloud(j, i).z))
                {
                    auto &zRGB = zColorMap.at<cv::Vec3b>(i, j);
                    zRGB[0] = 0;
                    zRGB[1] = 0;
                    zRGB[2] = 0;
                }
            }
        }
        return zColorMap;
    }

    void visualizeDepthMap(const pcl::PointCloud<pcl::PointXYZRGB> &pointCloud)
    {
        // Converting to Depth map in OpenCV format
        cv::Mat zColorMap = pointCloudToCvZ(pointCloud);
        // Visualizing Depth map
        cv::namedWindow("Depth map", cv::WINDOW_AUTOSIZE);
        cv::imshow("Depth map", zColorMap);
        cv::waitKey(0);
    }

    void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pointCloud)
    {
        auto viewer = pcl::visualization::PCLVisualizer("Viewer");

        viewer.addPointCloud<pcl::PointXYZRGB>(pointCloud);
        viewer.setCameraPosition(0, 0, 100, 0, -1, 0);

        std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
        std::cout << "Press q to exit the viewer application" << std::endl;
        while(!viewer.wasStopped())
        {
            viewer.spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void visualizeZividPointCloud(const Zivid::Frame &frame)
    {
        std::cout << "Setting up visualization" << std::endl;
        Zivid::Visualization::Visualizer visualizer;

        std::cout << "Visualizing point cloud" << std::endl;
        visualizer.showMaximized();
        visualizer.show(frame);
        visualizer.resetToFit();

        std::cout << "Running visualizer. Blocking  until window closes" << std::endl;
        visualizer.run();
    }

} // namespace
int main()
{
    try
    {
        Zivid::Application zivid;

        const auto fileData = std::string(ZIVID_SAMPLE_DATA_DIR) + "/BinWithCalibrationBoard.zdf";
        std::cout << "Reading ZDF frame from file: " << fileData << std::endl;
        const auto frame = Zivid::Frame(fileData);
        auto pointCloud = frame.pointCloud();

        std::cout << "Displaying the original point cloud" << std::endl;
        visualizeZividPointCloud(frame);

        std::cout << "Detecting and estimating pose of the Zivid checkerboard in the camera frame" << std::endl;
        const auto detectionResult = Zivid::Calibration::detectFeaturePoints(pointCloud);
        const auto transformCameraToCheckerboard = detectionResult.pose().toMatrix();

        std::cout << "Camera pose in checkerboard frame:" << std::endl;
        const auto transformCheckerboardToCamera = transformCameraToCheckerboard.inverse();
        std::cout << transformCheckerboardToCamera << std::endl;

        std::cout << "Transforming point cloud from camera frame to Checkerboard frame" << std::endl;
        pointCloud.transform(transformCheckerboardToCamera);

        std::cout << "Bottom-Left ROI Box corner:" << std::endl;
        const float roiBoxBottomLeftCornerX = -80.F; // Positive is "East"
        const float roiBoxBottomLeftCornerY = 280.F; // Positive is "South"
        const float roiBoxBottomLeftCornerZ = 5.F;   // Positive is "Down"
        std::cout << "X: " << roiBoxBottomLeftCornerX << std::endl
                  << "Y: " << roiBoxBottomLeftCornerY << std::endl
                  << "Z: " << roiBoxBottomLeftCornerZ << std::endl;

        std::cout << "ROI Box size:" << std::endl;
        const float roiBoxLength = 600.F;
        const float roiBoxWidth = 400.F;
        const float roiBoxHeight = 80.F;
        std::cout << "Length: " << roiBoxLength << std::endl
                  << "Width: " << roiBoxWidth << std::endl
                  << "Height: " << roiBoxHeight << std::endl;

        std::cout << "Filtering the point cloud based on ROI Box" << std::endl;
        const auto roiPointCloudPCL = roiBoxPointCloud(
            pointCloud,
            roiBoxBottomLeftCornerX,
            roiBoxBottomLeftCornerY,
            roiBoxBottomLeftCornerZ,
            roiBoxLength,
            roiBoxWidth,
            roiBoxHeight);

        std::cout << "Displaying transformed point cloud after ROI Box filtering" << std::endl;
        visualizePointCloud(roiPointCloudPCL);

        std::cout << "Displaying depth map of the transformed point cloud after ROI Box filtering" << std::endl;
        visualizeDepthMap(*roiPointCloudPCL);
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
