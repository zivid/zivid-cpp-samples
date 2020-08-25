/*
This example shows how to mask a point cloud from ZDF file
and store the resulting point cloud in PCL format.
The ZDF file for this sample can be found under the main instructions for Zivid samples.
*/

#include <Zivid/Zivid.h>

#include <opencv2/opencv.hpp>

#include <pcl/visualization/cloud_viewer.h>

#include <iostream>

namespace
{
    void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &pointCloud)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZRGB>);
        *cloudPTR = pointCloud;
        std::cout << "Running PCL visualizer. Block until window closes" << std::endl;
        pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
        viewer.showCloud(cloudPTR);
        std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
        std::cout << "Press q to exit the viewer application" << std::endl;
        while(!viewer.wasStopped())
        {
        }
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
        cv::Mat z(pointCloud.height, pointCloud.width, CV_8UC1, cv::Scalar(0)); // NOLINT(hicpp-signed-bitwise)
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

    pcl::PointCloud<pcl::PointXYZRGB> maskPointCloud(const Zivid::PointCloud &pointCloud, const cv::Mat &mask)
    {
        const auto data = pointCloud.copyPointsXYZColorsRGBA();
        const int height = data.height();
        const int width = data.width();

        // Creating point cloud structure
        pcl::PointCloud<pcl::PointXYZRGB> maskedPointCloud(width, height);
        maskedPointCloud.is_dense = false;
        maskedPointCloud.points.resize(height * width);

        // Copying data points within the mask. Rest is set to NaN
        for(int i = 0; i < height; i++)
        {
            for(int j = 0; j < width; j++)
            {
                if(mask.at<uint8_t>(i, j) > 0)
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

        return maskedPointCloud;
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

    pcl::PointCloud<pcl::PointXYZRGB> convertToPCLPointCloud(const Zivid::PointCloud &pointCloud)
    {
        const auto data = pointCloud.copyData<Zivid::PointXYZColorRGBA>();

        // Creating PCL point cloud structure
        pcl::PointCloud<pcl::PointXYZRGB> pointCloudPCL;
        pointCloudPCL.width = pointCloud.width();
        pointCloudPCL.height = pointCloud.height();
        pointCloudPCL.is_dense = false;
        pointCloudPCL.points.resize(pointCloudPCL.width * pointCloudPCL.height);

        // Filling in point cloud data
        for(size_t i = 0; i < pointCloudPCL.points.size(); ++i)
        {
            pointCloudPCL.points[i].x = data(i).point.x; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudPCL.points[i].y = data(i).point.y; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudPCL.points[i].z = data(i).point.z; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudPCL.points[i].r = data(i).color.r; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudPCL.points[i].g = data(i).color.g; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudPCL.points[i].b = data(i).color.b; // NOLINT(cppcoreguidelines-pro-type-union-access)
        }
        return pointCloudPCL;
    }

} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::string fileName = std::string(ZIVID_SAMPLE_DATA_DIR) + "/Zivid3D.zdf";
        std::cout << "Reading ZDF frame from file: " << fileName << std::endl;
        const auto frame = Zivid::Frame(fileName);

        std::cout << "Getting point cloud from frame" << std::endl;
        const auto pointCloud = frame.pointCloud();

        const int pixelsToDisplay = 300;
        std::cout << "Generating binary mask of central " << pixelsToDisplay << " x " << pixelsToDisplay << "pixels."
                  << std::endl;
        const int height = pointCloud.height();
        const int width = pointCloud.width();
        const int heightMin = (height - pixelsToDisplay) / 2;
        const int heightMax = (height + pixelsToDisplay) / 2;
        const int widthMin = (width - pixelsToDisplay) / 2;
        const int widthMax = (width + pixelsToDisplay) / 2;
        cv::Mat mask = cv::Mat::zeros(height, width, CV_8U);
        cv::rectangle(mask,
                      cv::Point(widthMin, heightMin),
                      cv::Point(widthMax, heightMax),
                      cv::Scalar(255, 255, 255),
                      cv::FILLED);

        std::cout << "Converting to PCL point cloud" << std::endl;
        const auto pointCloudPCL = convertToPCLPointCloud(pointCloud);

        std::cout << "Displaying point cloud before masking" << std::endl;
        visualizePointCloud(pointCloudPCL);

        std::cout << "Displaying depth map before masking" << std::endl;
        visualizeDepthMap(pointCloudPCL);

        std::cout << "Masking point cloud" << std::endl;
        const pcl::PointCloud<pcl::PointXYZRGB> maskedPointCloudPCL = maskPointCloud(pointCloud, mask);

        std::cout << "Displaying point cloud after masking" << std::endl;
        visualizePointCloud(maskedPointCloudPCL);

        std::cout << "Displaying depth map after masking" << std::endl;
        visualizeDepthMap(maskedPointCloudPCL);
    }

    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }
}
