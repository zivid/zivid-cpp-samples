/*
Convert point cloud from a ZDF file to OpenCV format, extract depth map and visualize it. 

This example shows how to convert point cloud from a ZDF file to OpenCV format, then extract and visualize depth map. The
ZDF file for this sample can be found under the main instructions for Zivid samples.
*/

#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>

namespace
{
    float getValueZ(const Zivid::PointZ &p)
    {
        return p.z;
    }

    bool isLesserOrNan(const Zivid::PointZ &a, const Zivid::PointZ &b)
    {
        if(std::isnan(getValueZ(a)) && std::isnan(getValueZ(b)))
        {
            return false;
        }
        return getValueZ(a) < getValueZ(b) ? true : std::isnan(getValueZ(a));
    }

    bool isGreaterOrNaN(const Zivid::PointZ &a, const Zivid::PointZ &b)
    {
        if(std::isnan(getValueZ(a)) && std::isnan(getValueZ(b)))
        {
            return false;
        }
        return getValueZ(a) > getValueZ(b) ? true : std::isnan(getValueZ(a));
    }

    void visualizePointCloud(const Zivid::PointCloud &pointCloud)
    {
        std::cout << "Setting up visualization" << std::endl;
        Zivid::Visualization::Visualizer visualizer;

        std::cout << "Visualizing point cloud" << std::endl;
        visualizer.showMaximized();
        visualizer.show(pointCloud);
        visualizer.resetToFit();

        std::cout << "Running visualizer. Blocking until window closes." << std::endl;
        visualizer.run();
    }

    cv::Mat pointCloudToCvZ(const Zivid::PointCloud &pointCloud)
    {
        cv::Mat z(pointCloud.height(), pointCloud.width(), CV_8UC1, cv::Scalar(0)); // NOLINT(hicpp-signed-bitwise)
        const auto points = pointCloud.copyPointsZ();

        // Getting min and max values for X, Y, Z images
        const auto *maxZ = std::max_element(points.data(), points.data() + pointCloud.size(), isLesserOrNan);
        const auto *minZ = std::max_element(points.data(), points.data() + pointCloud.size(), isGreaterOrNaN);

        // Filling in OpenCV matrix with the cloud data
        for(size_t i = 0; i < pointCloud.height(); i++)
        {
            for(size_t j = 0; j < pointCloud.width(); j++)
            {
                if(std::isnan(points(i, j).z))
                {
                    z.at<uchar>(i, j) = 0;
                }
                else
                {
                    z.at<uchar>(i, j) =
                        static_cast<unsigned char>((255.0F * (points(i, j).z - minZ->z) / (maxZ->z - minZ->z)));
                }
            }
        }

        // Applying color map
        cv::Mat zColorMap;
        cv::applyColorMap(z, zColorMap, cv::COLORMAP_VIRIDIS);

        // Setting invalid points (nan) to black
        for(size_t i = 0; i < pointCloud.height(); i++)
        {
            for(size_t j = 0; j < pointCloud.width(); j++)
            {
                if(std::isnan(points(i, j).z))
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


    cv::Mat pointCloudToCvBGR(const Zivid::PointCloud &pointCloud)
    {
        auto rgb = cv::Mat(pointCloud.height(), pointCloud.width(), CV_8UC4);
        pointCloud.copyData(reinterpret_cast<Zivid::ColorRGBA *>(rgb.data));
        auto bgr = cv::Mat(pointCloud.height(), pointCloud.width(), CV_8UC4);
        cv::cvtColor(rgb, bgr, cv::COLOR_BGR2RGB);

        return bgr;
    }


} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        const auto dataFile = std::string(ZIVID_SAMPLE_DATA_DIR) + "/Zivid3D.zdf";
        std::cout << "Reading " << dataFile << " point cloud" << std::endl;
        const auto pointCloud = Zivid::Frame(dataFile).pointCloud();

        visualizePointCloud(pointCloud);

        std::cout << "Converting to BGR image in OpenCV format" << std::endl;
        cv::Mat bgr = pointCloudToCvBGR(pointCloud);

        const auto bgrImageFile = "Image.png";
        std::cout << "Visualizing and saving BGR image to file: " << bgrImageFile << std::endl;
        cv::namedWindow("BGR image", cv::WINDOW_AUTOSIZE);
        cv::imshow("BGR image", bgr);
        cv::waitKey(0);
        cv::imwrite(bgrImageFile, bgr);

        std::cout << "Converting to Depth map in OpenCV format" << std::endl;
        cv::Mat zColorMap = pointCloudToCvZ(pointCloud);

        const auto depthMapFile = "DepthMap.png";
        std::cout << "Visualizing and saving Depth map to file: " << depthMapFile << std::endl;
        cv::namedWindow("Depth map", cv::WINDOW_AUTOSIZE);
        cv::imshow("Depth map", zColorMap);
        cv::waitKey(0);
        cv::imwrite(depthMapFile, zColorMap);
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
