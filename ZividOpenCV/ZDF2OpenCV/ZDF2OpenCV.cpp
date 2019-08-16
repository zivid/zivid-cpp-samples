/*
Import a ZDF point cloud and convert it to OpenCV format.
*/

#include <Zivid/CloudVisualizer.h>
#include <Zivid/Zivid.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>

enum class Axis
{
    X,
    Y,
    Z
};

template<Axis axis>
static float getValue(const Zivid::Point &p);

template<>
static float getValue<Axis::X>(const Zivid::Point &p)
{
    return p.x;
}

template<>
static float getValue<Axis::Y>(const Zivid::Point &p)
{
    return p.y;
}

template<>
static float getValue<Axis::Z>(const Zivid::Point &p)
{
    return p.z;
}

template<Axis axis>
static bool isLesserOrNan(const Zivid::Point &a, const Zivid::Point &b)
{
    return getValue<axis>(a) < getValue<axis>(b) ? true : std::isnan(getValue<axis>(a));
}

template<Axis axis>
static bool isGreaterOrNaN(const Zivid::Point &a, const Zivid::Point &b)
{
    return getValue<axis>(a) > getValue<axis>(b) ? true : std::isnan(getValue<axis>(a));
}

int main()
{
    try
    {
        Zivid::Application zivid;

        std::string Filename = "Zivid3D.zdf";
        std::cout << "Reading " << Filename << " point cloud" << std::endl;
        const auto frame = Zivid::Frame(Filename);

        std::cout << "Setting up visualization" << std::endl;
        Zivid::CloudVisualizer vis;
        zivid.setDefaultComputeDevice(vis.computeDevice());

        std::cout << "Displaying the point cloud" << std::endl;
        vis.showMaximized();
        vis.show(frame);
        vis.resetToFit();

        std::cout << "Running the visualizer. Blocking until the window closes" << std::endl;
        vis.run();

        std::cout << "Converting ZDF point cloud to OpenCV format" << std::endl;

        // Creating OpenCV structure
        const auto pointCloud = frame.getPointCloud();
        cv::Mat rgb((int)pointCloud.height(), (int)pointCloud.width(), CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Mat x((int)pointCloud.height(), (int)pointCloud.width(), CV_8UC1, cv::Scalar(0));
        cv::Mat y((int)pointCloud.height(), (int)pointCloud.width(), CV_8UC1, cv::Scalar(0));
        cv::Mat z((int)pointCloud.height(), (int)pointCloud.width(), CV_8UC1, cv::Scalar(0));

        // Getting min and max values for X, Y, Z images
        auto maxX =
            std::max_element(pointCloud.dataPtr(), pointCloud.dataPtr() + pointCloud.size(), isLesserOrNan<Axis::X>);
        auto minX =
            std::max_element(pointCloud.dataPtr(), pointCloud.dataPtr() + pointCloud.size(), isGreaterOrNaN<Axis::X>);
        auto maxY =
            std::max_element(pointCloud.dataPtr(), pointCloud.dataPtr() + pointCloud.size(), isLesserOrNan<Axis::Y>);
        auto minY =
            std::max_element(pointCloud.dataPtr(), pointCloud.dataPtr() + pointCloud.size(), isGreaterOrNaN<Axis::Y>);
        auto maxZ =
            std::max_element(pointCloud.dataPtr(), pointCloud.dataPtr() + pointCloud.size(), isLesserOrNan<Axis::Z>);
        auto minZ =
            std::max_element(pointCloud.dataPtr(), pointCloud.dataPtr() + pointCloud.size(), isGreaterOrNaN<Axis::Z>);

        // Filling in OpenCV matrices with the cloud data
        for(int i = 0; i < pointCloud.height(); i++)
        {
            for(int j = 0; j < pointCloud.width(); j++)
            {
                cv::Vec3b &color = rgb.at<cv::Vec3b>(i, j);
                color[0] = pointCloud(i, j).blue();
                color[1] = pointCloud(i, j).green();
                color[2] = pointCloud(i, j).red();

                if(std::isnan(pointCloud(i, j).z))
                {
                    x.at<uchar>(i, j) = 0;
                    y.at<uchar>(i, j) = 0;
                    z.at<uchar>(i, j) = 0;
                }
                else
                {
                    x.at<uchar>(i, j) = (uchar)(255.0f * (pointCloud(i, j).x - minX->x) / (maxX->x - minX->x));
                    y.at<uchar>(i, j) = (uchar)(255.0f * (pointCloud(i, j).y - minY->y) / (maxY->y - minY->y));
                    z.at<uchar>(i, j) = (uchar)(255.0f * (pointCloud(i, j).z - minZ->z) / (maxZ->z - minZ->z));
                }
            }
        }

        // Applying color map
        cv::Mat xJetColorMap, yJetColorMap, zJetColorMap;
        cv::applyColorMap(x, xJetColorMap, cv::COLORMAP_JET);
        cv::applyColorMap(y, yJetColorMap, cv::COLORMAP_JET);
        cv::applyColorMap(z, zJetColorMap, cv::COLORMAP_JET);

        // Setting nans to black
        for(int i = 0; i < pointCloud.height(); i++)
        {
            for(int j = 0; j < pointCloud.width(); j++)
            {
                if(std::isnan(pointCloud(i, j).z))
                {
                    cv::Vec3b &xRGB = xJetColorMap.at<cv::Vec3b>(i, j);
                    xRGB[0] = 0;
                    xRGB[1] = 0;
                    xRGB[2] = 0;

                    cv::Vec3b &yRGB = yJetColorMap.at<cv::Vec3b>(i, j);
                    yRGB[0] = 0;
                    yRGB[1] = 0;
                    yRGB[2] = 0;

                    cv::Vec3b &zRGB = zJetColorMap.at<cv::Vec3b>(i, j);
                    zRGB[0] = 0;
                    zRGB[1] = 0;
                    zRGB[2] = 0;
                }
            }
        }

        // Displaying the Depth image
        cv::namedWindow("Depth map", cv::WINDOW_AUTOSIZE);
        cv::imshow("Depth map", zJetColorMap);
        cv::waitKey(0);

        // Saving the Depth map
        cv::imwrite("Depth map.jpg", zJetColorMap);

        // Displaying the RGB image
        cv::namedWindow("RGB image", cv::WINDOW_AUTOSIZE);
        cv::imshow("RGB image", rgb);
        cv::waitKey(0);

        // Saving the RGB image
        cv::imwrite("RGB image.jpg", rgb);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}