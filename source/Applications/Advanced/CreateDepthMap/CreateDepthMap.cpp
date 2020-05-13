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
    x,
    y,
    z
};

template<Axis axis>
float getValue(const Zivid::Point &p);

template<>
float getValue<Axis::x>(const Zivid::Point &p)
{
    return p.x;
}

template<>
float getValue<Axis::y>(const Zivid::Point &p)
{
    return p.y;
}

template<>
float getValue<Axis::z>(const Zivid::Point &p)
{
    return p.z;
}

template<Axis axis>
bool isLesserOrNan(const Zivid::Point &a, const Zivid::Point &b)
{
    return getValue<axis>(a) < getValue<axis>(b) ? true : std::isnan(getValue<axis>(a));
}

template<Axis axis>
bool isGreaterOrNaN(const Zivid::Point &a, const Zivid::Point &b)
{
    return getValue<axis>(a) > getValue<axis>(b) ? true : std::isnan(getValue<axis>(a));
}

int main()
{
    try
    {
        Zivid::Application zivid;

        const auto fileName = Zivid::Environment::dataPath() + "/Zivid3D.zdf";
        std::cout << "Reading " << fileName << " point cloud" << std::endl;
        const auto frame = Zivid::Frame(fileName);

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
        const auto height = static_cast<unsigned int>(pointCloud.height());
        const auto width = static_cast<unsigned int>(pointCloud.width());
        cv::Mat rgb(height, width, CV_8UC3, cv::Scalar(0, 0, 0)); // NOLINT(hicpp-signed-bitwise)
        cv::Mat x(height, width, CV_8UC1, cv::Scalar(0));         // NOLINT(hicpp-signed-bitwise)
        cv::Mat y(height, width, CV_8UC1, cv::Scalar(0));         // NOLINT(hicpp-signed-bitwise)
        cv::Mat z(height, width, CV_8UC1, cv::Scalar(0));         // NOLINT(hicpp-signed-bitwise)

        // Getting min and max values for X, Y, Z images
        const auto *maxX = std::max_element(pointCloud.dataPtr(),
                                            std::next(pointCloud.dataPtr(), pointCloud.size()),
                                            isLesserOrNan<Axis::x>);
        const auto *minX = std::max_element(pointCloud.dataPtr(),
                                            std::next(pointCloud.dataPtr(), pointCloud.size()),
                                            isGreaterOrNaN<Axis::y>);
        const auto *maxY = std::max_element(pointCloud.dataPtr(),
                                            std::next(pointCloud.dataPtr(), pointCloud.size()),
                                            isLesserOrNan<Axis::z>);
        const auto *minY = std::max_element(pointCloud.dataPtr(),
                                            std::next(pointCloud.dataPtr(), pointCloud.size()),
                                            isGreaterOrNaN<Axis::y>);
        const auto *maxZ = std::max_element(pointCloud.dataPtr(),
                                            std::next(pointCloud.dataPtr(), pointCloud.size()),
                                            isLesserOrNan<Axis::z>);
        const auto *minZ = std::max_element(pointCloud.dataPtr(),
                                            std::next(pointCloud.dataPtr(), pointCloud.size()),
                                            isGreaterOrNaN<Axis::z>);

        // Filling in OpenCV matrices with the cloud data
        for(size_t i = 0; i < pointCloud.height(); i++)
        {
            for(size_t j = 0; j < pointCloud.width(); j++)
            {
                auto &color = rgb.at<cv::Vec3b>(i, j);
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
                    x.at<uchar>(i, j) =
                        static_cast<unsigned char>((255.0F * (pointCloud(i, j).x - minX->x) / (maxX->x - minX->x)));
                    y.at<uchar>(i, j) =
                        static_cast<unsigned char>((255.0F * (pointCloud(i, j).y - minY->y) / (maxY->y - minY->y)));
                    z.at<uchar>(i, j) =
                        static_cast<unsigned char>((255.0F * (pointCloud(i, j).z - minZ->z) / (maxZ->z - minZ->z)));
                }
            }
        }

        // Applying color map
        cv::Mat xJetColorMap;
        cv::Mat yJetColorMap;
        cv::Mat zJetColorMap;
        cv::applyColorMap(x, xJetColorMap, cv::COLORMAP_JET);
        cv::applyColorMap(y, yJetColorMap, cv::COLORMAP_JET);
        cv::applyColorMap(z, zJetColorMap, cv::COLORMAP_JET);

        // Setting nans to black
        for(size_t i = 0; i < pointCloud.height(); i++)
        {
            for(size_t j = 0; j < pointCloud.width(); j++)
            {
                if(std::isnan(pointCloud(i, j).z))
                {
                    auto &xRGB = xJetColorMap.at<cv::Vec3b>(i, j);
                    xRGB[0] = 0;
                    xRGB[1] = 0;
                    xRGB[2] = 0;

                    auto &yRGB = yJetColorMap.at<cv::Vec3b>(i, j);
                    yRGB[0] = 0;
                    yRGB[1] = 0;
                    yRGB[2] = 0;

                    auto &zRGB = zJetColorMap.at<cv::Vec3b>(i, j);
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
