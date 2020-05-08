/*
This example shows how to convert point cloud from ZDF file to OpenCV format, then extract and visualize depth map.
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
    enum class Axis
    {
        x,
        y,
        z
    };

    template<Axis axis>
    float getValue(const Zivid::PointXYZ &p);

    template<>
    float getValue<Axis::z>(const Zivid::PointXYZ &p)
    {
        return p.z;
    }

    template<Axis axis>
    bool isLesserOrNan(const Zivid::PointXYZ &a, const Zivid::PointXYZ &b)
    {
        return getValue<axis>(a) < getValue<axis>(b) ? true : std::isnan(getValue<axis>(a));
    }

    template<Axis axis>
    bool isGreaterOrNaN(const Zivid::PointXYZ &a, const Zivid::PointXYZ &b)
    {
        return getValue<axis>(a) > getValue<axis>(b) ? true : std::isnan(getValue<axis>(a));
    }

    void visualizePointCloud(const Zivid::PointCloud &pointCloud)
    {
        std::cout << "Setting up visualization" << std::endl;
        Zivid::Visualization::Visualizer visualizer;

        std::cout << "Visualizing point cloud" << std::endl;
        visualizer.showMaximized();
        visualizer.show(pointCloud);
        visualizer.resetToFit();

        std::cout << "Running visualizer. Blocking until window closes" << std::endl;
        visualizer.run();
    }

    cv::Mat pointCloudToCvZ(const Zivid::PointCloud &pointCloud)
    {
        // Creating OpenCV structure
        cv::Mat z(pointCloud.height(), pointCloud.width(), CV_8UC1, cv::Scalar(0)); // NOLINT(hicpp-signed-bitwise)

        const auto points = pointCloud.copyPointsXYZ();

        // Getting min and max values for X, Y, Z images
        const auto *maxZ = std::max_element(points.data(), points.data() + pointCloud.size(), isLesserOrNan<Axis::z>);
        const auto *minZ = std::max_element(points.data(), points.data() + pointCloud.size(), isGreaterOrNaN<Axis::z>);

        // Filling in OpenCV matrices with the cloud data
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
        cv::Mat zJetColorMap;
        cv::applyColorMap(z, zJetColorMap, cv::COLORMAP_JET);

        // Setting nans to black
        for(size_t i = 0; i < pointCloud.height(); i++)
        {
            for(size_t j = 0; j < pointCloud.width(); j++)
            {
                if(std::isnan(points(i, j).z))
                {
                    auto &zRGB = zJetColorMap.at<cv::Vec3b>(i, j);
                    zRGB[0] = 0;
                    zRGB[1] = 0;
                    zRGB[2] = 0;
                }
            }
        }

        return zJetColorMap;
    }

    cv::Mat pointCloudToCvBGR(const Zivid::PointCloud &pointCloud)
    {
        auto rgb = cv::Mat(pointCloud.height(), pointCloud.width(), CV_8UC4); // NOLINT(hicpp-signed-bitwise)
        pointCloud.copyData(
            reinterpret_cast<Zivid::ColorRGBA *>(rgb.data)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        auto bgr = cv::Mat(pointCloud.height(), pointCloud.width(), CV_8UC4); // NOLINT(hicpp-signed-bitwise)
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

        // Displaying the RGB image
        cv::namedWindow("RGB image", cv::WINDOW_AUTOSIZE);
        cv::imshow("RGB image", bgr);
        cv::waitKey(0);

        // Saving the RGB image
        cv::imwrite("RGB image.jpg", bgr);

        std::cout << "Converting Zivid PointCloud to OpenCV format" << std::endl;
        cv::Mat zJetColorMap = pointCloudToCvZ(pointCloud);

        // Displaying the Depth image
        cv::namedWindow("Depth map", cv::WINDOW_AUTOSIZE);
        cv::imshow("Depth map", zJetColorMap);
        cv::waitKey(0);

        // Saving the Depth map
        cv::imwrite("Depth map.jpg", zJetColorMap);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
