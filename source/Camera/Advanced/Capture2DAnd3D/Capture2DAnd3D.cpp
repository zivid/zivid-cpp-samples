/*
Capture 2D and 3D separately with the Zivid camera.

Capture separate 2D image with subsampling2x2. Then capture 3D with subsampling4x4
and upsampling2x2 to match resolution  of 2D.
Then use color from 2D when visualizing the 3D point cloud.
*/

#include <Zivid/Zivid.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <map>
#include <thread>

namespace
{
    void displayPointCloud(const Zivid::PointCloud &pointCloud, const cv::Mat &bgra)
    {
        const auto xyz = pointCloud.copyData<Zivid::PointXYZ>();

        auto pclPointCloud = pcl::PointCloud<pcl::PointXYZRGB>();
        pclPointCloud.width = xyz.width();
        pclPointCloud.height = xyz.height();
        pclPointCloud.is_dense = false;
        pclPointCloud.points.resize(xyz.size());
        for(size_t i = 0; i < xyz.size(); ++i)
        {
            pclPointCloud.points[i].x = xyz(i).x;                 // NOLINT(cppcoreguidelines-pro-type-union-access)
            pclPointCloud.points[i].y = xyz(i).y;                 // NOLINT(cppcoreguidelines-pro-type-union-access)
            pclPointCloud.points[i].z = xyz(i).z;                 // NOLINT(cppcoreguidelines-pro-type-union-access)
            pclPointCloud.points[i].b = bgra.at<cv::Vec4b>(i)[0]; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pclPointCloud.points[i].g = bgra.at<cv::Vec4b>(i)[1]; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pclPointCloud.points[i].r = bgra.at<cv::Vec4b>(i)[2]; // NOLINT(cppcoreguidelines-pro-type-union-access)
        }

        auto viewer = pcl::visualization::PCLVisualizer("Viewer");

        int viewRgb(0);
        viewer.createViewPort(0.0, 0.0, 1.0, 1.0, viewRgb);
        viewer.addText("Cloud RGB", 0, 0, "Mapped RGB", viewRgb);
        viewer.addPointCloud<pcl::PointXYZRGB>(pclPointCloud.makeShared(), "cloud", viewRgb);

        viewer.setCameraPosition(0, 0, -100, 0, -1, 0);

        std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
        std::cout << "Press q to exit the viewer application" << std::endl;
        while(!viewer.wasStopped())
        {
            viewer.spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Configuring 2D settings" << std::endl;
        const auto settings2D = Zivid::Settings2D{
            Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} },
            Zivid::Settings2D::Sampling::Pixel::blueSubsample2x2,
        };

        std::cout << "Configuring 3D settings" << std::endl;
        auto settings = Zivid::Settings{
            Zivid::Settings::Engine::phase,
            Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} },
            Zivid::Settings::Sampling::Pixel::blueSubsample4x4,
            Zivid::Settings::Sampling::Color::disabled,
            Zivid::Settings::Processing::Resampling::Mode::upsample2x2,
        };

        if((camera.info().model() == Zivid::CameraInfo::Model::zividTwo)
           || (camera.info().model() == Zivid::CameraInfo::Model::zividTwoL100))
        {
            std::cout
                << camera.info().modelName()
                << " does not support 4x4 subsampling. This sample is written to show how combinations of Sampling::Pixel and Processing::Resampling::Mode."
                << std::endl;
            settings = settings.copyWith(
                Zivid::Settings::Sampling::Pixel::blueSubsample2x2,
                Zivid::Settings::Processing::Resampling::Mode::disabled);
        }

        std::cout << "Capturing 2D frame" << std::endl;
        const auto frame2D = camera.capture(settings2D);
        std::cout << "Getting BGRA image" << std::endl;
        const auto image = frame2D.imageBGRA();
        const cv::Mat bgra(
            image.height(),
            image.width(),
            CV_8UC4, // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
            const_cast<void *>(static_cast<const void *>(image.data())));

        std::cout << "Capturing frame" << std::endl;

        const auto frame = camera.capture(settings);
        const auto pointCloud = frame.pointCloud();
        std::cout << "Visualizing point cloud" << std::endl;
        displayPointCloud(pointCloud, bgra);
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
