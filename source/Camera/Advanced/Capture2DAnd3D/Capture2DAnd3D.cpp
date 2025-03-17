/*
Capture 2D and 3D with the Zivid camera.

Capture separate 2D and 3D with different sampling modes based on camera model.
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
    Zivid::Settings get2DAnd3DSettings(const Zivid::Camera &camera)
    {
        auto settings = Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} },
                                         Zivid::Settings::Color{ Zivid::Settings2D{
                                             Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } } } };

        auto model = camera.info().model();
        switch(model.value())
        {
            case Zivid::CameraInfo::Model::ValueType::zividTwo:
            case Zivid::CameraInfo::Model::ValueType::zividTwoL100:
            {
                settings.set(Zivid::Settings::Sampling::Pixel::blueSubsample2x2);
                settings.set(Zivid::Settings::Processing::Resampling::Mode::upsample2x2);
                settings.color().value().set(Zivid::Settings2D::Sampling::Pixel::all);
                break;
            }
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusM130:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusM60:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusL110:
            {
                settings.set(Zivid::Settings::Sampling::Pixel::blueSubsample4x4);
                settings.set(Zivid::Settings::Processing::Resampling::Mode::upsample2x2);
                settings.color().value().set(Zivid::Settings2D::Sampling::Pixel::blueSubsample2x2);
                break;
            }
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusMR130:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusMR60:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusLR110:
            {
                settings.set(Zivid::Settings::Sampling::Pixel::by4x4);
                settings.set(Zivid::Settings::Processing::Resampling::Mode::upsample2x2);
                settings.color().value().set(Zivid::Settings2D::Sampling::Pixel::by2x2);
                break;
            }
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusSmall:
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusMedium:
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusLarge:
            {
                throw std::runtime_error("Unsupported camera model '" + model.toString() + "'");
            }
            default: throw std::runtime_error("Unhandled enum value '" + model.toString() + "'");
        }

        return settings;
    }

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

        viewer.setCameraPosition(0, 0, -100, 0, 0, 1000, 0, -1, 0);

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

        std::cout << "Configuring 2D and 3D settings" << std::endl;
        const auto settings = get2DAnd3DSettings(camera);

        std::cout << "Capturing 2D+3D" << std::endl;
        const auto frame = camera.capture2D3D(settings);

        std::cout << "Getting BGRA image" << std::endl;
        const auto image = frame.frame2D().value().imageBGRA();
        const cv::Mat bgra(
            image.height(),
            image.width(),
            CV_8UC4, // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
            const_cast<void *>(static_cast<const void *>(image.data())));

        std::cout << "Getting point cloud" << std::endl;
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
