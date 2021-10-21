/*
This example shows how to capture Zivid point clouds, with color and normals, and visualize it in 3D and as a normal
map. For scenes with high dynamic range we combine multiple acquisitions to get an HDR point cloud.
*/

#include <Zivid/Zivid.h>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
#include <iostream>
#include <thread>

namespace
{
    void visualizePointCloudAndNormalsPCL(
        const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> &pointCloudPCL,
        const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal>> &pointCloudWithNormalsPCL)
    {
        auto viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("Viewer");

        int viewRgb(0);
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, viewRgb);
        viewer->addText("Cloud RGB", 0, 0, "RGBText", viewRgb);
        viewer->addPointCloud<pcl::PointXYZRGB>(pointCloudPCL, "cloud", viewRgb);

        const int normalsSkipped = 10;
        std::cout << "Note! 1 out of " << normalsSkipped << " normals are visualized" << std::endl;

        int viewNormals(0);
        viewer->createViewPort(0.5, 0.0, 1.0, 1.0, viewNormals);
        viewer->addText("Cloud Normals", 0, 0, "NormalsText", viewNormals);
        viewer->addPointCloud<pcl::PointXYZRGBNormal>(pointCloudWithNormalsPCL, "cloudNormals", viewNormals);
        viewer->addPointCloudNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(pointCloudWithNormalsPCL,
                                                                                     pointCloudWithNormalsPCL,
                                                                                     normalsSkipped,
                                                                                     1,
                                                                                     "normals",
                                                                                     viewNormals);

        viewer->setCameraPosition(0, 0, -100, 0, -1, 0);

        std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
        std::cout << "Press q to me exit the viewer application" << std::endl;
        while(!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr zividToPclPoints(const Zivid::Array2D<Zivid::PointXYZColorRGBA> &data)
    {
        auto pointCloudPCL = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        pointCloudPCL->width = data.width();
        pointCloudPCL->height = data.height();
        pointCloudPCL->is_dense = false;
        pointCloudPCL->points.resize(data.size());
        for(size_t i = 0; i < data.size(); ++i)
        {
            pointCloudPCL->points[i].x = data(i).point.x; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudPCL->points[i].y = data(i).point.y; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudPCL->points[i].z = data(i).point.z; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudPCL->points[i].r = data(i).color.r; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudPCL->points[i].g = data(i).color.g; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudPCL->points[i].b = data(i).color.b; // NOLINT(cppcoreguidelines-pro-type-union-access)
        }
        return pointCloudPCL;
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr zividToPclVisualizationNormals(
        const Zivid::Array2D<Zivid::PointXYZColorRGBA> &data,
        const Zivid::Array2D<Zivid::NormalXYZ> &normals)
    {
        auto pointCloudWithNormals = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
        pointCloudWithNormals->width = data.width();
        pointCloudWithNormals->height = data.height();
        pointCloudWithNormals->is_dense = false;
        pointCloudWithNormals->points.resize(data.size());
        for(size_t i = 0; i < data.size(); ++i)
        {
            pointCloudWithNormals->points[i].x = data(i).point.x; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudWithNormals->points[i].y = data(i).point.y; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudWithNormals->points[i].z = data(i).point.z; // NOLINT(cppcoreguidelines-pro-type-union-access)
            // Scale colors for normals between 0 and 1 to create a normal map
            pointCloudWithNormals->points[i].r = static_cast<uint8_t>(
                0.5F * (1.0F - normals(i).x) * 255.F); // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudWithNormals->points[i].g = static_cast<uint8_t>(
                0.5F * (1.0F - normals(i).y) * 255.F); // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudWithNormals->points[i].b = static_cast<uint8_t>(
                0.5F * (1.0F - normals(i).z) * 255.F);                // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudWithNormals->points[i].normal_x = normals(i).x; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudWithNormals->points[i].normal_y = normals(i).y; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudWithNormals->points[i].normal_z = normals(i).z; // NOLINT(cppcoreguidelines-pro-type-union-access)
        }
        return pointCloudWithNormals;
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Configuring settings" << std::endl;
        Zivid::Settings settings;
        for(const auto aperture : { 9.57, 4.76, 2.59 })
        {
            std::cout << "Adding acquisition with aperture = " << aperture << std::endl;
            const auto acquisitionSettings = Zivid::Settings::Acquisition{
                Zivid::Settings::Acquisition::Aperture{ aperture },
            };
            settings.acquisitions().emplaceBack(acquisitionSettings);
        }

        std::cout << "Capturing frame (HDR)" << std::endl;
        const auto frame = camera.capture(settings);
        const auto pointCloud = frame.pointCloud();

        std::cout << "Creating PCL point cloud structure" << std::endl;
        const auto data = pointCloud.copyData<Zivid::PointXYZColorRGBA>();
        const auto pointCloudPCL = zividToPclPoints(data);

        std::cout << "Computing point cloud normals" << std::endl;
        const auto normals = pointCloud.copyData<Zivid::NormalXYZ>();

        std::cout << "Creating PCL normals structure suited for visualization" << std::endl;
        const auto pointCloudWithNormalsPCL = zividToPclVisualizationNormals(data, normals);

        std::cout << "Visualizing normals" << std::endl;
        visualizePointCloudAndNormalsPCL(pointCloudPCL, pointCloudWithNormalsPCL);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }
}
