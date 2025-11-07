/*
Capture Zivid point clouds, with color and normals, and visualize it in 3D and as a normal map.
*/

#include <Zivid/Zivid.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <chrono>
#include <iostream>
#include <thread>

namespace
{
    void visualizePointCloudAndNormalsPCL(
        const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pointCloud,
        const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &pointCloudWithNormals)
    {
        auto viewer = pcl::visualization::PCLVisualizer("Viewer");

        int viewRgb(0);
        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, viewRgb);
        viewer.addText("Cloud RGB", 0, 0, "RGBText", viewRgb);
        viewer.addPointCloud<pcl::PointXYZRGB>(pointCloud, "cloud", viewRgb);

        const int normalsSkipped = 10;
        std::cout << "Note! 1 out of " << normalsSkipped << " normals are visualized" << std::endl;

        int viewNormals(0);
        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, viewNormals);
        viewer.addText("Cloud Normals", 0, 0, "NormalsText", viewNormals);
        viewer.addPointCloud<pcl::PointXYZRGBNormal>(pointCloudWithNormals, "cloudNormals", viewNormals);
        viewer.addPointCloudNormals<pcl::PointXYZRGBNormal>(
            pointCloudWithNormals, normalsSkipped, 1, "normals", viewNormals);

        viewer.setCameraPosition(0, 0, -100, 0, 0, 1000, 0, -1, 0);

        std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
        std::cout << "Press q to exit the viewer application" << std::endl;
        while(!viewer.wasStopped())
        {
            viewer.spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }


    pcl::PointCloud<pcl::PointXYZRGB> convertToPCLPointCloud(const Zivid::Array2D<Zivid::PointXYZColorRGBA_SRGB> &data)
    {
        auto pointCloud = pcl::PointCloud<pcl::PointXYZRGB>();
        pointCloud.width = data.width();
        pointCloud.height = data.height();
        pointCloud.is_dense = false;
        pointCloud.points.resize(data.size());
        for(size_t i = 0; i < data.size(); ++i)
        {
            pointCloud.points[i].x = data(i).point.x; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloud.points[i].y = data(i).point.y; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloud.points[i].z = data(i).point.z; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloud.points[i].r = data(i).color.r; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloud.points[i].g = data(i).color.g; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloud.points[i].b = data(i).color.b; // NOLINT(cppcoreguidelines-pro-type-union-access)
        }
        return pointCloud;
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal> convertToPCLVisualizationNormals(
        const Zivid::Array2D<Zivid::PointXYZColorRGBA_SRGB> &data,
        const Zivid::Array2D<Zivid::NormalXYZ> &normals)
    {
        auto pointCloudWithNormals = pcl::PointCloud<pcl::PointXYZRGBNormal>();
        pointCloudWithNormals.width = data.width();
        pointCloudWithNormals.height = data.height();
        pointCloudWithNormals.is_dense = false;
        pointCloudWithNormals.points.resize(data.size());
        for(size_t i = 0; i < data.size(); ++i)
        {
            pointCloudWithNormals.points[i].x = data(i).point.x; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudWithNormals.points[i].y = data(i).point.y; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudWithNormals.points[i].z = data(i).point.z; // NOLINT(cppcoreguidelines-pro-type-union-access)
            // Scale colors for normals between 0 and 1 to create a normal map
            pointCloudWithNormals.points[i].r = static_cast<uint8_t>(
                0.5F * (1.0F - normals(i).x) * 255.F); // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudWithNormals.points[i].g = static_cast<uint8_t>(
                0.5F * (1.0F - normals(i).y) * 255.F); // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudWithNormals.points[i].b = static_cast<uint8_t>(
                0.5F * (1.0F - normals(i).z) * 255.F);               // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudWithNormals.points[i].normal_x = normals(i).x; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudWithNormals.points[i].normal_y = normals(i).y; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudWithNormals.points[i].normal_z = normals(i).z; // NOLINT(cppcoreguidelines-pro-type-union-access)
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

        std::cout << "Creating settings" << std::endl;
        Zivid::Settings settings = Zivid::Settings{
            Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{ Zivid::Settings::Acquisition{} } },
            Zivid::Settings::Color{
                Zivid::Settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } } }
        };

        std::cout << "Capturing frame" << std::endl;
        const auto frame = camera.capture2D3D(settings);
        const auto pointCloud = frame.pointCloud();

        std::cout << "Creating PCL point cloud structure" << std::endl;
        const auto data = pointCloud.copyData<Zivid::PointXYZColorRGBA_SRGB>();
        const auto pointCloudPCL = convertToPCLPointCloud(data);

        std::cout << "Computing point cloud normals" << std::endl;
        const auto normals = pointCloud.copyData<Zivid::NormalXYZ>();

        std::cout << "Creating PCL normals structure suited for visualization" << std::endl;
        const auto pointCloudWithNormalsPCL = convertToPCLVisualizationNormals(data, normals);

        std::cout << "Visualizing normals" << std::endl;
        visualizePointCloudAndNormalsPCL(pointCloudPCL.makeShared(), pointCloudWithNormalsPCL.makeShared());
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
