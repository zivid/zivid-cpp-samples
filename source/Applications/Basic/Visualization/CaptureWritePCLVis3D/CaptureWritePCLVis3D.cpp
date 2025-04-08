/*
Capture point clouds, with color, from the Zivid camera, save it to PCD file format, and visualize it.
*/

#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <clipp.h>

#include <iostream>
#include <thread>

namespace
{
    template<typename T>
    pcl::PointCloud<T> addDataToPCLPointCloud(const Zivid::Array2D<Zivid::PointXYZColorRGBA_SRGB> &data)
    {
        auto pointCloud = pcl::PointCloud<T>();

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

    pcl::PointCloud<pcl::PointXYZRGB> convertToPCLPointCloud(const Zivid::Array2D<Zivid::PointXYZColorRGBA_SRGB> &data)
    {
        return addDataToPCLPointCloud<pcl::PointXYZRGB>(data);
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal> convertToPCLPointCloud(
        const Zivid::Array2D<Zivid::PointXYZColorRGBA_SRGB> &data,
        const Zivid::Array2D<Zivid::NormalXYZ> &normals)
    {
        auto pointCloud = addDataToPCLPointCloud<pcl::PointXYZRGBNormal>(data);

        for(size_t i = 0; i < data.size(); ++i)
        {
            pointCloud.points[i].normal_x = normals(i).x; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloud.points[i].normal_y = normals(i).y; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloud.points[i].normal_z = normals(i).z; // NOLINT(cppcoreguidelines-pro-type-union-access)
        }

        return pointCloud;
    }

    void addPointCloudToViewer(
        pcl::visualization::PCLVisualizer &viewer,
        const pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr &pointCloud)
    {
        viewer.addPointCloud<pcl::PointXYZRGBNormal>(pointCloud);

        const int normalsSkipped = 10;
        std::cout << "Note! 1 out of " << normalsSkipped << " normals are visualized" << std::endl;
        viewer.addPointCloudNormals<pcl::PointXYZRGBNormal>(pointCloud, normalsSkipped, 1, "normals");
    }

    void addPointCloudToViewer(
        pcl::visualization::PCLVisualizer &viewer,
        const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pointCloud)
    {
        viewer.addPointCloud<pcl::PointXYZRGB>(pointCloud);
    }

    template<typename T>
    void visualizePointCloudPCL(const pcl::PointCloud<T> &pointCloud)
    {
        auto viewer = pcl::visualization::PCLVisualizer("Viewer");

        addPointCloudToViewer(viewer, pointCloud.makeShared());

        viewer.setCameraPosition(0, 0, -100, 0, 0, 1000, 0, -1, 0);

        std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
        std::cout << "Press q to exit the viewer application" << std::endl;
        while(!viewer.wasStopped())
        {
            viewer.spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    template<typename T>
    void visualizeAndSavePointCloudPCL(const pcl::PointCloud<T> &pointCloud)
    {
        std::cout << "Visualizing PCL point cloud" << std::endl;
        visualizePointCloudPCL(pointCloud);

        std::string pointCloudFile = "Zivid3D.pcd";
        std::cout << "Saving point cloud to file: " << pointCloudFile << std::endl;
        pcl::io::savePCDFileBinary(pointCloudFile, pointCloud);
        std::cerr << "Saved " << pointCloud.points.size() << " points" << std::endl;
    }

} // namespace

int main(int argc, char **argv)
{
    try
    {
        bool useNormals = false;
        auto cli =
            clipp::group(clipp::option("normals").set(useNormals).doc("Compute normals in captured point cloud."));

        if(!parse(argc, argv, cli))
        {
            auto fmt = clipp::doc_formatting{};
            std::cout << "SYNOPSIS:" << std::endl;
            std::cout << clipp::usage_lines(cli, "CaptureWritePCLVis3D", fmt) << std::endl;
            std::cout << "OPTIONS:" << std::endl;
            std::cout << clipp::documentation(cli) << std::endl;
            throw std::runtime_error{ "Invalid usage" };
        }

        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Creating settings" << std::endl;
        const auto settings =
            Zivid::Settings{ Zivid::Settings::Acquisitions{
                                 Zivid::Settings::Acquisition{ Zivid::Settings::Acquisition::Aperture{ 5.66 } } },
                             Zivid::Settings::Color{ Zivid::Settings2D{
                                 Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } } } };

        std::cout << "Capturing frame" << std::endl;
        const auto frame = camera.capture2D3D(settings);
        const auto pointCloud = frame.pointCloud();
        const auto data = pointCloud.copyData<Zivid::PointXYZColorRGBA_SRGB>();

        if(useNormals)
        {
            std::cout << "Computing point cloud normals" << std::endl;
            const auto normals = pointCloud.copyData<Zivid::NormalXYZ>();

            std::cout << "Converting Zivid point cloud with normals to PCL format" << std::endl;
            const auto pointCloudWithNormalsPCL = convertToPCLPointCloud(data, normals);

            visualizeAndSavePointCloudPCL(pointCloudWithNormalsPCL);
        }
        else
        {
            std::cout << "Converting Zivid point cloud to PCL format" << std::endl;
            const auto pointCloudPCL = convertToPCLPointCloud(data);

            visualizeAndSavePointCloudPCL(pointCloudPCL);
        }
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
