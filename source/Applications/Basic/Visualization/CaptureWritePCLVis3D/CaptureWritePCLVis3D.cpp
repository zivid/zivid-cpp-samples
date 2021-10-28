/*
This example shows how capture point clouds, with color and with/without normals, from the Zivid camera,
convert it to PCL format, save it to PCD file, and visualize it.
*/

#include <Zivid/Zivid.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <thread>

namespace
{
    template<typename T>
    boost::shared_ptr<pcl::PointCloud<T>> addDataToPCLPointCloud(const Zivid::Array2D<Zivid::PointXYZColorRGBA> &data)
    {
        auto pointCloud = boost::make_shared<pcl::PointCloud<T>>();

        pointCloud->width = data.width();
        pointCloud->height = data.height();
        pointCloud->is_dense = false;
        pointCloud->points.resize(data.size());
        for(size_t i = 0; i < data.size(); ++i)
        {
            pointCloud->points[i].x = data(i).point.x; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloud->points[i].y = data(i).point.y; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloud->points[i].z = data(i).point.z; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloud->points[i].r = data(i).color.r; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloud->points[i].g = data(i).color.g; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloud->points[i].b = data(i).color.b; // NOLINT(cppcoreguidelines-pro-type-union-access)
        }
        return pointCloud;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertToPCLPointCloud(const Zivid::Array2D<Zivid::PointXYZColorRGBA> &data)
    {
        return addDataToPCLPointCloud<pcl::PointXYZRGB>(data);
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr convertToPCLPointCloud(
        const Zivid::Array2D<Zivid::PointXYZColorRGBA> &data,
        const Zivid::Array2D<Zivid::NormalXYZ> &normals)
    {
        auto pointCloud = addDataToPCLPointCloud<pcl::PointXYZRGBNormal>(data);

        for(size_t i = 0; i < data.size(); ++i)
        {
            pointCloud->points[i].normal_x = normals(i).x; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloud->points[i].normal_y = normals(i).y; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloud->points[i].normal_z = normals(i).z; // NOLINT(cppcoreguidelines-pro-type-union-access)
        }

        return pointCloud;
    }

    void addPointCloudToViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                               const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal>> &pointCloud)
    {
        viewer->addPointCloud<pcl::PointXYZRGBNormal>(pointCloud);

		const int normalsSkipped = 10;
        std::cout << "Note! 1 out of " << normalsSkipped << " normals are visualized" << std::endl;
        viewer->addPointCloudNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(pointCloud,
                                                                                     pointCloud,
                                                                                     normalsSkipped,
                                                                                     1,
                                                                                     "normals");
    }

    void addPointCloudToViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                               const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> &pointCloud)
    {
        viewer->addPointCloud<pcl::PointXYZRGB>(pointCloud);
    }

    template<typename T>
    void visualizePointCloudPCL(const boost::shared_ptr<pcl::PointCloud<T>> &pointCloud)
    {
        auto viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("Viewer");

        addPointCloudToViewer(viewer, pointCloud);

        viewer->setCameraPosition(0, 0, -100, 0, -1, 0);

        std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
        std::cout << "Press q to me exit the viewer application" << std::endl;
        while(!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    template<typename T>
    void visualizeAndSavePointCloudPCL(const boost::shared_ptr<pcl::PointCloud<T>> &pointCloud)
    {
        std::cout << "Visualizing PCL point cloud" << std::endl;
        visualizePointCloudPCL(pointCloud);

        std::string pointCloudFile = "Zivid3D.pcd";
        std::cout << "Saving point cloud to file: " << pointCloudFile << std::endl;
        pcl::io::savePCDFileBinary(pointCloudFile, *pointCloud);
        std::cerr << "Saved " << pointCloud->points.size() << " points" << std::endl;
    }

} // namespace

int main(int argc, char **argv)
{
    try
    {
        bool useNormals = argc >= 2 && std::string(argv[1]) == "normals";

        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Creating settings" << std::endl;
        const auto settings = Zivid::Settings{ Zivid::Settings::Acquisitions{
            Zivid::Settings::Acquisition{ Zivid::Settings::Acquisition::Aperture{ 5.66 } } } };

        std::cout << "Capturing frame" << std::endl;
        const auto frame = camera.capture(settings);
        const auto pointCloud = frame.pointCloud();
        const auto data = pointCloud.copyData<Zivid::PointXYZColorRGBA>();

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
}
