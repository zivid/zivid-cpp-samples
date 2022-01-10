/*
Read point cloud from PCL file and visualize it.

The PCD file for this sample can be found under the main instructions for Zivid samples.
*/

#include <Zivid/Zivid.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <thread>

namespace
{
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
    void visualizePointCloud(const pcl::PointCloud<T> &pointCloud)
    {
        auto viewer = pcl::visualization::PCLVisualizer("Viewer");

        addPointCloudToViewer(viewer, pointCloud.makeShared());

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

int main(int argc, char **argv)
{
    try
    {
        bool useNormals = argc >= 2 && std::string(argv[1]) == "normals";

        std::string pointCloudFile = std::string(ZIVID_SAMPLE_DATA_DIR) + "/Zivid3D.pcd";
        std::cout << "Reading PCD point cloud from file: " << pointCloudFile << std::endl;

        if(useNormals)
        {
            auto pointCloudWithNormalsPCL = pcl::PointCloud<pcl::PointXYZRGBNormal>();

            pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(pointCloudFile, pointCloudWithNormalsPCL);
            std::cout << "Loaded " << pointCloudWithNormalsPCL.width * pointCloudWithNormalsPCL.height << " points"
                      << std::endl;

            visualizePointCloud(pointCloudWithNormalsPCL);
        }
        else
        {
            auto pointCloudPCL = pcl::PointCloud<pcl::PointXYZRGB>();

            pcl::io::loadPCDFile<pcl::PointXYZRGB>(pointCloudFile, pointCloudPCL);
            std::cout << "Loaded " << pointCloudPCL.width * pointCloudPCL.height << " points" << std::endl;

            visualizePointCloud(pointCloudPCL);
        }

        return 0;
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }
}
