/*
This example shows how to read point cloud from PCL file and visualize it.
The PCD file for this sample can be found under the main instructions for Zivid samples.
*/

#include <Zivid/Zivid.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>

int main()
{
    try
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);

        std::string pointCloudFile = std::string(ZIVID_SAMPLE_DATA_DIR) + "/Zivid3D.pcd";
        std::cout << "Reading PCD point cloud from file: " << pointCloudFile << std::endl;
        pcl::io::loadPCDFile<pcl::PointXYZRGB>(pointCloudFile, *cloudPtr);
        std::cout << "Loaded " << cloudPtr->width * cloudPtr->height << " points" << std::endl;

        std::cout << "Visualizing point cloud" << std::endl;
        pcl::visualization::CloudViewer cloudViewer("Simple Cloud Viewer");
        std::cout << "Running visualizer. Blocking until window closes" << std::endl;
        cloudViewer.showCloud(cloudPtr);
        std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
        std::cout << "Press q to me exit the viewer application" << std::endl;
        while(!cloudViewer.wasStopped())
        {
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
