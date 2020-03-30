/*
This example shows how to read a PCL point cloud and visualize it. To get a
Zivid point cloud in .PCD file format, run ZDF2PCD sample. Then, copy it to
the correct directory for this sample.
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
        std::string filenamePCD = "Zivid3D.pcd";
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZRGB>);

        // Reading a .PCL point cloud
        if(pcl::io::loadPCDFile<pcl::PointXYZRGB>(filenamePCD, *cloudPTR) == -1) //* load the file
        {
            std::cerr
                << "Error: "
                << "Run ZDF2PCD sample to get a Zivid point cloud in .PCD file format, then copy it in the correct directory for this sample. \n"
                << std::endl;
            return (-1);
        }
        std::cout << "Loaded " << cloudPTR->width * cloudPTR->height << " data points from " + filenamePCD << std::endl;

        //Simple Cloud Visualization
        pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
        viewer.showCloud(cloudPTR);
        std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
        std::cout << "Press q to me exit the viewer application" << std::endl;
        while(!viewer.wasStopped())
        {
        }

        return 0;
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
