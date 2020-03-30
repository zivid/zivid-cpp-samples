/*
This example shows how capture a Zivid point cloud, save it to a .PCD file format,
and visualize it.
*/

#include <Zivid/CloudVisualizer.h>
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

        Zivid::Application zivid;

        std::cout << "Setting up visualization" << std::endl;
        Zivid::CloudVisualizer vis;
        zivid.setDefaultComputeDevice(vis.computeDevice());

        std::cout << "Connecting to the camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Adjusting the iris" << std::endl;
        camera << Zivid::Settings::Iris{ 20 };

        std::cout << "Capturing a frame" << std::endl;
        auto frame = camera.capture();

        std::cout << "Displaying the frame" << std::endl;
        vis.showMaximized();
        vis.show(frame);
        vis.resetToFit();

        std::cout << "Running the Zivid visualizer. Blocking until the window closes" << std::endl;
        vis.run();

        auto pointCloud = frame.getPointCloud();

        // Creating a PointCloud structure
        pcl::PointCloud<pcl::PointXYZRGB> cloud;

        // Filling in the cloud data
        cloud.width = pointCloud.width();
        cloud.height = pointCloud.height();
        cloud.is_dense = false;
        cloud.points.resize(cloud.width * cloud.height);

        for(size_t i = 0; i < cloud.points.size(); ++i)
        {
            auto point = pointCloud.operator()(i);

            cloud.points[i].x = point.x;
            cloud.points[i].y = point.y;
            cloud.points[i].z = point.z;
            cloud.points[i].r = point.red();
            cloud.points[i].g = point.green();
            cloud.points[i].b = point.blue();
        }

        //Simple Cloud Visualization
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZRGB>);
        *cloudPTR = cloud;

        std::cout << "Run the PCL visualizer. Block until window closes" << std::endl;
        pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
        viewer.showCloud(cloudPTR);
        std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
        std::cout << "Press q to me exit the viewer application" << std::endl;
        while(!viewer.wasStopped())
        {
        }

        //Saving to a .PCD file format
        std::cerr << "Saving " << cloud.points.size() << " data points to " + filenamePCD << std::endl;
        pcl::io::savePCDFileBinary(filenamePCD, cloud);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
