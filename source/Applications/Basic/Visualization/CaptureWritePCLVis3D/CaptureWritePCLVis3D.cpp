/*
This example shows how capture point clouds, with color, from the Zivid camera,
save it to PCD file format, and visualize it.
*/

#include <Zivid/Visualization/Visualizer.h>
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

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Creating settings" << std::endl;
        const auto settings = Zivid::Settings{ Zivid::Settings::Acquisitions{
            Zivid::Settings::Acquisition{ Zivid::Settings::Acquisition::Aperture{ 5.66 } } } };

        std::cout << "Capturing a frame" << std::endl;
        const auto frame = camera.capture(settings);

        std::cout << "Setting up visualization" << std::endl;
        Zivid::Visualization::Visualizer visualizer;

        std::cout << "Visualizing point cloud" << std::endl;
        visualizer.showMaximized();
        visualizer.show(frame);
        visualizer.resetToFit();

        std::cout << "Running visualizer. Blocking until window closes" << std::endl;
        visualizer.run();

        const auto pointCloud = frame.pointCloud();
        const auto data = pointCloud.copyData<Zivid::PointXYZColorRGBA>();

        // Creating a PointCloud structure
        pcl::PointCloud<pcl::PointXYZRGB> cloud;

        // Filling in the cloud data
        cloud.width = pointCloud.width();
        cloud.height = pointCloud.height();
        cloud.is_dense = false;
        cloud.points.resize(cloud.width * cloud.height);

        for(size_t i = 0; i < cloud.points.size(); ++i)
        {
            cloud.points[i].x = data(i).point.x; // NOLINT(cppcoreguidelines-pro-type-union-access)
            cloud.points[i].y = data(i).point.y; // NOLINT(cppcoreguidelines-pro-type-union-access)
            cloud.points[i].z = data(i).point.z; // NOLINT(cppcoreguidelines-pro-type-union-access)
            cloud.points[i].r = data(i).color.r; // NOLINT(cppcoreguidelines-pro-type-union-access)
            cloud.points[i].g = data(i).color.g; // NOLINT(cppcoreguidelines-pro-type-union-access)
            cloud.points[i].b = data(i).color.b; // NOLINT(cppcoreguidelines-pro-type-union-access)
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
