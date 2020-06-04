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
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Creating settings" << std::endl;
        const auto settings = Zivid::Settings{ Zivid::Settings::Acquisitions{
            Zivid::Settings::Acquisition{ Zivid::Settings::Acquisition::Aperture{ 5.66 } } } };

        std::cout << "Capturing frame" << std::endl;
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

        std::cout << "Creating PCL point cloud structure" << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB> pointCloudPCL;

        std::cout << "Filling in point cloud data" << std::endl;
        pointCloudPCL.width = pointCloud.width();
        pointCloudPCL.height = pointCloud.height();
        pointCloudPCL.is_dense = false;
        pointCloudPCL.points.resize(pointCloudPCL.width * pointCloudPCL.height);

        for(size_t i = 0; i < pointCloudPCL.points.size(); ++i)
        {
            pointCloudPCL.points[i].x = data(i).point.x; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudPCL.points[i].y = data(i).point.y; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudPCL.points[i].z = data(i).point.z; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudPCL.points[i].r = data(i).color.r; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudPCL.points[i].g = data(i).color.g; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pointCloudPCL.points[i].b = data(i).color.b; // NOLINT(cppcoreguidelines-pro-type-union-access)
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
        *cloudPtr = pointCloudPCL;

        std::cout << "Visualizing point cloud" << std::endl;
        pcl::visualization::CloudViewer cloudViewer("Simple Cloud Viewer");
        std::cout << "Running visualizer. Blocking until window closes" << std::endl;
        cloudViewer.showCloud(cloudPtr);
        std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
        std::cout << "Press q to me exit the viewer application" << std::endl;
        while(!cloudViewer.wasStopped())
        {
        }

        std::string pointCloudFile = "Zivid3D.pcd";
        std::cout << "Saving point cloud to file: " << pointCloudFile << std::endl;
        pcl::io::savePCDFileBinary(pointCloudFile, pointCloudPCL);
        std::cerr << "Saved " << pointCloudPCL.points.size() << " points" << std::endl;
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
