/*
This example shows how to convert a Zivid point cloud from a .ZDF file format
to a .PCD file format.
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
		std::string filenameZDF = "Zivid3D.zdf";
		std::string filenamePCD = "Zivid3D.pcd";

		Zivid::Application zivid;

		// Reading a .ZDF point cloud
		std::cout << "Reading " + filenameZDF << std::endl;
		Zivid::Frame frame(filenameZDF);


		auto PointCloud = frame.getPointCloud();

		// Creating a PointCloud structure
		pcl::PointCloud<pcl::PointXYZRGB> cloud;

		// Filling in the cloud data
		cloud.width = PointCloud.width();
		cloud.height = PointCloud.height();
		cloud.is_dense = false;
		cloud.points.resize(cloud.width * cloud.height);

		for (size_t i = 0; i < cloud.points.size(); ++i)
		{
			auto Point = PointCloud.operator()(i);

			cloud.points[i].x = Point.x;
			cloud.points[i].y = Point.y;
			cloud.points[i].z = Point.z;
			cloud.points[i].r = (uint8_t)Point.red();
			cloud.points[i].g = (uint8_t)Point.green();
			cloud.points[i].b = (uint8_t)Point.blue();
		}

		//Saving to a .PCD file format
		std::cerr << "Saving " << cloud.points.size() << " data points to " + filenamePCD << std::endl;
		pcl::io::savePCDFileBinary(filenamePCD, cloud);
	}
	catch (const std::exception &e)
	{
		std::cerr << "Error: " << Zivid::toString(e) << std::endl;
		return EXIT_FAILURE;
	}
}
