/*
This example shows how to import and display a Zivid point cloud from a.ZDF
file.
*/

#include <Zivid/CloudVisualizer.h>
#include <Zivid/Zivid.h>

#include <iostream>

int main()
{
	try
	{
		Zivid::Application zivid;

		std::cout << "Setting up visualization" << std::endl;
		Zivid::CloudVisualizer vis;
		zivid.setDefaultComputeDevice(vis.computeDevice());

		std::string Filename = "Zivid3D.zdf";
		std::cout << "Reading " << Filename << " point cloud" << std::endl;
		Zivid::Frame frame = Zivid::Frame(Filename);

		std::cout << "Displaying the frame" << std::endl;
		vis.showMaximized();
		vis.show(frame);
		vis.resetToFit();

		std::cout << "Running the visualizer. Blocking until the window closes" << std::endl;
		vis.run();
	}
	catch (const std::exception &e)
	{
		std::cerr << "Error: " << Zivid::toString(e) << std::endl;
		return EXIT_FAILURE;
	}
}
