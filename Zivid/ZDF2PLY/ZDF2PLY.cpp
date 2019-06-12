/*
This example shows how to convert a Zivid point cloud from a .ZDF file format
to a .PLY file format.
*/

#include <Zivid/Zivid.h>

#include <iostream>

int main()
{
	try
	{
		Zivid::Application zivid;

		std::string FilenameZDF = "Zivid3D.zdf";
		std::string FilenamePLY = "Zivid3D.ply";

		Zivid::Frame frame = Zivid::Frame(FilenameZDF);

		std::cout << "Saving the frame to " << FilenamePLY << std::endl;
		frame.save(FilenamePLY);

	}
	catch (const std::exception &e)
	{
		std::cerr << "Error: " << Zivid::toString(e) << std::endl;
		return EXIT_FAILURE;
	}
}
