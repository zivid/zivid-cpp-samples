/*
Capture point clouds, with color, from the Zivid camera.
*/

#include <Zivid/Zivid.h>

#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Creating default capture settings" << std::endl;
        const auto settings = Zivid::Settings(Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} });

        std::cout << "Capturing frame" << std::endl;
        const auto frame = camera.capture(settings);

        const auto dataFile = "Frame.zdf";
        std::cout << "Saving frame to file: " << dataFile << std::endl;
        frame.save(dataFile);

        const auto dataFilePLY = "PointCloud.ply";
        std::cout << "Exporting point cloud to file: " << dataFilePLY << std::endl;
        frame.save(dataFilePLY);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
