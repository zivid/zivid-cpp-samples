/*
This example shows how to capture a Zivid point cloud and save it to a .PLY
file format.
*/

#include <Zivid/Zivid.h>

#include <chrono>
#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        auto FilenamePLY = "Zivid3D.ply";

        std::cout << "Connecting to the camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Configuring the camera settings" << std::endl;
        camera << Zivid::Settings::Iris{ 20 } << Zivid::Settings::ExposureTime{ std::chrono::microseconds{ 8333 } }
               << Zivid::Settings::Filters::Outlier::Enabled::yes << Zivid::Settings::Filters::Outlier::Threshold{ 5 };

        std::cout << "Capturing a frame" << std::endl;
        auto frame = camera.capture();

        std::cout << "Saving the frame to " << FilenamePLY << std::endl;
        frame.save(FilenamePLY);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
