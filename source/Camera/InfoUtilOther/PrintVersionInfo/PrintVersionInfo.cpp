/*
This example shows how to read information from each connected camera.
*/

#include <Zivid/Zivid.h>

#include <iostream>

int main()
{
    try
    {
        std::cout << "Finding cameras" << std::endl;
        Zivid::Application zivid;
        std::cout << "Zivid SDK: " << Zivid::Version::coreLibraryVersion() << std::endl;
        auto cameras = zivid.cameras();
        std::cout << "Found " << cameras.size() << " cameras" << std::endl;
        for(auto &camera : cameras)
        {
            std::cout << "Camera Info: " << camera.info() << std::endl;
        }
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }
}
