/*
List connected cameras and print camera version and state information for each connected camera.
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
            std::cout << camera.info() << std::endl;
            std::cout << camera.state() << std::endl;
        }

        for(auto &camera : cameras)
        {
            const auto temperature = camera.state().temperature();
            std::cout << "Temperatures:" << std::endl;
            std::cout << "  DMD:     " << temperature.dmd().value() << " °C" << std::endl;
            std::cout << "  LED:     " << temperature.led().value() << " °C" << std::endl;
            std::cout << "  Lens:    " << temperature.lens().value() << " °C" << std::endl;
            std::cout << "  PCB:     " << temperature.pcb().value() << " °C" << std::endl;
            std::cout << "  General: " << temperature.general().value() << " °C" << std::endl;
        }
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
