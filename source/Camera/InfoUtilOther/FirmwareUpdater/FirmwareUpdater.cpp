/*
Update firmware on the Zivid camera.
*/

#include <Zivid/Zivid.h>

#include <cmath>
#include <iostream>
#include <thread>

int main()
{
    try
    {
        Zivid::Application zivid;

        auto cameras = zivid.cameras();
        if(cameras.empty())
        {
            std::cout << "No camera found." << std::endl;
            return EXIT_FAILURE;
        }
        std::cout << "Found " << cameras.size() << " camera(s)." << std::endl;
        for(auto &camera : cameras)
        {
            if(!Zivid::Firmware::isUpToDate(camera))
            {
                std::cout << "Firmware update required" << std::endl;
                std::cout << "Updating firmware on camera " << camera.info().serialNumber()
                          << ", model name: " << camera.info().modelName()
                          << ", firmware version: " << camera.info().firmwareVersion() << std::endl;
                Zivid::Firmware::update(camera, [](double progressPercentage, const std::string &stageDescription) {
                    std::cout << std::round(progressPercentage) << " % : " << stageDescription
                              << (progressPercentage < 100 ? "..." : "") << std::endl;
                });
            }
            else
            {
                std::cout << "Skipping update of camera " << camera.info().serialNumber()
                          << ", model name: " << camera.info().modelName()
                          << ", firmware version: " << camera.info().firmwareVersion() << std::endl;
            }
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
