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
                std::cout << "Updating firmware on camera " << camera.serialNumber()
                          << ", model name: " << camera.modelName()
                          << ", firmware version: " << camera.firmwareVersion() << std::endl;
                Zivid::Firmware::update(camera, [](double progressPercentage, const std::string &stageDescription) {
                    std::cout << std::round(progressPercentage) << " % : " << stageDescription
                              << (progressPercentage < 100 ? "..." : "") << std::endl;
                });
            }
            else
            {
                std::cout << "Skipping update of camera " << camera.serialNumber()
                          << ", model name: " << camera.modelName()
                          << ", firmware version: " << camera.firmwareVersion() << std::endl;
            }
        }
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
