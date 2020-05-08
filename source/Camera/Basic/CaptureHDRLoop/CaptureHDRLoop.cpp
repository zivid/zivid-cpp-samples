/*
This example shows how to acquire HDR images from the Zivid camera in a loop, with settings from files.
*/

#include <Zivid/Zivid.h>

#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to the camera" << std::endl;
        auto camera = zivid.connectCamera();

        const size_t captures = 3;
        const size_t framesPerCapture = 3;
        for(size_t set = 1; set <= captures; set++)
        {
            std::cout << "Configuring HDR settings" << std::endl;

            std::vector<Zivid::Settings> settingsVector;
            for(size_t frame = 1; frame <= framesPerCapture; frame++)
            {
                std::stringstream settingsPath;
                settingsPath << Zivid::Environment::dataPath() + "/Settings/Set" << set << "/Frame0" << frame << ".yml";
                std::cout << "Add settings from " << settingsPath.str() << ":" << std::endl;
                const auto setting = Zivid::Settings(settingsPath.str());
                std::cout << setting.toString() << std::endl;
                settingsVector.emplace_back(setting);
            }

            const auto hdrFrame = Zivid::HDR::capture(camera, settingsVector);

            std::stringstream hdrPath;
            hdrPath << "HDR_" << set << ".zdf";
            std::cout << "Saving the HDR to " << hdrPath.str() << std::endl;
            hdrFrame.save(hdrPath.str());
        }
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
