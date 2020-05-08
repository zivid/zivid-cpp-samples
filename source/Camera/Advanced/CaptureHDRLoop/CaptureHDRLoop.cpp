/*
This example shows how to cover the same dynamic range in a scene with different acquisition settings.
This possibility allows to optimize settings for quality, speed, or to find a compromise. The camera
captures multi acquisition HDR point clouds in a loop, with settings from YML files.
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
        for(size_t i = 1; i <= captures; i++)
        {
            std::stringstream settingsFile;
            settingsFile << std::string(ZIVID_SAMPLE_DATA_DIR) + "/Settings/Settings0" << i << ".yml";
            std::cout << "Configuring settings from file: " << settingsFile.str() << ":" << std::endl;
            const auto settings = Zivid::Settings(settingsFile.str());
            std::cout << settings << std::endl;

            std::cout << "Capturing HDR frame" << std::endl;
            const auto hdrFrame = camera.capture(settings);

            std::stringstream hdrPath;
            hdrPath << "HDR_" << i << ".zdf";
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
