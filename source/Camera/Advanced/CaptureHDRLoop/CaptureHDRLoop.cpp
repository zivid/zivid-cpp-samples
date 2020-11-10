/*
This example shows how to cover the same dynamic range in a scene with different acquisition settings.
This possibility allows to optimize settings for quality, speed, or to find a compromise. The camera
captures multi acquisition HDR point clouds in a loop, with settings from YML files. The YML files for this sample can
be found under the main instructions for Zivid samples.
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

        const size_t captures = 3;
        for(size_t i = 1; i <= captures; i++)
        {
            std::stringstream settingsFile;
            settingsFile << std::string(ZIVID_SAMPLE_DATA_DIR) + "/Settings/Settings0" << i << ".yml";
            std::cout << "Configuring settings from file: " << settingsFile.str() << ":" << std::endl;
            const auto settings = Zivid::Settings(settingsFile.str());
            std::cout << settings << std::endl;

            std::cout << "Capturing frame (HDR)" << std::endl;
            const auto frame = camera.capture(settings);

            std::stringstream dataFile;
            dataFile << "Frame0" << i << ".zdf";
            std::cout << "Saving frame to file: " << dataFile.str() << std::endl;
            frame.save(dataFile.str());
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
