/*
This example shows how to capture point clouds, with color, from the Zivid camera,
with settings from YML file. The YML files for this sample can be found under the main instructions
for Zivid samples.
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

        std::cout << "Creating settings from file" << std::endl;
        const auto settingsFile = std::string(ZIVID_SAMPLE_DATA_DIR) + "/Settings/Settings01.yml";
        const auto settings = Zivid::Settings(settingsFile);

        std::cout << "Capturing frame" << std::endl;
        const auto frame = camera.capture(settings);

        const auto *dataFile = "Frame.zdf";
        std::cout << "Saving frame to file: " << dataFile << std::endl;
        frame.save(dataFile);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        if(std::cin.get() == '\n')
        {
            return EXIT_FAILURE;
        }
    }
}
