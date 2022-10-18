/*
Capture point clouds, with color, from the Zivid camera, with settings from YML file and diagnostics enabled.

Enabling diagnostics allows collecting additional data to be saved in the ZDF file.
Send ZDF files with diagnostics enabled to the Zivid support team to allow more thorough troubleshooting.
Have in mind that enabling diagnostics increases the capture time and the RAM usage.

The YML file for this sample can be found under the main instructions for Zivid samples.
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

        std::cout << "Configuring settings from file" << std::endl;
        const auto cameraModel = camera.info().model().toString().substr(0, 8);
        const auto settingsFile = std::string(ZIVID_SAMPLE_DATA_DIR) + "/Settings/" + cameraModel + "/Settings01.yml";
        auto settings = Zivid::Settings(settingsFile);

        std::cout << "Enabling diagnostics" << std::endl;
        settings.set(Zivid::Settings::Diagnostics::Enabled::yes);

        std::cout << "Capturing frame" << std::endl;
        const auto frame = camera.capture(settings);

        const auto dataFile = "FrameWithDiagnostics.zdf";
        std::cout << "Saving frame with diagnostic data to file: " << dataFile << std::endl;
        frame.save(dataFile);
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
