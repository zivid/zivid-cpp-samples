/*
Capture a 2D+3D frame and a 2D frame from the Zivid camera with diagnostics enabled.

Enabling diagnostics allows collecting additional data to be saved in the ZDF file.
Send ZDF files with diagnostics enabled to the Zivid support team to allow more thorough troubleshooting.
The 2D frame ZDF (Frame2DWithDiagnostics.zdf) must be loaded using the Frame2D API.
Have in mind that enabling diagnostics increases the capture time and the RAM usage.

For more information on diagnostics, check out this article:
https://support.zivid.com/en/latest/reference-articles/settings/diagnostics.html
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

        std::cout << "Configuring settings for 2D+3D capture" << std::endl;
        auto settings = Zivid::Settings{ Zivid::Settings::Acquisitions{
                                             Zivid::Settings::Acquisition{ Zivid::Settings::Acquisition{} } },
                                         Zivid::Settings::Color{ Zivid::Settings2D{
                                             Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } } } };

        std::cout << "Enabling diagnostics" << std::endl;
        settings.set(Zivid::Settings::Diagnostics::Enabled::yes);

        std::cout << "Capturing 2D+3D frame" << std::endl;
        const auto frame = camera.capture2D3D(settings);

        const auto dataFile = "FrameWithDiagnostics.zdf";
        std::cout << "Saving frame with diagnostic data to file: " << dataFile << std::endl;
        frame.save(dataFile);

        std::cout << "Configuring settings for 2D capture" << std::endl;
        auto settings2D = Zivid::Settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } };

        std::cout << "Enabling 2D diagnostics" << std::endl;
        settings2D.set(Zivid::Settings2D::Diagnostics::Enabled::yes);

        std::cout << "Capturing 2D frame" << std::endl;
        const auto frame2D = camera.capture2D(settings2D);

        const auto dataFile2D = "Frame2DWithDiagnostics.zdf";
        std::cout << "Saving 2D frame with diagnostic data to file: " << dataFile2D << std::endl;
        frame2D.save(dataFile2D);
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
