/*
Capture point clouds, with color, from the Zivid camera, with settings from YML file and diagnostics enabled.

Enabling diagnostics allows collecting additional data to be saved in the ZDF file.
Send ZDF files with diagnostics enabled to the Zivid support team to allow more thorough troubleshooting.
Have in mind that enabling diagnostics increases the capture time and the RAM usage.

The YML file for this sample can be found under the main instructions for Zivid samples.
*/

#include <Zivid/Zivid.h>

#include <iostream>

namespace
{
    std::string settingsFolder(const Zivid::Camera &camera)
    {
        switch(camera.info().model().value())
        {
            case Zivid::CameraInfo::Model::ValueType::zividTwo:
            case Zivid::CameraInfo::Model::ValueType::zividTwoL100: return "zivid2";
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusM130:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusM60:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusL110: return "zivid2Plus";
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusMR130:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusMR60:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusLR110: return "zivid2Plus/R";
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusSmall:
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusMedium:
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusLarge: break;

            default: throw std::runtime_error("Unhandled enum value '" + camera.info().model().toString() + "'");
        }
        throw std::invalid_argument("Invalid camera model");
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Configuring settings from file" << std::endl;
        const auto settingsFile =
            std::string(ZIVID_SAMPLE_DATA_DIR) + "/Settings/" + settingsFolder(camera) + "/Settings01.yml";
        auto settings = Zivid::Settings(settingsFile);

        std::cout << "Enabling diagnostics" << std::endl;
        settings.set(Zivid::Settings::Diagnostics::Enabled::yes);

        std::cout << "Capturing frame" << std::endl;
        const auto frame = camera.capture2D3D(settings);

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
