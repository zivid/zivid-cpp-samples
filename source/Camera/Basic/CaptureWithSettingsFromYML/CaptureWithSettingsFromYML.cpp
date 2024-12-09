/*
Capture point clouds, with color, from the Zivid camera, with settings from YML file.

The YML files for this sample can be found under the main Zivid sample instructions.
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

        std::cout << "Loading settings from file" << std::endl;
        const auto settingsFile =
            std::string(ZIVID_SAMPLE_DATA_DIR) + "/Settings/" + settingsFolder(camera) + "/Settings01.yml";
        const auto settings = Zivid::Settings(settingsFile);

        std::cout << "Capturing frame" << std::endl;
        const auto frame = camera.capture(settings);

        const auto dataFile = "Frame.zdf";
        std::cout << "Saving frame to file: " << dataFile << std::endl;
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
