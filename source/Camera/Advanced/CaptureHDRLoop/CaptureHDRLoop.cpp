/*
Cover the same dynamic range in a scene with different acquisition settings to optimize for quality, speed, or to find a compromise.

The camera captures multi acquisition HDR point clouds in a loop, with settings from YML files.
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

        const size_t captures = 3;
        for(size_t i = 1; i <= captures; i++)
        {
            std::stringstream settingsFile;
            settingsFile << std::string(ZIVID_SAMPLE_DATA_DIR) + "/Settings/" << settingsFolder(camera) << "/Settings0"
                         << i << ".yml";
            std::cout << "Loading settings from file: " << settingsFile.str() << ":" << std::endl;
            const auto settings = Zivid::Settings(settingsFile.str());
            std::cout << settings << std::endl;

            std::cout << "Capturing frame (HDR)" << std::endl;
            const auto frame = camera.capture2D3D(settings);

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

    return EXIT_SUCCESS;
}
