/*
Measure ambient light conditions in the scene and output the measured flickering frequency of the ambient light if flickering is detected.
*/

#include <Zivid/Presets.h>
#include <Zivid/Zivid.h>

#include <iostream>

namespace
{
    std::string findSettings2D3D(const Zivid::Camera &camera)
    {
        const std::string presetsPath = std::string(ZIVID_SAMPLE_DATA_DIR) + "/Settings";

        switch(camera.info().model().value())
        {
            case Zivid::CameraInfo::Model::ValueType::zividTwo:
            {
                return presetsPath + "/Zivid_Two_M70_ManufacturingSpecular.yml";
            }
            case Zivid::CameraInfo::Model::ValueType::zividTwoL100:
            {
                return presetsPath + "/Zivid_Two_L100_ManufacturingSpecular.yml";
            }
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusM130:
            {
                return presetsPath + "/Zivid_Two_Plus_M130_ConsumerGoodsQuality.yml";
            }
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusM60:
            {
                return presetsPath + "/Zivid_Two_Plus_M60_ConsumerGoodsQuality.yml";
            }
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusL110:
            {
                return presetsPath + "/Zivid_Two_Plus_L110_ConsumerGoodsQuality.yml";
            }
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusMR130:
            {
                return presetsPath + "/Zivid_Two_Plus_MR130_ConsumerGoodsQuality.yml";
            }
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusMR60:
            {
                return presetsPath + "/Zivid_Two_Plus_MR60_ConsumerGoodsQuality.yml";
            }
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusLR110:
            {
                return presetsPath + "/Zivid_Two_Plus_LR110_ConsumerGoodsQuality.yml";
            }
            case Zivid::CameraInfo::Model::ValueType::zivid3XL250:
            {
                return presetsPath + "/Zivid_Three_XL250_DepalletizationQuality.yml";
            }
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusSmall:
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusMedium:
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusLarge: break;

            default: throw std::runtime_error("Unhandled enum value '" + camera.info().model().toString() + "'");
        }
        throw std::invalid_argument("Invalid camera model");
    }

    std::string addSuffixBeforeExtension(const std::string &path, const std::string &suffix)
    {
        auto posDot = path.find_last_of('.');
        return path.substr(0, posDot) + suffix + path.substr(posDot);
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;
        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Connected to " << camera.info().serialNumber() << camera.info().modelName() << std::endl;

        std::cout << "Measuring scene conditions" << std::endl;
        auto sceneConditions = camera.measureSceneConditions();
        auto flickerClassification = sceneConditions.ambientLight().flickerClassification().toString();
        std::cout << "Flicker classification: " << flickerClassification << std::endl;

        if(flickerClassification != "noFlicker")
        {
            auto flickerFrequency = sceneConditions.ambientLight().flickerFrequency();
            std::cout << "Measured flickering frequency in the scene: " << flickerFrequency << " Hz." << std::endl;
        }

        std::string settingsPath;
        if(flickerClassification != "unknownFlicker")
        {
            settingsPath = findSettings2D3D(camera);
        }

        if(flickerClassification == "noFlicker")
        {
            std::cout << "No flickering lights were detected in the scene." << std::endl;
        }
        else if(flickerClassification == "unknownFlicker")
        {
            std::cout << "Flickering not found to match any known grid frequency." << std::endl;
            std::cout
                << "This is a non-standard flickering frequency. Consider adjusting the exposure time to be a multiple of this frequency to avoid artifacts."
                << std::endl;
            return EXIT_SUCCESS;
        }
        else if(flickerClassification == "grid50hz")
        {
            std::cout << "Found flickering corresponding to 50 Hz frequency in the scene, applying compensated preset:"
                      << std::endl;
            settingsPath = addSuffixBeforeExtension(settingsPath, "_50Hz");
        }
        else if(flickerClassification == "grid60hz")
        {
            std::cout << "Found flickering corresponding to 60 Hz frequency in the scene, applying compensated preset:"
                      << std::endl;
            settingsPath = addSuffixBeforeExtension(settingsPath, "_60Hz");
        }
        else
        {
            std::cout << "Invalid flicker classification" << std::endl;
            return EXIT_FAILURE;
        }
        std::cout << settingsPath << std::endl;
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
