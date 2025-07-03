/*
Measure ambient light conditions in the scene and output the measured flickering frequency of the ambient light if flickering is detected.
*/

#include <Zivid/Presets.h>
#include <Zivid/Zivid.h>

#include <iostream>

namespace
{
    std::string sanitizedModelName(const Zivid::Camera &camera)
    {
        using Model = Zivid::CameraInfo::Model::ValueType;
        switch(camera.info().model().value())
        {
            case Model::zividTwo: return "Zivid_Two_M70";
            case Model::zividTwoL100: return "Zivid_Two_L100";
            case Model::zivid2PlusM130: return "Zivid_Two_Plus_M130";
            case Model::zivid2PlusM60: return "Zivid_Two_Plus_M60";
            case Model::zivid2PlusL110: return "Zivid_Two_Plus_L110";
            case Model::zivid2PlusMR130: return "Zivid_Two_Plus_MR130";
            case Model::zivid2PlusMR60: return "Zivid_Two_Plus_MR60";
            case Model::zivid2PlusLR110: return "Zivid_Two_Plus_LR110";
            case Model::zividOnePlusSmall: return "Zivid_One_Plus_Small";
            case Model::zividOnePlusMedium: return "Zivid_One_Plus_Medium";
            case Model::zividOnePlusLarge: return "Zivid_One_Plus_Large";

            default: throw std::runtime_error("Unhandled camera model: " + camera.info().model().toString());
        }
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
        std::cout << flickerClassification << std::endl;

        if(flickerClassification == "noFlicker")
        {
            std::cout << "No flickering lights were detected in the scene." << std::endl;
            const auto settingsPath = std::string(ZIVID_SAMPLE_DATA_DIR) + "/Settings/" + sanitizedModelName(camera)
                                      + "_ConsumerGoodsFast.yml";
            std::cout << settingsPath << std::endl;
            return EXIT_SUCCESS;
        }

        if(flickerClassification == "unknownFlicker")
        {
            auto unknownFlickerFrequency = sceneConditions.ambientLight().flickerFrequency();
            std::cout << "Flickering not found to match any known grid frequency." << std::endl;
            std::cout << "Measured flickering frequency in the scene is: " << unknownFlickerFrequency << "Hz."
                      << std::endl;
            std::cout
                << "This is a non-standard flickering frequency. Consider adjusting the exposure time to be a multiple of this frequency to avoid artifacts."
                << std::endl;
            return EXIT_SUCCESS;
        }
        if(flickerClassification == "grid50hz")
        {
            std::cout << "Found flickering corresponding to 50 Hz frequency in the scene, applying compensated preset:"
                      << std::endl;
            const auto settings50HzPath = std::string(ZIVID_SAMPLE_DATA_DIR) + "/Settings/" + sanitizedModelName(camera)
                                          + "_ConsumerGoodsFast_50Hz.yml";
            std::cout << settings50HzPath << std::endl;
        }
        else if(flickerClassification == "grid60hz")
        {
            std::cout << "Found flickering corresponding to 60 Hz frequency in the scene, applying compensated preset:"
                      << std::endl;
            const auto settings60HzPath = std::string(ZIVID_SAMPLE_DATA_DIR) + "/Settings/" + sanitizedModelName(camera)
                                          + "_ConsumerGoodsFast_60Hz.yml";
            std::cout << settings60HzPath << std::endl;
        }

        auto flickerFrequency = sceneConditions.ambientLight().flickerFrequency();
        std::cout << "Measured flickering frequency in the scene: " << flickerFrequency << " Hz." << std::endl;
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
