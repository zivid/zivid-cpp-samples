/*
Capture and save a point cloud, with colors, using GenICam interface and Halcon C++ SDK.
*/

#include <Zivid/Zivid.h>
#include <halconcpp/HalconCpp.h>

#include <iostream>

std::string presetPath(const std::string &model)
{
    const std::string presetsPath = std::string(ZIVID_SAMPLE_DATA_DIR) + "/Settings";

    if(model.find("zividTwoL100") != std::string::npos)
        return presetsPath + "/Zivid_Two_L100_ManufacturingSpecular.yml";

    if(model.find("zividTwo") != std::string::npos) return presetsPath + "/Zivid_Two_M70_ManufacturingSpecular.yml";

    if(model.find("zivid2PlusM130") != std::string::npos)
        return presetsPath + "/Zivid_Two_Plus_M130_ConsumerGoodsQuality.yml";

    if(model.find("zivid2PlusM60") != std::string::npos)
        return presetsPath + "/Zivid_Two_Plus_M60_ConsumerGoodsQuality.yml";

    if(model.find("zivid2PlusL110") != std::string::npos)
        return presetsPath + "/Zivid_Two_Plus_L110_ConsumerGoodsQuality.yml";

    if(model.find("zivid2PlusMR130") != std::string::npos)
        return presetsPath + "/Zivid_Two_Plus_MR130_ConsumerGoodsQuality.yml";

    if(model.find("zivid2PlusMR60") != std::string::npos)
        return presetsPath + "/Zivid_Two_Plus_MR60_ConsumerGoodsQuality.yml";

    if(model.find("zivid2PlusLR110") != std::string::npos)
        return presetsPath + "/Zivid_Two_Plus_LR110_ConsumerGoodsQuality.yml";

    if(model.find("zivid3XL250") != std::string::npos)
        return presetsPath + "/Zivid_Three_XL250_DepalletizationQuality.yml";

    if(model.find("zividOnePlus") != std::string::npos)
        throw std::runtime_error("Unsupported Zivid One+ model: " + model);

    throw std::invalid_argument("Invalid camera model: " + model);
}

void savePointCloud(const HalconCpp::HObjectModel3D &model, const std::string &fileName)
{
    model.WriteObjectModel3d(
        HalconCpp::HString{ "ply" },
        HalconCpp::HString{ fileName.c_str() },
        HalconCpp::HString{ "invert_normals" },
        HalconCpp::HString{ "false" });
}

void setColorsInObjectModel3D(
    HalconCpp::HObjectModel3D &objectModel3D,
    const HalconCpp::HImage &RGB,
    const HalconCpp::HImage &zReduced)
{
    const HalconCpp::HRegion domain = zReduced.GetDomain();
    HalconCpp::HTuple rows, cols;
    domain.GetRegionPoints(&rows, &cols);

    objectModel3D
        .SetObjectModel3dAttribMod(HalconCpp::HTuple("red"), "points", RGB.AccessChannel(1).GetGrayval(rows, cols));
    objectModel3D
        .SetObjectModel3dAttribMod(HalconCpp::HTuple("green"), "points", RGB.AccessChannel(2).GetGrayval(rows, cols));
    objectModel3D
        .SetObjectModel3dAttribMod(HalconCpp::HTuple("blue"), "points", RGB.AccessChannel(3).GetGrayval(rows, cols));
}

HalconCpp::HString getFirstAvailableZividDevice()
{
    auto devices = HalconCpp::HTuple();
    auto information = HalconCpp::HTuple();
    HalconCpp::InfoFramegrabber("GenICamTL", "device", &information, &devices);

    auto zividDevices = devices.TupleRegexpSelect("Zivid");
    if(zividDevices.Length() == 0)
    {
        throw std::runtime_error("No Zivid devices found. Please check your setup.");
    }
    return zividDevices[0];
}

int main()
{
    try
    {
        std::cout << "Connecting to camera" << std::endl;
        const auto zividDevice = getFirstAvailableZividDevice();
        auto framegrabber = HalconCpp::HTuple();
        HalconCpp::OpenFramegrabber(
            "GenICamTL",
            1,
            1,
            0,
            0,
            0,
            0,
            "progressive",
            -1,
            "default",
            -1,
            "false",
            "default",
            zividDevice,
            0,
            0,
            &framegrabber);

        std::cout << "Configuring settings" << std::endl;
        HalconCpp::SetFramegrabberParam(framegrabber, "create_objectmodel3d", "enable");
        HalconCpp::SetFramegrabberParam(framegrabber, "add_objectmodel3d_overlay_attrib", "enable");

        HalconCpp::HTuple modelTuple;
        HalconCpp::GetFramegrabberParam(framegrabber, "CameraInfoModel", &modelTuple);

        const std::string modelName = modelTuple.ToString().Text();
        const HalconCpp::HString settingFile(presetPath(modelName).c_str());

        HalconCpp::SetFramegrabberParam(framegrabber, "LoadSettingsFromFile", settingFile);

        std::cout << "Capturing frame" << std::endl;
        auto region = HalconCpp::HRegion();
        auto contours = HalconCpp::HXLDCont();
        auto data = HalconCpp::HTuple();
        auto frame = HalconCpp::HImage();
        HalconCpp::GrabData(&frame, &region, &contours, framegrabber, &data);

        const auto x = frame.SelectObj(1);
        const auto y = frame.SelectObj(2);
        const auto z = frame.SelectObj(3);
        const auto snr = frame.SelectObj(4);
        const auto rgb = frame.SelectObj(5);

        std::cout << "Removing invalid 3D points (zeroes)" << std::endl;
        auto reducedRegion = HalconCpp::HObject();
        auto zReduced = HalconCpp::HImage();
        HalconCpp::Threshold(z, &reducedRegion, 0.0001, 10000);
        HalconCpp::ReduceDomain(z, reducedRegion, &zReduced);

        std::cout << "Constructing ObjectModel3D based on XYZ data" << std::endl;
        HalconCpp::HObjectModel3D objectModel3D(x, y, zReduced);

        std::cout << "Adding RGB to ObjectModel3D" << std::endl;
        setColorsInObjectModel3D(objectModel3D, rgb, zReduced);

        const auto pointCloudFile = "Zivid3D.ply";
        std::cout << "Saving point cloud to file: " << pointCloudFile << std::endl;
        savePointCloud(objectModel3D, pointCloudFile);
    }
    catch(HalconCpp::HException &except)
    {
        std::cerr << "Error: " << except.ErrorMessage() << std::endl;
        return EXIT_FAILURE;
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
