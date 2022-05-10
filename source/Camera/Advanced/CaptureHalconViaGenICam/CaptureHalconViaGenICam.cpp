/*
Capture and save a point cloud, with colors, using GenICam interface and Halcon C++ SDK.
*/

#include <Zivid/Zivid.h>
#include <halconcpp/HalconCpp.h>

#include <chrono>
#include <iostream>

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
        HalconCpp::SetFramegrabberParam(framegrabber, "AcquisitionMode", "SingleFrame");

        HalconCpp::SetFramegrabberParam(framegrabber, "Aperture", 5.66);
        HalconCpp::SetFramegrabberParam(framegrabber, "ExposureTime", 8333);
        HalconCpp::SetFramegrabberParam(framegrabber, "Gain", 2);
        HalconCpp::SetFramegrabberParam(framegrabber, "Brightness", 1.0);
        HalconCpp::SetFramegrabberParam(framegrabber, "ProcessingFiltersOutlierRemovalEnabled", 1);
        HalconCpp::SetFramegrabberParam(framegrabber, "ProcessingFiltersOutlierRemovalThreshold", 5);
        HalconCpp::SetFramegrabberParam(framegrabber, "ProcessingFiltersSmoothingGaussianEnabled", 1);
        HalconCpp::SetFramegrabberParam(framegrabber, "ProcessingFiltersSmoothingGaussianSigma", 1.5);

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
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
