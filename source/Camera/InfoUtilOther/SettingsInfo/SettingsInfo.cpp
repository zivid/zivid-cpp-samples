/*
Read settings info from the Zivid camera.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.
*/

#include <Zivid/Experimental/SettingsInfo.h>
#include <Zivid/Zivid.h>

#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        const auto cameraInfo = camera.info();

        std::cout << "Default camera settings:" << std::endl;
        auto defaultSettings = Zivid::Experimental::SettingsInfo::defaultValue<Zivid::Settings>(cameraInfo);
        defaultSettings.acquisitions().emplaceBack(
            Zivid::Experimental::SettingsInfo::defaultValue<Zivid::Settings::Acquisition>(cameraInfo));

        defaultSettings.set(Zivid::Settings::Color{
            Zivid::Experimental::SettingsInfo::defaultValue<Zivid::Settings2D>(cameraInfo)
                .copyWith(Zivid::Settings2D::Acquisitions{
                    Zivid::Experimental::SettingsInfo::defaultValue<Zivid::Settings2D::Acquisition>(cameraInfo) }) });

        std::cout << defaultSettings << std::endl;

        std::cout << "Default camera (e.g., Aperture) setting value:" << std::endl;
        const auto defaultSettingValue =
            Zivid::Experimental::SettingsInfo::defaultValue<Zivid::Settings::Acquisition::Aperture>(cameraInfo);
        const auto defaultSettingValue2D =
            Zivid::Experimental::SettingsInfo::defaultValue<Zivid::Settings2D::Acquisition::Aperture>(cameraInfo);
        std::cout << "  3D: " << defaultSettingValue << "\n  2D: " << defaultSettingValue2D << std::endl;

        std::cout << "Valid camera (e.g., Aperture) setting range (for settings of types double and duration):"
                  << std::endl;
        const auto validSettingRange =
            Zivid::Experimental::SettingsInfo::validRange<Zivid::Settings::Acquisition::Aperture>(cameraInfo);
        const auto validSettingRange2D =
            Zivid::Experimental::SettingsInfo::validRange<Zivid::Settings2D::Acquisition::Aperture>(cameraInfo);
        std::cout << "  3D: " << validSettingRange << "\n  2D: " << validSettingRange2D << std::endl;

        std::cout << "Valid camera (e.g., Reflection Filter) setting values (for settings of types bool and enum):"
                  << std::endl;
        const auto validSettingValues = Zivid::Experimental::SettingsInfo::validValues<
            Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled>(cameraInfo);
        for(const auto &value : validSettingValues)
        {
            std::cout << value << std::endl;
        }

        std::cout << "Camera resolution for default settings:" << std::endl;
        const auto resolution = Zivid::Experimental::SettingsInfo::resolution(cameraInfo, defaultSettings);
        std::cout << "  Height: " << resolution.height() << std::endl;
        std::cout << "  Width: " << resolution.width() << std::endl;

        std::cout << "Point cloud (GPU memory) resolution:" << std::endl;
        const auto pointCloud = camera.capture2D3D(defaultSettings).pointCloud();
        std::cout << "  Height: " << pointCloud.height() << std::endl;
        std::cout << "  Width: " << pointCloud.width() << std::endl;

        std::cout << "Point cloud (CPU memory) resolution:" << std::endl;
        const auto data = pointCloud.copyPointsXYZColorsRGBA();
        std::cout << "  Height: " << data.height() << std::endl;
        std::cout << "  Width: " << data.width() << std::endl;
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
