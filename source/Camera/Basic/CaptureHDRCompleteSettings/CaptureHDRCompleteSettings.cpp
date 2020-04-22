/*
This example shows how to acquire an HDR image from the Zivid camera with fully
configured settings for each frame. In general, taking an HDR image is a lot
simpler than this as the default settings work for most scenes. The purpose of
this example is to demonstrate how to configure all the settings.
*/

#include <Zivid/Zivid.h>

#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to the camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Configuring settings same for all HDR frames" << std::endl;
        auto settingsDefault = Zivid::Settings();
        settingsDefault.set(Zivid::Settings::Brightness{ 1 })
            .set(Zivid::Settings::Bidirectional{ false })
            .set(Zivid::Settings::Filters::Contrast::Enabled::yes)
            .set(Zivid::Settings::Filters::Contrast::Threshold{ 5 })
            .set(Zivid::Settings::Filters::Gaussian::Enabled::yes)
            .set(Zivid::Settings::Filters::Gaussian::Sigma{ 1.5 })
            .set(Zivid::Settings::Filters::Outlier::Enabled::yes)
            .set(Zivid::Settings::Filters::Outlier::Threshold{ 5 })
            .set(Zivid::Settings::Filters::Reflection::Enabled::yes)
            .set(Zivid::Settings::Filters::Saturated::Enabled::yes)
            .set(Zivid::Settings::BlueBalance{ 1.081 })
            .set(Zivid::Settings::RedBalance{ 1.709 });

        std::cout << "Configuring settings different for all HDR frames" << std::endl;
        const std::vector<size_t> iris{ 17U, 27U, 27U };
        const std::vector<int> exposureTime{ 10000, 10000, 40000 };
        const std::vector<double> gain{ 1.0, 1.0, 2.0 };
        std::vector<Zivid::Settings> settingsHDR;
        for(size_t i = 0; i < iris.size(); ++i)
        {
            settingsHDR.push_back(
                settingsDefault.set(Zivid::Settings::Iris{ iris.at(i) })
                    .set(Zivid::Settings::ExposureTime{ std::chrono::microseconds{ exposureTime.at(i) } })
                    .set(Zivid::Settings::Gain{ gain.at(i) }));
            std::cout << "Frame " << i << " " << settingsHDR.at(i) << std::endl;
        }

        std::cout << "Capturing HDR frame" << std::endl;
        const auto hdrFrame = Zivid::HDR::capture(camera, settingsHDR);

        std::cout << "Saving the frame" << std::endl;
        hdrFrame.save("HDR.zdf");
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}