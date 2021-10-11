/*
This example shows how to capture point clouds, with color, from the Zivid camera.
For scenes with high dynamic range we combine multiple acquisitions to get an HDR
point cloud. This example shows how to fully configure settings for each acquisition.
In general, capturing an HDR point cloud is a lot simpler than this. The purpose of
this example is to demonstrate how to configure all the settings.
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

        std::cout << "Configuring global processing settings:" << std::endl;
        Zivid::Settings settings{
            Zivid::Settings::Experimental::Engine::phase,
            Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
            Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{ 1.5 },
            Zivid::Settings::Processing::Filters::Noise::Removal::Enabled::yes,
            Zivid::Settings::Processing::Filters::Noise::Removal::Threshold{ 7.0 },
            Zivid::Settings::Processing::Filters::Outlier::Removal::Enabled::yes,
            Zivid::Settings::Processing::Filters::Outlier::Removal::Threshold{ 5.0 },
            Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes,
            Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Enabled::yes,
            Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Strength{ 0.4 },
            Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Enabled::no,
            Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Threshold{ 0.5 },
            Zivid::Settings::Processing::Color::Balance::Red{ 1 },
            Zivid::Settings::Processing::Color::Balance::Green{ 1 },
            Zivid::Settings::Processing::Color::Balance::Blue{ 1 }
        };
        std::cout << settings.processing() << std::endl;

        std::cout << "Configuring base acquisition with settings same for all HDR acquisition:" << std::endl;
        const auto baseAcquisition = Zivid::Settings::Acquisition{ Zivid::Settings::Acquisition::Brightness{ 1.8 } };
        std::cout << baseAcquisition << std::endl;

        std::cout << "Configuring acquisition settings different for all HDR acquisitions" << std::endl;
        const std::vector<double> aperture{ 8.0, 4.0, 4.0 };
        const std::vector<double> gain{ 1.0, 1.0, 2.0 };
        const std::vector<size_t> exposureTime{ 10000, 10000, 40000 };
        for(size_t i = 0; i < aperture.size(); ++i)
        {
            std::cout << "Acquisition " << i + 1 << ":" << std::endl;
            std::cout << "  Exposure Time: " << exposureTime.at(i) << std::endl;
            std::cout << "  Aperture: " << aperture.at(i) << std::endl;
            std::cout << "  Gain: " << gain.at(i) << std::endl;
            const auto acquisitionSettings =
                baseAcquisition.copyWith(Zivid::Settings::Acquisition::Aperture{ aperture.at(i) },
                                         Zivid::Settings::Acquisition::Gain{ gain.at(i) },
                                         Zivid::Settings::Acquisition::ExposureTime{
                                             std::chrono::microseconds{ exposureTime.at(i) } });
            settings.acquisitions().emplaceBack(acquisitionSettings);
        }

        std::cout << "Capturing frame (HDR)" << std::endl;
        const auto frame = camera.capture(settings);

        std::cout << "Complete settings used:" << std::endl;
        std::cout << frame.settings() << std::endl;

        const auto *dataFile = "Frame.zdf";
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
}