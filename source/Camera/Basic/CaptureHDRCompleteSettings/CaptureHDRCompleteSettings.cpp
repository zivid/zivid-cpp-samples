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

        std::cout << "Connecting to the camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Creating settings same for all HDR acquisitions" << std::endl;
        Zivid::Settings settings{
            Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
            Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{ 1.5 },
            Zivid::Settings::Processing::Filters::Noise::Removal::Enabled::yes,
            Zivid::Settings::Processing::Filters::Noise::Removal::Threshold{ 10.0 },
            Zivid::Settings::Processing::Filters::Outlier::Removal::Enabled::yes,
            Zivid::Settings::Processing::Filters::Outlier::Removal::Threshold{ 5.0 },
            Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes,
            Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Enabled::yes,
            Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Strength{ 0.4 },
            Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Enabled::yes,
            Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Threshold{ 0.5 },
            Zivid::Settings::Processing::Color::Balance::Red{ 1 },
            Zivid::Settings::Processing::Color::Balance::Green{ 1 },
            Zivid::Settings::Processing::Color::Balance::Blue{ 1 }
        };
        std::cout << settings.processing() << std::endl;

        std::cout << "Creating base acquisition with settings same for all HDR acquisitions" << std::endl;
        const auto baseAcquisition =
            Zivid::Settings::Acquisition{ Zivid::Settings::Acquisition::Brightness{ 1.8 },
                                          Zivid::Settings::Acquisition::Patterns::Sine::Bidirectional{ false } };
        std::cout << baseAcquisition << std::endl;

        std::cout << "Configuring acquisition settings different for all HDR acquisitions" << std::endl;
        const std::vector<double> aperture{ 8.0, 4.0, 4.0 };
        const std::vector<double> gain{ 1.0, 1.0, 2.0 };
        const std::vector<size_t> exposureTime{ 10000, 10000, 40000 };
        const std::vector<double> gain{ 1.0, 1.0, 2.0 };
        for(size_t i = 0; i < aperture.size(); ++i)
        {
            const auto acquisitionSettings =
                baseAcquisition.copyWith(Zivid::Settings::Acquisition::Aperture{ aperture.at(i) },
                                         Zivid::Settings::Acquisition::Gain{ gain.at(i) },
                                         Zivid::Settings::Acquisition::ExposureTime{
                                             std::chrono::microseconds{ exposureTime.at(i) } });
            settings.acquisitions().emplaceBack(acquisitionSettings);
        }

        std::cout << "Capturing HDR frame" << std::endl;
        const auto hdrFrame = camera.capture(settings);

        std::cout << "Used settings:" << std::endl;
        std::cout << hdrFrame.settings() << std::endl;

        std::cout << "Saving the HDR frame" << std::endl;
        hdrFrame.save("HDR.zdf");
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}