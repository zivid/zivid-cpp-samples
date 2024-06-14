/*
Capture point clouds, with color, from the Zivid camera with fully configured settings.

For scenes with high dynamic range we combine multiple acquisitions to get an HDR
point cloud. This example shows how to fully configure settings for each acquisition.
In general, capturing an HDR point cloud is a lot simpler than this. The purpose of
this example is to demonstrate how to configure all the settings.

This sample also demonstrates how to save and load settings from file.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.
*/

#include <Zivid/Zivid.h>

#include <iostream>

namespace
{
    using std::chrono::microseconds;

    std::tuple<std::vector<double>, std::vector<double>, std::vector<microseconds>, std::vector<double>>
    getExposureValues(const Zivid::Camera &camera)
    {
        const auto model = camera.info().model();
        switch(model.value())
        {
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusSmall:
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusMedium:
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusLarge: break;
            case Zivid::CameraInfo::Model::ValueType::zividTwo:
            case Zivid::CameraInfo::Model::ValueType::zividTwoL100:
            {
                const std::vector<double> apertures{ 5.66, 2.38, 2.1 };
                const std::vector<double> gains{ 1.0, 1.0, 1.0 };
                const std::vector<microseconds> exposureTimes{ microseconds{ 1677 },
                                                               microseconds{ 5000 },
                                                               microseconds{ 100000 } };
                const std::vector<double> brightnesses{ 1.8, 1.8, 1.8 };
                return { apertures, gains, exposureTimes, brightnesses };
            }
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusM130:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusM60:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusL110:
            {
                const std::vector<double> apertures{ 5.66, 2.8, 2.37 };
                const std::vector<double> gains{ 1.0, 1.0, 1.0 };
                const std::vector<microseconds> exposureTimes{ microseconds{ 1677 },
                                                               microseconds{ 5000 },
                                                               microseconds{ 100000 } };
                const std::vector<double> brightnesses{ 2.2, 2.2, 2.2 };
                return { apertures, gains, exposureTimes, brightnesses };
            }
        }
        throw std::runtime_error("Unhandled enum value '" + model.toString() + "'");
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Configuring settings for capture:" << std::endl;
        Zivid::Settings settings{
            Zivid::Settings::Engine::phase,
            Zivid::Settings::Sampling::Color::rgb,
            Zivid::Settings::Sampling::Pixel::blueSubsample2x2,
            Zivid::Settings::RegionOfInterest::Box::Enabled::yes,
            Zivid::Settings::RegionOfInterest::Box::PointO{ 1000, 1000, 1000 },
            Zivid::Settings::RegionOfInterest::Box::PointA{ 1000, -1000, 1000 },
            Zivid::Settings::RegionOfInterest::Box::PointB{ -1000, 1000, 1000 },
            Zivid::Settings::RegionOfInterest::Box::Extents{ -1000, 1000 },
            Zivid::Settings::RegionOfInterest::Depth::Enabled::yes,
            Zivid::Settings::RegionOfInterest::Depth::Range{ 200, 2000 },
            Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
            Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{ 1.5 },
            Zivid::Settings::Processing::Filters::Noise::Removal::Enabled::yes,
            Zivid::Settings::Processing::Filters::Noise::Removal::Threshold{ 7.0 },
            Zivid::Settings::Processing::Filters::Noise::Suppression::Enabled::yes,
            Zivid::Settings::Processing::Filters::Noise::Repair::Enabled::yes,
            Zivid::Settings::Processing::Filters::Outlier::Removal::Enabled::yes,
            Zivid::Settings::Processing::Filters::Outlier::Removal::Threshold{ 5.0 },
            Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes,
            Zivid::Settings::Processing::Filters::Reflection::Removal::Mode::global,
            Zivid::Settings::Processing::Filters::Cluster::Removal::Enabled::yes,
            Zivid::Settings::Processing::Filters::Cluster::Removal::MaxNeighborDistance{ 10 },
            Zivid::Settings::Processing::Filters::Cluster::Removal::MinArea{ 100 },
            Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Enabled::yes,
            Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Strength{ 0.4 },
            Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Enabled::no,
            Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Threshold{ 0.5 },
            Zivid::Settings::Processing::Filters::Hole::Repair::Enabled::yes,
            Zivid::Settings::Processing::Filters::Hole::Repair::HoleSize{ 0.2 },
            Zivid::Settings::Processing::Filters::Hole::Repair::Strictness{ 1 },
            Zivid::Settings::Processing::Resampling::Mode::upsample2x2,
            Zivid::Settings::Processing::Color::Balance::Red{ 1.0 },
            Zivid::Settings::Processing::Color::Balance::Green{ 1.0 },
            Zivid::Settings::Processing::Color::Balance::Blue{ 1.0 },
            Zivid::Settings::Processing::Color::Gamma{ 1.0 },
            Zivid::Settings::Processing::Color::Experimental::Mode::automatic
        };
        std::cout << settings << std::endl;

        std::cout << "Configuring base acquisition with settings same for all HDR acquisition:" << std::endl;
        const auto baseAcquisition = Zivid::Settings::Acquisition{};
        std::cout << baseAcquisition << std::endl;

        std::cout << "Configuring acquisition settings different for all HDR acquisitions" << std::endl;
        auto exposureValues = getExposureValues(camera);
        const std::vector<double> aperture = std::get<0>(exposureValues);
        const std::vector<double> gain = std::get<1>(exposureValues);
        const std::vector<std::chrono::microseconds> exposureTime = std::get<2>(exposureValues);
        const std::vector<double> brightness = std::get<3>(exposureValues);
        for(size_t i = 0; i < aperture.size(); ++i)
        {
            std::cout << "Acquisition " << i + 1 << ":" << std::endl;
            std::cout << "  Exposure Time: " << exposureTime.at(i).count() << std::endl;
            std::cout << "  Aperture: " << aperture.at(i) << std::endl;
            std::cout << "  Gain: " << gain.at(i) << std::endl;
            std::cout << "  Brightness: " << brightness.at(i) << std::endl;
            const auto acquisitionSettings = baseAcquisition.copyWith(
                Zivid::Settings::Acquisition::Aperture{ aperture.at(i) },
                Zivid::Settings::Acquisition::Gain{ gain.at(i) },
                Zivid::Settings::Acquisition::ExposureTime{ exposureTime.at(i) },
                Zivid::Settings::Acquisition::Brightness{ brightness.at(i) });
            settings.acquisitions().emplaceBack(acquisitionSettings);
        }

        std::cout << "Capturing frame (HDR)" << std::endl;
        const auto frame = camera.capture(settings);

        std::cout << "Complete settings used:" << std::endl;
        std::cout << frame.settings() << std::endl;

        const auto dataFile = "Frame.zdf";
        std::cout << "Saving frame to file: " << dataFile << std::endl;
        frame.save(dataFile);

        const auto settingsFile = "Settings.yml";
        std::cout << "Saving settings to file: " << settingsFile << std::endl;
        settings.save(settingsFile);

        std::cout << "Loading settings from file: " << settingsFile << std::endl;
        const auto settingsFromFile = Zivid::Settings(settingsFile);
        std::cout << settingsFromFile << std::endl;
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
