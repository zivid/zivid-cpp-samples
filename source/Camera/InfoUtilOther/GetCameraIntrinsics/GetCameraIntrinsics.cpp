/*
Read intrinsic parameters from the Zivid camera (OpenCV model) or estimate them from the point cloud.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.
*/

#include <Zivid/Experimental/Calibration.h>
#include <Zivid/Experimental/SettingsInfo.h>
#include <Zivid/Zivid.h>

#include <chrono>
#include <iostream>

namespace
{
    void printParameterDelta(const std::string &label, const double fixed_value, const double estimated_value)
    {
        const auto delta = fixed_value - estimated_value;
        if(std::abs(delta) > 0.0)
        {
            std::cout << std::right << std::setfill(' ') << std::setw(6) << label << ": ";
            std::cout << std::right << std::setfill(' ') << std::setw(6) << std::fixed << std::setprecision(2) << delta;
            std::cout << " (" << std::right << std::setfill(' ') << std::setw(6) << std::fixed << std::setprecision(2)
                      << (100 * delta) / fixed_value << "% )";
            std::cout << std::endl;
        }
    }

    void printIntrinsicParametersDelta(
        const Zivid::CameraIntrinsics &fixed_intrinsics,
        const Zivid::CameraIntrinsics &estimated_intrinsics)
    {
        printParameterDelta(
            "CX", fixed_intrinsics.cameraMatrix().cx().value(), estimated_intrinsics.cameraMatrix().cx().value());
        printParameterDelta(
            "CY", fixed_intrinsics.cameraMatrix().cy().value(), estimated_intrinsics.cameraMatrix().cy().value());
        printParameterDelta(
            "FX", fixed_intrinsics.cameraMatrix().fx().value(), estimated_intrinsics.cameraMatrix().fx().value());
        printParameterDelta(
            "FY", fixed_intrinsics.cameraMatrix().fy().value(), estimated_intrinsics.cameraMatrix().fy().value());

        printParameterDelta(
            "K1", fixed_intrinsics.distortion().k1().value(), estimated_intrinsics.distortion().k1().value());
        printParameterDelta(
            "K2", fixed_intrinsics.distortion().k2().value(), estimated_intrinsics.distortion().k2().value());
        printParameterDelta(
            "P1", fixed_intrinsics.distortion().p1().value(), estimated_intrinsics.distortion().p1().value());
        printParameterDelta(
            "P2", fixed_intrinsics.distortion().p2().value(), estimated_intrinsics.distortion().p2().value());
    }

    void printIntrinsicParametersWithDescription(const Zivid::CameraIntrinsics &intrinsics)
    {
        std::cout << "Separated camera intrinsic parameters with description:" << std::endl;

        std::cout << "    CX: " << std::left << std::setfill(' ') << std::setw(13) << std::fixed << std::setprecision(2)
                  << intrinsics.cameraMatrix().cx().value() << Zivid::CameraIntrinsics::CameraMatrix::CX::description
                  << std::endl;
        std::cout << "    CY: " << std::left << std::setfill(' ') << std::setw(13) << std::fixed << std::setprecision(2)
                  << intrinsics.cameraMatrix().cy().value() << Zivid::CameraIntrinsics::CameraMatrix::CY::description
                  << std::endl;
        std::cout << "    FX: " << std::left << std::setfill(' ') << std::setw(13) << std::fixed << std::setprecision(2)
                  << intrinsics.cameraMatrix().fx().value() << Zivid::CameraIntrinsics::CameraMatrix::FX::description
                  << std::endl;
        std::cout << "    FY: " << std::left << std::setfill(' ') << std::setw(13) << std::fixed << std::setprecision(2)
                  << intrinsics.cameraMatrix().fy().value() << Zivid::CameraIntrinsics::CameraMatrix::FY::description
                  << std::endl;

        std::cout << "    K1: " << std::left << std::setfill(' ') << std::setw(13) << std::fixed << std::setprecision(4)
                  << intrinsics.distortion().k1().value() << Zivid::CameraIntrinsics::Distortion::K1::description
                  << std::endl;
        std::cout << "    K2: " << std::left << std::setfill(' ') << std::setw(13) << std::fixed << std::setprecision(4)
                  << intrinsics.distortion().k2().value() << Zivid::CameraIntrinsics::Distortion::K2::description
                  << std::endl;
        std::cout << "    K3: " << std::left << std::setfill(' ') << std::setw(13) << std::fixed << std::setprecision(4)
                  << intrinsics.distortion().k3().value() << Zivid::CameraIntrinsics::Distortion::K3::description
                  << std::endl;
        std::cout << "    P1: " << std::left << std::setfill(' ') << std::setw(13) << std::fixed << std::setprecision(4)
                  << intrinsics.distortion().p1().value() << Zivid::CameraIntrinsics::Distortion::P1::description
                  << std::endl;
        std::cout << "    P2: " << std::left << std::setfill(' ') << std::setw(13) << std::fixed << std::setprecision(4)
                  << intrinsics.distortion().p2().value() << Zivid::CameraIntrinsics::Distortion::P2::description
                  << std::endl;
    }

    Zivid::Settings subsampledSettingsForCamera(const Zivid::Camera &camera)
    {
        auto settingsSubsampled = Zivid::Settings(
            Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} },
            Zivid::Settings::Color(
                Zivid::Settings2D(Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} })));

        auto model = camera.info().model();
        switch(model.value())
        {
            case Zivid::CameraInfo::Model::ValueType::zividTwo:
            case Zivid::CameraInfo::Model::ValueType::zividTwoL100:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusM130:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusM60:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusL110:
            {
                settingsSubsampled.set(Zivid::Settings::Sampling::Pixel::blueSubsample2x2);
                settingsSubsampled.color().value().set(Zivid::Settings2D::Sampling::Pixel::blueSubsample2x2);
                break;
            }
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusMR130:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusMR60:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusLR110:
            {
                settingsSubsampled.set(Zivid::Settings::Sampling::Pixel::by2x2);
                settingsSubsampled.color().value().set(Zivid::Settings2D::Sampling::Pixel::by2x2);
                break;
            }
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusSmall:
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusMedium:
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusLarge:
            {
                throw std::runtime_error("Unsupported camera model '" + model.toString() + "'");
            }
            default: throw std::runtime_error("Unhandled enum value '" + model.toString() + "'");
        }

        return settingsSubsampled;
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Getting camera intrinsics" << std::endl;
        const auto intrinsics = Zivid::Experimental::Calibration::intrinsics(camera);

        std::cout << intrinsics << std::endl;

        const auto outputFile = "Intrinsics.yml";
        std::cout << "Saving camera intrinsics to file: " << outputFile << std::endl;
        intrinsics.save(outputFile);

        printIntrinsicParametersWithDescription(intrinsics);

        std::cout
            << std::endl
            << "Difference between fixed intrinsics and estimated intrinsics for different apertures and temperatures:"
            << std::endl;

        for(const auto aperture : { 5.66, 4.00, 2.83 })
        {
            const auto settings = Zivid::Settings{
                Zivid::Settings::Acquisitions{
                    Zivid::Settings::Acquisition{ Zivid::Settings::Acquisition::Aperture{ aperture } } },
                Zivid::Settings::Color{ Zivid::Settings2D{ Zivid::Settings2D::Acquisitions{
                    Zivid::Settings2D::Acquisition{ Zivid::Settings2D::Acquisition::Aperture{ aperture } } } } }
            };
            const auto frame = camera.capture2D3D(settings);
            const auto estimated_intrinsics = Zivid::Experimental::Calibration::estimateIntrinsics(frame);
            const auto temperature = frame.state().temperature().lens().value();
            std::cout << std::endl
                      << "Aperture: " << std::fixed << std::setprecision(2) << aperture
                      << ", Lens Temperature: " << temperature << "\370C" << std::endl;
            printIntrinsicParametersDelta(intrinsics, estimated_intrinsics);
        }

        const auto settingsSubsampled = subsampledSettingsForCamera(camera);
        const auto fixedIntrinsicsForSubsampledSettingsPath = "FixedIntrinsicsSubsampled2x2.yml";
        std::cout << "Saving camera intrinsics for subsampled capture to file: "
                  << fixedIntrinsicsForSubsampledSettingsPath << std::endl;
        const auto fixedIntrinsicsForSubsampledSettings =
            Zivid::Experimental::Calibration::intrinsics(camera, settingsSubsampled);
        fixedIntrinsicsForSubsampledSettings.save(fixedIntrinsicsForSubsampledSettingsPath);
        const auto frame = camera.capture2D3D(settingsSubsampled);
        const auto estimatedIntrinsicsForSubsampledSettings =
            Zivid::Experimental::Calibration::estimateIntrinsics(frame);
        const auto estimatedIntrinsicsForSubsampledSettingsPath = "EstimatedIntrinsicsFromSubsampled2x2Capture.yml";
        std::cout << "Saving estimated camera intrinsics for subsampled capture to file: "
                  << estimatedIntrinsicsForSubsampledSettingsPath << std::endl;
        estimatedIntrinsicsForSubsampledSettings.save(estimatedIntrinsicsForSubsampledSettingsPath);
        std::cout << std::endl
                  << "Difference between fixed and estimated intrinsics for subsampled point cloud: " << std::endl;
        printIntrinsicParametersDelta(fixedIntrinsicsForSubsampledSettings, estimatedIntrinsicsForSubsampledSettings);

        const auto settings2D =
            Zivid::Settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } };
        std::cout << "Getting camera intrinsics for 2D settings" << std::endl;
        const auto fixedIntrinsicsForSettings2D = Zivid::Experimental::Calibration::intrinsics(camera, settings2D);
        std::cout << fixedIntrinsicsForSettings2D << std::endl;
        const auto fixedIntrinsicsForSettings2DPath = "FixedIntrinsicsSettings2D.yml";
        std::cout << "Saving camera intrinsics for 2D settings to file: " << fixedIntrinsicsForSettings2DPath
                  << std::endl;
        fixedIntrinsicsForSettings2D.save(fixedIntrinsicsForSettings2DPath);
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
