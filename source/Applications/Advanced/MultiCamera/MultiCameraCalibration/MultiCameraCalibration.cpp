/*
Use captures of a calibration object to generate transformation matrices to a single coordinate frame, from connected cameras.
*/

#include <clipp.h>

#include <Zivid/Calibration/Detector.h>
#include <Zivid/Zivid.h>
#include <iostream>
#include <vector>

namespace
{
    Zivid::Frame assistedCapture(Zivid::Camera &camera)
    {
        const auto parameters = Zivid::CaptureAssistant::SuggestSettingsParameters{
            Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency::none,
            Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{ std::chrono::milliseconds{ 800 } }
        };
        const auto settings = Zivid::CaptureAssistant::suggestSettings(camera, parameters);
        return camera.capture2D3D(settings);
    }

    std::vector<Zivid::Camera> connectToAllAvailableCameras(const std::vector<Zivid::Camera> &cameras)
    {
        std::vector<Zivid::Camera> connectedCameras;
        for(auto camera : cameras)
        {
            if(camera.state().status() == Zivid::CameraState::Status::available)
            {
                std::cout << "Connecting to camera: " << camera.info().serialNumber() << std::endl;
                camera.connect();
                connectedCameras.push_back(camera);
            }
            else
            {
                std::cout << "Camera " << camera.info().serialNumber() << "is not available. "
                          << "Camera status: " << camera.state().status() << std::endl;
            }
        }
        return connectedCameras;
    }
} // namespace

int main(int argc, char **argv)
{
    try
    {
        Zivid::Application zivid;

        std::string transformationMatricesSavePath;

        clipp::group cli;
        cli.push_back(
            clipp::values("Path to YAML files", transformationMatricesSavePath)
            % "Path where transformation matrix YAML files will be saved");

        if(!parse(argc, argv, cli))
        {
            auto fmt = clipp::doc_formatting{}.alternatives_min_split_size(1).surround_labels("\"", "\"");
            std::cout << "SYNOPSIS:" << std::endl;
            std::cout << clipp::usage_lines(cli, "MultiCameraCalibration", fmt) << std::endl;
            std::cout << "OPTIONS:" << std::endl;
            std::cout << clipp::documentation(cli) << std::endl;
            throw std::runtime_error("No path provided.");
        }

        std::cout << "Finding cameras" << std::endl;
        auto cameras = zivid.cameras();
        std::cout << "Number of cameras found: " << cameras.size() << std::endl;

        auto connectedCameras = connectToAllAvailableCameras(cameras);
        if(connectedCameras.size() < 2)
        {
            throw std::runtime_error("At least two cameras need to be connected");
        }
        std::cout << "Number of connected cameras: " << connectedCameras.size() << std::endl;

        auto detectionResults = std::vector<Zivid::Calibration::DetectionResult>();
        auto serialNumbers = std::vector<std::string>();
        for(auto &camera : connectedCameras)
        {
            const auto serial = camera.info().serialNumber().toString();
            std::cout << "Capturing frame with camera: " << serial << std::endl;
            const auto frame = assistedCapture(camera);
            std::cout << "Detecting checkerboard in point cloud" << std::endl;
            const auto detectionResult = Zivid::Calibration::detectCalibrationBoard(frame);
            if(detectionResult)
            {
                detectionResults.push_back(detectionResult);
                serialNumbers.push_back(serial);
            }
            else
            {
                throw std::runtime_error(
                    "Could not detect checkerboard. Please ensure it is visible from all cameras.");
            }
        }

        std::cout << "Performing Multi-camera calibration" << std::endl;
        const auto results = Zivid::Calibration::calibrateMultiCamera(detectionResults);
        if(results)
        {
            std::cout << "Multi-camera calibration OK" << std::endl;
            const auto &transforms = results.transforms();
            const auto &residuals = results.residuals();
            for(size_t i = 0; i < transforms.size(); ++i)
            {
                transforms[i].save(transformationMatricesSavePath + "/" + serialNumbers[i] + ".yaml");

                std::cout << transforms[i] << std::endl;
                std::cout << residuals[i] << std::endl;
            }
        }
        else
        {
            std::cout << "Multi-camera calibration FAILED" << std::endl;
        }
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
