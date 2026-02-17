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

    struct Detection
    {
        std::string serialNumber;
        Zivid::Calibration::DetectionResult detectionResult;

        Detection(std::string serial, Zivid::Calibration::DetectionResult result)
            : serialNumber(std::move(serial))
            , detectionResult(std::move(result))
        {}
    };

    // Detect checkerboard feature points from each camera capture
    std::vector<Detection> getDetections(
        const std::vector<Zivid::Camera> &connectedCameras,
        const std::string &settingsPath)
    {
        auto detectionsList = std::vector<Detection>();

        for(auto camera : connectedCameras)
        {
            const auto serial = camera.info().serialNumber().toString();
            std::cout << "Capturing frame with camera: " << serial << std::endl;
            if(settingsPath.empty())
            {
                const auto frame = Zivid::Calibration::captureCalibrationBoard(camera);
            }
            else
            {
                const auto settings = Zivid::Settings(settingsPath);
                const auto frame = camera.capture2D3D(settings);
            }
            const auto frame = Zivid::Calibration::captureCalibrationBoard(camera);
            std::cout << "Detecting checkerboard in point cloud" << std::endl;
            const auto detectionResult = Zivid::Calibration::detectCalibrationBoard(frame);
            if(detectionResult)
            {
                Detection currentDetection(serial, detectionResult);
                detectionsList.push_back(currentDetection);
            }
            else
            {
                throw std::runtime_error(
                    "Could not detect checkerboard. Please ensure it is visible from all cameras.");
            }
        }

        return detectionsList;
    }

    // Perform multi-camera calibration
    void runMultiCameraCalibration(
        std::vector<Detection> detectionsList,
        const std::string &transformationMatricesSavePath)
    {
        std::vector<Zivid::Calibration::DetectionResult> detectionResultsList;
        detectionResultsList.reserve(detectionsList.size());

        for(const auto &detection : detectionsList)
        {
            detectionResultsList.emplace_back(detection.detectionResult);
        }

        const auto results = Zivid::Calibration::calibrateMultiCamera(detectionResultsList);

        if(results)
        {
            std::cout << "Multi-camera calibration OK." << std::endl;
            const auto &transforms = results.transforms();
            const auto &residuals = results.residuals();
            for(size_t i = 0; i < transforms.size(); ++i)
            {
                transforms[i].save(transformationMatricesSavePath + "/" + detectionsList[i].serialNumber + ".yaml");

                std::cout << "Pose of camera " << detectionsList[i].serialNumber << " in first camera "
                          << detectionsList[0].serialNumber << " frame:\n"
                          << transforms[i] << std::endl;

                std::cout << residuals[i] << std::endl;
            }
        }
        else
        {
            std::cout << "Multi-camera calibration FAILED." << std::endl;
        }
    }
} // namespace

int main(int argc, char **argv)
{
    try
    {
        bool showHelp = false;
        std::string transformationMatricesSavePath;
        std::string settingsPath;
        auto cli =
            (clipp::option("-h", "--help").set(showHelp) % "Show help message",
             clipp::values("Path to YAML files", transformationMatricesSavePath)
                 % "Path where the transformation matrices YAML files will be saved",
             clipp::option("--settings-path")
                 & clipp::value("path", settingsPath) % "Path to the camera settings YML file");
        if(!clipp::parse(argc, argv, cli))
        {
            auto fmt = clipp::doc_formatting{}.alternatives_min_split_size(1).surround_labels("\"", "\"");
            std::cout << "SYNOPSIS:" << std::endl;
            std::cout << clipp::usage_lines(cli, "MultiCameraCalibration", fmt) << std::endl;
            std::cout << "OPTIONS:" << std::endl;
            std::cout << clipp::documentation(cli) << std::endl;
            if(showHelp)
            {
                return EXIT_SUCCESS;
            }
            throw std::runtime_error("No path provided.");
        }

        Zivid::Application zivid;

        std::cout << "Finding cameras" << std::endl;
        auto cameras = zivid.cameras();
        std::cout << "Number of cameras found: " << cameras.size() << std::endl;

        auto connectedCameras = connectToAllAvailableCameras(cameras);
        if(connectedCameras.size() < 2)
        {
            throw std::runtime_error("At least two cameras need to be connected");
        }
        std::cout << "Number of connected cameras: " << connectedCameras.size() << std::endl;

        // detect checkerboard feature points from Capture for each camera
        const auto detections = getDetections(connectedCameras, settingsPath);

        // Perform multi-camera calibration
        runMultiCameraCalibration(detections, transformationMatricesSavePath);
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
