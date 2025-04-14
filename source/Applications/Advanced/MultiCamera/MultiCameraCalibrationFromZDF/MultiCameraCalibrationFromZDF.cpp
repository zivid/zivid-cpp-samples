/*
Use captures of a calibration object to generate transformation matrices to a single coordinate frame, from a ZDF files.
*/

#include <clipp.h>

#include <Zivid/Calibration/Detector.h>
#include <Zivid/Zivid.h>
#include <iostream>
#include <vector>

namespace
{
    struct Detection
    {
        std::string serialNumber;
        Zivid::Calibration::DetectionResult detectionResult;

        Detection(std::string serial, Zivid::Calibration::DetectionResult result)
            : serialNumber(std::move(serial))
            , detectionResult(std::move(result))
        {}
    };

    // Read from ZDF and detect checkerboard feature points
    std::vector<Detection> getDetectionsFromZDF(const std::vector<std::string> &zdfFileList)
    {
        auto detectionsList = std::vector<Detection>();

        for(const std::string &fileName : zdfFileList)
        {
            std::cout << "Reading " << fileName << " point cloud" << std::endl;
            const auto frame = Zivid::Frame(fileName);
            const auto serial = frame.cameraInfo().serialNumber().toString();

            std::cout << "Detecting checkerboard in point cloud..." << std::endl;
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
        Zivid::Application zivid;

        std::string transformationMatricesSavePath;

        auto zdfFileList = std::vector<std::string>{};
        auto cli =
            (clipp::required("-zdf")
                 & clipp::values("ZDF filenames", zdfFileList)
                       % "List of ZDF files which contain captures of checker boards",
             clipp::required("-o", "--output-dir")
                 & clipp::value(
                     "Transformation Matrices Files will be saved into this directory",
                     transformationMatricesSavePath));

        if(!parse(argc, argv, cli))
        {
            auto fmt = clipp::doc_formatting{}.alternatives_min_split_size(1).surround_labels("\"", "\"");
            std::cout << "SYNOPSIS:" << std::endl;
            std::cout << clipp::usage_lines(cli, "MultiCameraCalibrationFromZDF", fmt) << std::endl;
            std::cout << "OPTIONS:" << std::endl;
            std::cout << clipp::documentation(cli) << std::endl;
            throw std::runtime_error("No files provided.");
        }

        // Read from ZDF and detect checkerboard feature
        const auto detections = getDetectionsFromZDF(zdfFileList);

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
