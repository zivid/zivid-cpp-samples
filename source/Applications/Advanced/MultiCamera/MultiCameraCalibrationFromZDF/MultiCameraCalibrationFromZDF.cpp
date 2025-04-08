/*
Use captures of a calibration object to generate transformation matrices to a single coordinate frame, from a ZDF files.
*/

#include <clipp.h>

#include <Zivid/Calibration/Detector.h>
#include <Zivid/Zivid.h>
#include <cmath>
#include <iostream>
#include <vector>

int main(int argc, char **argv)
{
    try
    {
        Zivid::Application zivid;

        std::string transformationMatricesSavePath;

        auto zdfFileList = std::vector<std::string>{};
        auto cli =
            (clipp::values("File Names", zdfFileList) % "List of ZDF files which contain captures of checker boards",
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

        // Read from ZDF and detect checkerboard feature points
        auto detectionResults = std::vector<Zivid::Calibration::DetectionResult>();
        auto serialNumbers = std::vector<std::string>();

        for(std::string &fileName : zdfFileList)
        {
            std::cout << "Reading " << fileName << " point cloud" << std::endl;
            const auto frame = Zivid::Frame(fileName);
            const auto serial = frame.cameraInfo().serialNumber().toString();

            std::cout << "Detecting checkerboard in point cloud..." << std::endl;
            const auto detectionResult = Zivid::Calibration::detectCalibrationBoard(frame);
            if(detectionResult.valid())
            {
                detectionResults.push_back(detectionResult);
                serialNumbers.push_back(serial);
            }
            else
            {
                throw std::runtime_error(
                    "Could not detect checkerboard. Please ensure it is visible from all cameras. "
                    + detectionResult.statusDescription());
            }
        }

        // Perform multi-camera calibration
        const auto results = Zivid::Calibration::calibrateMultiCamera(detectionResults);
        if(results)
        {
            std::cout << "Multi-camera calibration OK." << std::endl;
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
            std::cout << "Multi-camera calibration FAILED." << std::endl;
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
