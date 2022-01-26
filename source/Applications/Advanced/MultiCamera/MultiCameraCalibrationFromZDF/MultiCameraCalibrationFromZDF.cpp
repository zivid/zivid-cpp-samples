/*
Use captures of a calibration object to generate transformation matrices to a single coordinate frame, from a ZDF files.
*/

#include <opencv2/core/core.hpp>

#include <clipp.h>

#include <Zivid/Zivid.h>
#include <cmath>
#include <iostream>
#include <vector>

namespace
{
    void saveTransformationMatricesToYAML(
        const std::vector<Zivid::Matrix4x4> &transforms,
        const std::vector<std::string> &fileList,
        const std::string &path)
    {
        // Save Transformation Matrices to .YAML file
        cv::FileStorage fileStorageOut;
        if(!fileStorageOut.open(path, cv::FileStorage::Mode::WRITE))
        {
            throw std::runtime_error("Could not open " + path + " for writing");
        }
        for(size_t i = 0; i < transforms.size(); ++i)
        {
            fileStorageOut
                .write("TransformationMatrix_" + std::to_string(i), cv::Mat(cv::Matx44f(transforms.at(i).data())));
            std::string serialNumberToWrite = fileList.at(i);
            // Exclude non-alphanumeric prefix from file name
            size_t firstIndex = std::distance(
                serialNumberToWrite.begin(),
                std::find_if(serialNumberToWrite.begin(), serialNumberToWrite.end(), [](unsigned char c) {
                    return std::isalnum(c);
                }));
            size_t lastIndex = serialNumberToWrite.length();
            // Exclude extension from file name
            if(serialNumberToWrite.find_last_of('.') != std::string::npos)
            {
                lastIndex = serialNumberToWrite.find_last_of('.');
            }
            fileStorageOut.write(
                "SerialNumber_" + std::to_string(i), serialNumberToWrite.substr(firstIndex, lastIndex - firstIndex));
        }
        fileStorageOut.release();
    }
} // namespace

int main(int argc, char **argv)
{
    try
    {
        Zivid::Application zivid;

        auto transformationMatricesFileName = std::string("MultiCameraCalibrationResults.yaml");
        auto fileList = std::vector<std::string>{};
        auto cli =
            (clipp::values("File Names", fileList) % "List of .zdf files which contain captures of checker boards",
             clipp::option("-o")
                 & clipp::value("Transformation Matrices File Name", transformationMatricesFileName)
                       % "Path to .yaml file with all transformation matrices");

        if(!parse(argc, argv, cli))
        {
            auto fmt = clipp::doc_formatting{}.alternatives_min_split_size(1).surround_labels("\"", "\"");
            std::cout << "SYNOPSIS:" << std::endl;
            std::cout << clipp::usage_lines(cli, "MultiCameraCalibrationFromZDF", fmt) << std::endl;
            std::cout << "OPTIONS:" << std::endl;
            std::cout << clipp::documentation(cli) << std::endl;
            throw std::runtime_error("No files provided.");
        }

        // Read from .zdf and detect checkerboard feature points
        auto detectionResults = std::vector<Zivid::Calibration::DetectionResult>();
        for(std::string &fileName : fileList)
        {
            std::cout << "Reading " << fileName << " point cloud" << std::endl;
            const auto frame = Zivid::Frame(fileName);

            std::cout << "Detecting checkerboard in point cloud..." << std::endl;
            const auto detectionResult = Zivid::Calibration::detectFeaturePoints(frame.pointCloud());
            if(detectionResult)
            {
                detectionResults.push_back(detectionResult);
            }
            else
            {
                throw std::runtime_error(
                    "Could not detect checkerboard. Please ensure it is visible from all cameras.");
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
                std::cout << transforms[i] << std::endl;
                std::cout << residuals[i] << std::endl;
            }
            saveTransformationMatricesToYAML(transforms, fileList, transformationMatricesFileName);
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
}
