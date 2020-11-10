/*
This sample shows how to perform Multi-Camera calibration.
*/

#include <opencv2/core/core.hpp>

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
        return camera.capture(settings);
    }

    void saveTransformationMatricesToYAML(const std::vector<Zivid::Matrix4x4> &transforms,
                                          const std::vector<Zivid::Camera> &cameras,
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
            fileStorageOut.write("TransformationMatrix_" + std::to_string(i),
                                 cv::Mat(cv::Matx44f(transforms.at(i).data())));
            fileStorageOut.write("SerialNumber_" + std::to_string(i), cameras.at(i).info().serialNumber().toString());
        }
        fileStorageOut.release();
    }
} // namespace

int main()
{
    try
    {
        std::cout << "Finding cameras" << std::endl;
        Zivid::Application zivid;
        auto cameras = zivid.cameras();
        if(cameras.size() < 2)
        {
            throw std::runtime_error("At least two cameras need to be connected");
        }
        std::cout << "Number of cameras found: " << cameras.size() << std::endl;
        for(auto &camera : cameras)
        {
            std::cout << "Connecting to camera: " << camera.info().serialNumber() << std::endl;
            camera.connect();
        }

        auto detectionResults = std::vector<Zivid::Calibration::DetectionResult>();
        for(auto &camera : cameras)
        {
            std::cout << "Capturing frame with camera: " << camera.info().serialNumber() << std::endl;
            const auto frame = assistedCapture(camera);
            std::cout << "Detecting checkerboard in point cloud" << std::endl;
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

        std::cout << "Performing Multi-camera calibration" << std::endl;
        const auto results = Zivid::Calibration::calibrateMultiCamera(detectionResults);
        if(results)
        {
            std::cout << "Multi-camera calibration OK" << std::endl;
            const auto &transforms = results.transforms();
            const auto &residuals = results.residuals();
            for(size_t i = 0; i < transforms.size(); ++i)
            {
                std::cout << transforms[i] << std::endl;
                std::cout << residuals[i] << std::endl;
            }
            saveTransformationMatricesToYAML(transforms, cameras, "MultiCameraCalibrationResults.yaml");
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
}
