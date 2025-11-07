/*
Correct the dimension trueness of a Zivid camera.

This example shows how to perform In-field correction. This involves gathering data from
a compatible calibration board at several distances, calculating an updated camera
correction, and optionally saving that new correction to the camera.

The correction will persist on the camera even though the camera is power-cycled or
connected to a different PC. After saving a correction, it will automatically be used any
time that camera captures a new point cloud.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.
*/

#include <Zivid/Calibration/Detector.h>
#include <Zivid/Calibration/InfieldCorrection.h>
#include <Zivid/Zivid.h>

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace
{
    bool yesNoPrompt(const std::string &question)
    {
        while(true)
        {
            std::string response;
            std::cout << question << ": y/n " << std::endl;
            std::cin >> response;

            if(std::cin.fail() || response == "n" || response == "N")
            {
                return false;
            }
            if(response == "y" || response == "Y")
            {
                return true;
            }
            std::cout << "Invalid response. Please respond with either 'y' or 'n'." << std::endl;
        }
    }

    std::vector<Zivid::Calibration::InfieldCorrectionInput> collectDataset(Zivid::Camera &camera)
    {
        std::vector<Zivid::Calibration::InfieldCorrectionInput> dataset;
        std::cout << "Please point the camera at a Zivid infield calibration board. " << std::endl;

        const std::string printLine = "------------------------------------------------------------------------";
        while(true)
        {
            std::cout << printLine << std::endl;
            if(yesNoPrompt("Capture (y) or finish (n)?"))
            {
                std::cout << "Capturing calibration board" << std::endl;
                const auto detectionResult = Zivid::Calibration::detectCalibrationBoard(camera);
                if(detectionResult.valid())
                {
                    const auto input = Zivid::Calibration::InfieldCorrectionInput{ detectionResult };

                    if(input.valid())
                    {
                        dataset.emplace_back(input);
                        std::cout << "Valid measurement at: " << input.detectionResult().centroid() << std::endl;
                    }
                    else
                    {
                        std::cout << "****Invalid Input****" << std::endl;
                        std::cout << "Feedback: " << input.statusDescription() << std::endl;
                    }
                }
                else
                {
                    std::cout << "****Failed Detection****" << std::endl;
                    std::cout << "Feedback: " << detectionResult.statusDescription() << std::endl;
                }
                std::cout << printLine << std::endl;
            }
            else
            {
                std::cout << "End of capturing stage." << std::endl;
                std::cout << printLine << std::endl;
                break;
            }
            std::cout << "You have collected " << dataset.size() << " valid measurements so far." << std::endl;
        }
        return dataset;
    }

} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        // Gather data
        const auto dataset = collectDataset(camera);

        // Calculate infield correction
        std::cout << "Collected " << dataset.size() << " valid measurements." << std::endl;
        std::cout << "Computing new camera correction..." << std::endl;
        const auto correction = Zivid::Calibration::computeCameraCorrection(dataset);
        const auto accuracyEstimate = correction.accuracyEstimate();
        std::cout
            << "If written to the camera, this correction can be expected to yield a dimension accuracy error of "
            << std::fixed << std::setprecision(2) << 100.0F * accuracyEstimate.dimensionAccuracy()
            << "% or better in the range of z=[" << static_cast<int>(std::round(accuracyEstimate.zMin())) << ","
            << static_cast<int>(std::round(accuracyEstimate.zMax()))
            << "] across the full FOV. Accuracy close to where the correction data was collected is likely better."
            << std::endl;

        // Optionally save to camera
        if(yesNoPrompt("Save to camera?"))
        {
            std::cout << "Writing camera correction..." << std::endl;
            Zivid::Calibration::writeCameraCorrection(camera, correction);
            std::cout << "Success" << std::endl;
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
