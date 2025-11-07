/*
Check the dimension trueness of a Zivid camera from a ZDF file.

This example shows how to verify the local dimension trueness of a camera from a ZDF file. If the trueness is much worse
than expected, the camera may have been damaged by shock in shipping or handling. If so, look at the CorrectCameraInField
sample.

Why is verifying camera accuracy from a ZDF file useful?

Let us assume that your system is in production. You want to verify the accuracy of the camera while the system is running.
At the same time, you want to minimize the time the robot and the camera are used for anything else than their main task,
e.g., bin picking. Instead of running a full infield verification live, which consists of capturing, detecting, and
estimating accuracy, you can instead only capture and save results to ZDF files on disk. As the robot and the camera go
back to their main tasks, you can load the ZDF files and verify the accuracy offline, using a different PC than the one
used in production. In addition, you can send these ZDF files to Zivid Customer Success for investigation.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.
*/

#include <Zivid/Calibration/Detector.h>
#include <Zivid/Calibration/InfieldCorrection.h>
#include <Zivid/Zivid.h>

#include <iomanip>
#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        const auto fileCamera = std::string(ZIVID_SAMPLE_DATA_DIR) + "/BinWithCalibrationBoard.zfc";
        std::cout << "Creating virtual camera using file: " << fileCamera << std::endl;
        auto camera = zivid.createFileCamera(fileCamera);

        // Calibration board can be captured live, while the system is in production, and saved to ZDF file, for later use in
        // offline infield verification

        std::cout << "Capturing calibration board" << std::endl;
        const auto frame = Zivid::Calibration::captureCalibrationBoard(camera);

        const auto dataFile = "FrameWithCalibrationBoard.zdf";
        std::cout << "Saving frame to file: " << dataFile << ", for later use in offline infield verification"
                  << std::endl;
        frame.save(dataFile);

        // The ZDF captured with captureCalibrationBoard(camera) that contains the calibration board can be loaded for
        // offline infield verification

        std::cout << "Reading frame from file: " << dataFile << ", for offline infield verification" << std::endl;
        const auto loadedFrame = Zivid::Frame(dataFile);

        std::cout << "Detecting calibration board" << std::endl;
        const auto detectionResult = Zivid::Calibration::detectCalibrationBoard(loadedFrame);

        const auto input = Zivid::Calibration::InfieldCorrectionInput{ detectionResult };
        if(!input.valid())
        {
            throw std::runtime_error(
                "Capture not valid for infield verification! Feedback: " + input.statusDescription());
        }

        std::cout << "Successful measurement at " << detectionResult.centroid() << std::endl;
        const auto verification = Zivid::Calibration::verifyCamera(input);
        std::cout << "Estimated dimension trueness error at measured position: " << std::setprecision(2) << std::fixed
                  << verification.localDimensionTrueness() * 100.0F << "%" << std::endl;
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
