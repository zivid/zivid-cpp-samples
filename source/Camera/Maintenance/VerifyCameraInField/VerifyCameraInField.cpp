/*
Check the dimension trueness of a Zivid camera.

This example shows how to verify the local dimension trueness of a camera.
If the trueness is much worse than expected, the camera may have been damaged by
shock in shipping or handling. If so, look at the CorrectCameraInField sample.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.
*/

#include <Zivid/Experimental/Calibration/InfieldCorrection.h>
#include <Zivid/Zivid.h>

#include <iomanip>
#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        // For convenience, print the timestamp of the latest correction
        if(Zivid::Experimental::Calibration::hasCameraCorrection(camera))
        {
            const auto timestamp = Zivid::Experimental::Calibration::cameraCorrectionTimestamp(camera);
            const auto time = std::chrono::system_clock::to_time_t(timestamp);
            std::cout << "Timestamp of current camera correction: " << std::put_time(std::gmtime(&time), "%FT%TZ")
                      << std::endl;
        }
        else
        {
            std::cout << "This camera has no infield correction written to it." << std::endl;
        }

        // Gather data
        std::cout << "Capturing calibration board" << std::endl;
        const auto detectionResult = Zivid::Experimental::Calibration::detectFeaturePoints(camera);

        // Prepare data and check that it is appropriate for infield verification
        const auto input = Zivid::Experimental::Calibration::InfieldCorrectionInput{ detectionResult };
        if(!input.valid())
        {
            throw std::runtime_error(
                "Capture not valid for infield verification! Feedback: " + input.statusDescription());
        }

        // Show results
        std::cout << "Successful measurement at " << detectionResult.centroid() << std::endl;
        const auto verification = Zivid::Experimental::Calibration::verifyCamera(input);
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