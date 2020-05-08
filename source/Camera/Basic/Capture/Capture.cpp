/*
This example shows how to capture point clouds, with color, from the Zivid camera.
*/

#include <Zivid/Zivid.h>

#include <chrono>
#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        const auto *resultFile = "Result.zdf";

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Creating settings" << std::endl;
        const auto settings =
            Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{
                                 Zivid::Settings::Acquisition::Aperture{ 5.66 },
                                 Zivid::Settings::Acquisition::ExposureTime{ std::chrono::microseconds{ 8333 } } } },
                             Zivid::Settings::Processing::Filters::Outlier::Removal::Enabled::yes,
                             Zivid::Settings::Processing::Filters::Outlier::Removal::Threshold{ 5.0 } };

        std::cout << "Capture a frame" << std::endl;
        const auto frame = camera.capture(settings);

        std::cout << "Saving frame to file: " << resultFile << std::endl;
        frame.save(resultFile);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
