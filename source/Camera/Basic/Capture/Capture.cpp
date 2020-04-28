#include <Zivid/Zivid.h>

#include <chrono>
#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        const auto *resultFile = "result.zdf";

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Adjusting the camera settings" << std::endl;
        camera << Zivid::Settings::Iris{ 20 } << Zivid::Settings::ExposureTime{ std::chrono::microseconds{ 8333 } }
               << Zivid::Settings::Filters::Outlier::Enabled::yes << Zivid::Settings::Filters::Outlier::Threshold{ 5 };

        std::cout << "Capture a frame" << std::endl;
        auto frame = camera.capture();

        std::cout << "Saving frame to file: " << resultFile << std::endl;
        frame.save(resultFile);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
