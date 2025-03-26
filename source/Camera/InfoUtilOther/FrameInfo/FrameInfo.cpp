/*
Read frame info from the Zivid camera.

The frame info consists of the version information for installed software at the time of capture,
information about the system that captured the frame, and the time stamp of the capture.

*/

#include <Zivid/Zivid.h>

#include <chrono>
#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Capturing frame" << std::endl;
        const auto settings =
            Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} },
                             Zivid::Settings::Color{ Zivid::Settings2D{
                                 Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } } } };

        const auto frame = camera.capture2D3D(settings);

        const auto frameInfo = frame.info();

        std::cout << "The version information for installed software at the time of image capture:" << std::endl;
        std::cout << frameInfo.softwareVersion() << std::endl;

        std::cout << "Information about the system that captured this frame:" << std::endl;
        std::cout << frameInfo.systemInfo() << std::endl;

        std::cout << "The time of frame capture:" << std::endl;
        std::cout << frameInfo.timeStamp() << std::endl;

        std::cout << "Acquisition time:" << std::endl;
        std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(
                         frameInfo.metrics().acquisitionTime().value())
                         .count()
                  << " ms" << std::endl;

        std::cout << "Capture time:" << std::endl;
        std::cout
            << std::chrono::duration_cast<std::chrono::milliseconds>(frameInfo.metrics().captureTime().value()).count()
            << " ms" << std::endl;
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
