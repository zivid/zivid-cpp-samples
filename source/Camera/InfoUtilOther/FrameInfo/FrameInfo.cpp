/*
Read frame info from the Zivid camera.

The frame info consists of the version information for installed software at the time of capture,
information about the system that captured the frame, and the time stamp of the capture.

*/

#include <Zivid/Zivid.h>

#include <iostream>

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
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Capturing frame" << std::endl;
        const auto frame = assistedCapture(camera);

        const auto frameInfo = frame.info();

        std::cout << "The version information for installed software at the time of image capture:" << std::endl;
        std::cout << frameInfo.softwareVersion() << std::endl;

        std::cout << "Information about the system that captured this frame:" << std::endl;
        std::cout << frameInfo.systemInfo() << std::endl;

        std::cout << "The time of frame capture:" << std::endl;
        std::cout << frameInfo.timeStamp() << std::endl;
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
