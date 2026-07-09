/*
Capture a frame with diagnostics enabled and create a file camera from it.

A file camera is a virtual camera that replays captures offline using the raw sensor data
stored in the original frame. This allows you to develop and test without a physical camera.

The workflow:
1. Capture a frame with diagnostics enabled
2. Save the diagnostics frame as a .zdf file
3. Disconnect from the camera (no longer needed)
4. Load the .zdf frame and create a file camera from it
5. Adjust processing settings and capture from the file camera
6. Save the resulting frame

For more information about file cameras, check out this tutorial:
https://support.zivid.com/en/latest/camera/academy/camera/file-camera.html
*/

#include <Zivid/Zivid.h>

#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Creating default settings" << std::endl;
        Zivid::Settings settings{
            Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} },
            Zivid::Settings::Color{
                Zivid::Settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } } },
        };
        std::cout << "Enabling diagnostics" << std::endl;
        settings.set(Zivid::Settings::Diagnostics::Enabled::yes);

        std::cout << "Capturing frame with diagnostics" << std::endl;
        const auto frameWithDiagnostics = camera.capture2D3D(settings);

        const auto frameWithDiagnosticsFile = "FrameWithDiagnostics.zdf";
        std::cout << "Saving diagnostics frame to: " << frameWithDiagnosticsFile << std::endl;
        frameWithDiagnostics.save(frameWithDiagnosticsFile);

        std::cout << "Disconnecting from camera" << std::endl;
        camera.disconnect();

        std::cout << "Loading ZDF with diagnostics enabled from: " << frameWithDiagnosticsFile << std::endl;
        const auto loadedFrameWithDiagnostics = Zivid::Frame(frameWithDiagnosticsFile);

        std::cout << "Creating file camera from frame" << std::endl;
        auto fileCamera = zivid.createFileCamera(loadedFrameWithDiagnostics);

        std::cout << "File camera info: " << fileCamera.info() << std::endl;

        std::cout << "Configuring settings" << std::endl;
        auto settingsFromFrame = loadedFrameWithDiagnostics.settings();
        settingsFromFrame.set(Zivid::Settings::Diagnostics::Enabled::no);
        settingsFromFrame.set(Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes);
        settingsFromFrame.set(Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{ 1.5 });
        settingsFromFrame.set(Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes);
        settingsFromFrame.set(Zivid::Settings::Processing::Filters::Reflection::Removal::Mode::global);

        std::cout << "Capturing from file camera" << std::endl;
        const auto fileCameraFrame = fileCamera.capture2D3D(settingsFromFrame);

        const auto fileCameraFrameFile = "FrameFromFileCameraWithNoDiagnostics.zdf";
        std::cout << "Saving file camera frame to: " << fileCameraFrameFile << std::endl;
        fileCameraFrame.save(fileCameraFrameFile);
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
