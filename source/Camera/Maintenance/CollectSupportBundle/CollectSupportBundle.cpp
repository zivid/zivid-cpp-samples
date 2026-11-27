/*
Collect a support bundle to send to Zivid customer support.

A support bundle is a single compressed archive (.tar.zst) with what customer support needs to investigate an issue:
the SDK log files, frames you attach, and the information, state and health of every camera the application knows about.
This sample connects to a camera, captures a 3D frame and a 2D frame, and saves the 3D frame to a ZDF file. Then it
creates a bundle with the two frames, the ZDF file and the logs from the last hour in the current folder, and prints the
progress while the SDK creates the bundle in the background.

Creating a bundle never connects to a camera, so connect the cameras you want health information for before creating it.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.

For more information about support bundles, check out this tutorial:
https://support.zivid.com/en/latest/camera/academy/camera/support-bundle.html
*/

#include <Zivid/Experimental/SupportBundle.h>
#include <Zivid/Zivid.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace
{
    std::string toUtcString(const std::chrono::system_clock::time_point &timePoint)
    {
        const auto time = std::chrono::system_clock::to_time_t(timePoint);
        std::stringstream stream;
        stream << std::put_time(std::gmtime(&time), "%FT%TZ");
        return stream.str();
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Capturing 3D frame" << std::endl;
        const auto frame =
            camera.capture3D(Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} } });

        std::cout << "Capturing 2D frame" << std::endl;
        const auto frame2D =
            camera.capture2D(Zivid::Settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } });

        const auto zdfFile = std::filesystem::current_path() / "SupportBundleFrame.zdf";
        std::cout << "Saving 3D frame to file: " << zdfFile << std::endl;
        frame.save(zdfFile.string());

        const auto before = std::chrono::system_clock::now();
        const auto since = before - std::chrono::hours{ 1 };
        const auto bundleBuilder = Zivid::Experimental::SupportBundle::builder(std::filesystem::current_path())
                                       .attachedFrames({ frame })
                                       .attachedFrame2Ds({ frame2D })
                                       .attachedZDFFiles({ zdfFile })
                                       .filterLogs(since, before);

        std::cout << "Log files in the time window:" << std::endl;
        for(const auto &logFile : Zivid::Experimental::SupportBundle::logFiles())
        {
            if(logFile.modified >= since && logFile.created <= before)
            {
                std::cout << "  " << toUtcString(logFile.created) << " to " << toUtcString(logFile.modified) << ", "
                          << logFile.sizeBytes << " bytes" << std::endl;
            }
        }

        const auto estimate = bundleBuilder.estimateSize();
        std::cout << "Estimated size before compression: " << estimate.logBytes << " bytes of logs and "
                  << estimate.frameBytes << " bytes of frames" << std::endl;

        std::cout << "Creating support bundle" << std::endl;
        const auto bundle = bundleBuilder.create(zivid);
        for(const auto &update : bundle.progress())
        {
            std::cout << std::fixed << std::setprecision(0) << std::setw(4) << update.percentage << "% "
                      << update.description << std::endl;
        }
        const auto bundlePath = bundle.get();

        std::cout << "Support bundle saved to " << bundlePath << std::endl;
        std::cout << "Send this file to customersuccess@zivid.com" << std::endl;
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
