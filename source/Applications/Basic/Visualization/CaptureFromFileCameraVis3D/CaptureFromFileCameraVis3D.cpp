/*
Capture point clouds, with color, with the Zivid file camera and visualize them.
This sample can be used without access to a physical camera.

The file camera is created from a ZDF with diagnostics.
ZDF files with diagnostics are found in Zivid Sample Data.
See the instructions in README.md to download the Zivid Sample Data.
There are nine available file cameras to choose from, one for each camera model.
The default ZDF used in this sample is from Zivid 2 M70.

For more information about file cameras, check out this tutorial:
https://support.zivid.com/en/latest/camera/academy/camera/file-camera.html
*/

#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <clipp.h>
#include <iostream>

int main(int argc, char **argv)
{
    try
    {
        bool userInput = false;

        std::string fileCameraPath;
        auto cli =
            (clipp::option("--file-camera").set(userInput, true)
             & clipp::value("<Path to a ZDF with diagnostics enabled>", fileCameraPath));

        if(!parse(argc, argv, cli))
        {
            auto fmt = clipp::doc_formatting{}.alternatives_min_split_size(1).surround_labels("\"", "\"");
            std::cout << clipp::usage_lines(cli, "Usage: ", fmt) << std::endl;
            throw std::runtime_error{ "Invalid usage" };
        }

        Zivid::Application zivid;

        const auto fileCamera =
            userInput ? fileCameraPath : std::string(ZIVID_SAMPLE_DATA_DIR) + "/FileCameraZivid2M70.zdf";
        const auto loadedFrameWithDiagnostics = Zivid::Frame(fileCamera);

        std::cout << "Creating virtual camera using file: " << fileCamera << std::endl;
        auto camera = zivid.createFileCamera(loadedFrameWithDiagnostics);

        std::cout << "Capturing frame" << std::endl;
        auto settings = loadedFrameWithDiagnostics.settings();
        settings.set(Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes);
        settings.set(Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{ 1.5 });
        settings.set(Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes);
        settings.set(Zivid::Settings::Processing::Filters::Reflection::Removal::Mode::global);
        settings.set(
            Zivid::Settings::RegionOfInterest::Box{
                Zivid::Settings::RegionOfInterest::Box::Enabled::yes,
                Zivid::Settings::RegionOfInterest::Box::PointO{ { -331, 201, 661 } },
                Zivid::Settings::RegionOfInterest::Box::PointA{ { 299, 203, 667 } },
                Zivid::Settings::RegionOfInterest::Box::PointB{ { -331, -203, 844 } },
                Zivid::Settings::RegionOfInterest::Box::Extents{ 0, 178 } });
        const auto frame = camera.capture2D3D(settings);

        std::cout << "Setting up visualization" << std::endl;
        Zivid::Visualization::Visualizer visualizer;

        std::cout << "Visualizing point cloud" << std::endl;
        visualizer.showMaximized();
        visualizer.show(frame);
        visualizer.resetToFit();

        std::cout << "Running visualizer. Blocking until window closes." << std::endl;
        visualizer.run();
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
