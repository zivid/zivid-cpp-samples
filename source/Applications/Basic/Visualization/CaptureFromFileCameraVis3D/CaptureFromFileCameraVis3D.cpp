/*
Capture point clouds, with color, from the virtual Zivid camera, and visualize it. Currently supported by Zivid One.

This example can be used without access to a physical camera.
The ZFC file for this sample can be found under the main instructions for Zivid samples.
*/

#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        // The fileCamera file is in Zivid Sample Data. See instructions in README.md
        const auto fileCamera = std::string(ZIVID_SAMPLE_DATA_DIR) + "/FileCameraZividOne.zfc";

        std::cout << "Creating virtual camera using file: " << fileCamera << std::endl;
        auto camera = zivid.createFileCamera(fileCamera);

        std::cout << "Configuring settings" << std::endl;
        const auto settings = Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} },
                                               Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
                                               Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{ 1.5 },
                                               Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes,
                                               Zivid::Settings::Processing::Color::Balance::Red{ 1 },
                                               Zivid::Settings::Processing::Color::Balance::Green{ 1 },
                                               Zivid::Settings::Processing::Color::Balance::Blue{ 1 } };

        std::cout << "Capturing frame" << std::endl;
        const auto frame = camera.capture(settings);

        std::cout << "Setting up visualization" << std::endl;
        Zivid::Visualization::Visualizer visualizer;

        std::cout << "Visualizing point cloud" << std::endl;
        visualizer.showMaximized();
        visualizer.show(frame);
        visualizer.resetToFit();

        std::cout << "Running visualizer. Blocking until window closes" << std::endl;
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
