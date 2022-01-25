/*
Capture point clouds, with color, from the Zivid camera.
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

        std::cout << "Creating settings" << std::endl;
        const auto settings =
            Zivid::Settings{ Zivid::Settings::Experimental::Engine::phase,
                             Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{
                                 Zivid::Settings::Acquisition::Aperture{ 5.66 },
                                 Zivid::Settings::Acquisition::ExposureTime{ std::chrono::microseconds{ 6500 } } } },
                             Zivid::Settings::Processing::Filters::Outlier::Removal::Enabled::yes,
                             Zivid::Settings::Processing::Filters::Outlier::Removal::Threshold{ 5.0 } };

        std::cout << "Capturing frame" << std::endl;
        const auto frame = camera.capture(settings);

        const auto *dataFile = "Frame.zdf";
        std::cout << "Saving frame to file: " << dataFile << std::endl;
        frame.save(dataFile);

        const auto *dataFilePLY = "PointCloud.ply";
        std::cout << "Exporting point cloud to file: " << dataFilePLY << std::endl;
        frame.save(dataFilePLY);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }
}
