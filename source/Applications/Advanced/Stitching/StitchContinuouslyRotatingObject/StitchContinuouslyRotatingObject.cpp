/*
Stitch point clouds from a continuously rotating object without pre-alignment using Local Point Cloud Registration and apply Voxel Downsample.

It is assumed that the object is rotating around its own axis and the camera is stationary.
The camera settings should have defined a region of interest box that removes unnecessary points, keeping only the object to be stitched.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.

*/

#include <Zivid/Experimental/LocalPointCloudRegistrationParameters.h>
#include <Zivid/Experimental/PointCloudExport.h>
#include <Zivid/Experimental/Toolbox/PointCloudRegistration.h>
#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <clipp.h>

#include <Eigen/Core>

#include <filesystem>
#include <iostream>
#include <string>
#include <thread>

namespace
{
    void visualizePointCloud(const Zivid::UnorganizedPointCloud &unorganizedPointCloud)
    {
        Zivid::Visualization::Visualizer visualizer;

        visualizer.showMaximized();
        visualizer.show(unorganizedPointCloud);
        visualizer.resetToFit();

        std::cout << "Running visualizer. Blocking until window closes." << std::endl;
        visualizer.run();
    }
} // namespace

int main(int argc, char **argv)
{
    try
    {
        std::string settingsPath;
        bool showHelp = false;

        auto cli =
            (clipp::option("-h", "--help").set(showHelp) % "Show help message",
             clipp::option("--settings-path")
                 & clipp::value("path", settingsPath) % "Path to the camera settings YML file");

        if(!clipp::parse(argc, argv, cli) || showHelp)
        {
            auto fmt = clipp::doc_formatting{}.alternatives_min_split_size(1).surround_labels("\"", "\"");
            std::cout << "USAGE:" << std::endl;
            std::cout << clipp::usage_lines(cli, argv[0], fmt) << std::endl;
            std::cout << "OPTIONS:" << std::endl;
            std::cout << clipp::documentation(cli) << std::endl;
            return showHelp ? EXIT_SUCCESS : EXIT_FAILURE;
        }

        Zivid::Application app;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = app.connectCamera();

        std::cout << "Loading settings from file: " << settingsPath << std::endl;
        const auto settings = Zivid::Settings(settingsPath);

        Zivid::UnorganizedPointCloud unorganizedStitchedPointCloud;
        auto registrationParams = Zivid::Experimental::LocalPointCloudRegistrationParameters{};
        auto previousToCurrentPointCloudTransform = Zivid::Matrix4x4::identity();

        for(int numberOfCaptures = 0; numberOfCaptures < 20; ++numberOfCaptures)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            auto frame = camera.capture2D3D(settings);
            const auto unorganizedPointCloud = frame.pointCloud().toUnorganizedPointCloud().voxelDownsampled(1.0, 2);

            if(numberOfCaptures != 0)
            {
                if(unorganizedStitchedPointCloud.size() < 4 || unorganizedPointCloud.size() < 4)
                {
                    std::cout << "Not enough points for registration, skipping stitching..." << std::endl;
                    continue;
                }

                const auto registrationResult = Zivid::Experimental::Toolbox::localPointCloudRegistration(
                    unorganizedStitchedPointCloud,
                    unorganizedPointCloud,
                    registrationParams,
                    previousToCurrentPointCloudTransform);

                if(!registrationResult.converged())
                {
                    std::cout << "Registration did not converge..." << std::endl;
                    continue;
                }

                previousToCurrentPointCloudTransform = registrationResult.transform().toMatrix();
                unorganizedStitchedPointCloud.transform(previousToCurrentPointCloudTransform.inverse());
            }

            unorganizedStitchedPointCloud.extend(unorganizedPointCloud);
            std::cout << "Captures done: " << numberOfCaptures << std::endl;
        }

        std::cout << "Voxel-downsampling the stitched point cloud" << std::endl;
        unorganizedStitchedPointCloud = unorganizedStitchedPointCloud.voxelDownsampled(0.75, 2);

        visualizePointCloud(unorganizedStitchedPointCloud);

        const auto fileName = "StitchedPointCloudOfRotatingObject.ply";
        Zivid::Experimental::PointCloudExport::FileFormat::PLY plyFile{
            fileName,
            Zivid::Experimental::PointCloudExport::FileFormat::PLY::Layout::unordered,
            Zivid::Experimental::PointCloudExport::ColorSpace::sRGB
        };
        std::cout << "Exporting point cloud to file: " << plyFile.fileName() << std::endl;
        Zivid::Experimental::PointCloudExport::exportUnorganizedPointCloud(unorganizedStitchedPointCloud, plyFile);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
