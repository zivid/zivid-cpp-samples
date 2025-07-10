/*
Stitch point clouds from a continuously rotating object without pre-alignment using Local Point Cloud Registration and apply Voxel Downsample.

It is assumed that the object is rotating around its own axis and the camera is stationary.
The camera settings should have defined a region of interest box that removes unnecessary points, keeping only the object to be stitched.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.

*/

#include <Zivid/Experimental/LocalPointCloudRegistrationParameters.h>
#include <Zivid/Experimental/PointCloudExport.h>
#include <Zivid/Experimental/Toolbox/PointCloudRegistration.h>
#include <Zivid/Zivid.h>

#include <clipp.h>

#include <Eigen/Core>
#include <open3d/Open3D.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <thread>

namespace
{
    open3d::t::geometry::PointCloud copyToOpen3D(const Zivid::UnorganizedPointCloud &pointCloud)
    {
        auto device = open3d::core::Device("CPU:0");
        auto xyzTensor =
            open3d::core::Tensor({ static_cast<int64_t>(pointCloud.size()), 3 }, open3d::core::Dtype::Float32, device);
        auto rgbTensor =
            open3d::core::Tensor({ static_cast<int64_t>(pointCloud.size()), 3 }, open3d::core::Dtype::Float32, device);

        pointCloud.copyData(reinterpret_cast<Zivid::PointXYZ *>(xyzTensor.GetDataPtr<float>()));

        // Open3D does not store colors in 8-bit
        auto *rgbPtr = rgbTensor.GetDataPtr<float>();
        auto rgbaColors = pointCloud.copyColorsRGBA_SRGB();
        for(size_t i = 0; i < pointCloud.size(); ++i)
        {
            rgbPtr[3 * i] = static_cast<float>(rgbaColors(i).r) / 255.0f;
            rgbPtr[3 * i + 1] = static_cast<float>(rgbaColors(i).g) / 255.0f;
            rgbPtr[3 * i + 2] = static_cast<float>(rgbaColors(i).b) / 255.0f;
        }

        open3d::t::geometry::PointCloud cloud(device);
        cloud.SetPointPositions(xyzTensor);
        cloud.SetPointColors(rgbTensor);
        return cloud;
    }

    void visualizePointCloud(const open3d::t::geometry::PointCloud &cloud)
    {
        open3d::visualization::Visualizer visualizer;
        visualizer.CreateVisualizerWindow("Open3D Viewer", 1024, 768);

        visualizer.AddGeometry(std::make_shared<open3d::geometry::PointCloud>(cloud.ToLegacy()));

        auto &renderOption = visualizer.GetRenderOption();
        renderOption.background_color_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        renderOption.point_size_ = 1.0;
        renderOption.show_coordinate_frame_ = true;

        auto &viewControl = visualizer.GetViewControl();
        viewControl.SetFront(Eigen::Vector3d(0.0, 0.0, -1.0));
        viewControl.SetUp(Eigen::Vector3d(0.0, -1.0, 0.0));

        std::cout << "Press h to access the help menu" << std::endl;
        std::cout << "Press q to exit the viewer application" << std::endl;
        visualizer.Run(); // Block until window closed
        visualizer.DestroyVisualizerWindow();
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

        const auto unorganizedStitchedPointCloudOpen3D = copyToOpen3D(unorganizedStitchedPointCloud);
        visualizePointCloud(unorganizedStitchedPointCloudOpen3D);

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
