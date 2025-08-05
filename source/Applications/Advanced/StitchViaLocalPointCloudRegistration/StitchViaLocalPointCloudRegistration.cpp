/*
Stitch two point clouds using a transformation estimated by Local Point Cloud Registration and apply Voxel Downsample.

Dataset: https://support.zivid.com/en/latest/api-reference/samples/sample-data.html

Extract the content into:
    - Windows:   %ProgramData%/Zivid/StitchingPointClouds/
    - Linux:     /usr/share/Zivid/data/StitchingPointClouds/

StitchingPointClouds/
    └── BlueObject/

The folder must contain two ZDF files used for this sample.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.

*/

#include <Zivid/Experimental/LocalPointCloudRegistrationParameters.h>
#include <Zivid/Experimental/Toolbox/PointCloudRegistration.h>
#include <Zivid/Zivid.h>

#include <Eigen/Core>
#include <open3d/Open3D.h>

#include <cmath>
#include <iostream>
#include <vector>

namespace
{
    open3d::t::geometry::PointCloud copyToOpen3D(const Zivid::UnorganizedPointCloud &pointCloud)
    {
        using namespace open3d::core;
        auto device = Device("CPU:0");
        auto xyzTensor = Tensor({ static_cast<int64_t>(pointCloud.size()), 3 }, Dtype::Float32, device);
        auto rgbTensor = Tensor({ static_cast<int64_t>(pointCloud.size()), 3 }, Dtype::Float32, device);

        pointCloud.copyData(reinterpret_cast<Zivid::PointXYZ *>(xyzTensor.GetDataPtr<float>()));

        // Open3D does not store colors in 8-bit
        const auto rgbaColors = pointCloud.copyColorsRGBA_SRGB();
        for(size_t i = 0; i < pointCloud.size(); ++i)
        {
            const auto r = static_cast<float>(rgbaColors(i).r) / 255.0f;
            const auto g = static_cast<float>(rgbaColors(i).g) / 255.0f;
            const auto b = static_cast<float>(rgbaColors(i).b) / 255.0f;
            rgbTensor.SetItem(TensorKey::Index(i), Tensor::Init({ r, g, b }));
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
        renderOption.point_size_ = 2.0;
        renderOption.show_coordinate_frame_ = true;

        auto &viewControl = visualizer.GetViewControl();
        viewControl.SetFront(Eigen::Vector3d(0.0, 0.0, -1.0));
        viewControl.SetUp(Eigen::Vector3d(0.0, -1.0, 0.0));

        std::cout << "Press h to access the help menu" << std::endl;
        std::cout << "Press q to exit the viewer application" << std::endl;
        visualizer.Run();
        visualizer.DestroyVisualizerWindow();
    }

} // namespace

int main()
{
    try
    {
        Zivid::Application app;

        // Ensure the dataset is extracted to the correct location depending on the operating system:
        //   - Windows:   %ProgramData%/Zivid/StitchingPointClouds/
        //   - Linux:     /usr/share/Zivid/data/StitchingPointClouds/
        //  StitchingPointClouds/
        //      └── BlueObject/
        std::cout << "Reading point clouds from ZDF files" << std::endl;
        const auto directory = std::filesystem::path(ZIVID_SAMPLE_DATA_DIR) / "StitchingPointClouds" / "BlueObject";

        if(!std::filesystem::exists(directory))
        {
            std::ostringstream oss;
            oss << "Missing dataset folders.\n"
                << "Make sure 'StitchingPointClouds/BlueObject/' exist at " << ZIVID_SAMPLE_DATA_DIR << ".\n\n"
                << "You can download the dataset (StitchingPointClouds.zip) from:\n"
                << "https://support.zivid.com/en/latest/api-reference/samples/sample-data.html";

            throw std::runtime_error(oss.str());
        }

        const auto frame1 = Zivid::Frame((directory / "BlueObject.zdf").string());
        const auto frame2 = Zivid::Frame((directory / "BlueObjectSlightlyMoved.zdf").string());

        std::cout << "Converting organized point clouds to unorganized point clouds and voxel downsampling"
                  << std::endl;
        const auto unorganizedPointCloud1 = frame1.pointCloud().toUnorganizedPointCloud();
        const auto unorganizedPointCloud2 = frame2.pointCloud().toUnorganizedPointCloud();

        std::cout << "Displaying point clouds before stitching" << std::endl;
        Zivid::UnorganizedPointCloud unorganizedNotStitchedPointCloud;
        unorganizedNotStitchedPointCloud.extend(unorganizedPointCloud1);
        unorganizedNotStitchedPointCloud.extend(unorganizedPointCloud2);
        const auto unorganizedNotStitchedPointCloudOpen3D = copyToOpen3D(unorganizedNotStitchedPointCloud);
        visualizePointCloud(unorganizedNotStitchedPointCloudOpen3D);

        std::cout << "Estimating transformation between point clouds" << std::endl;
        const auto unorganizedPointCloud1LPCR = unorganizedPointCloud1.voxelDownsampled(1.0, 3);
        const auto unorganizedPointCloud2LPCR = unorganizedPointCloud2.voxelDownsampled(1.0, 3);
        const auto registrationParams = Zivid::Experimental::LocalPointCloudRegistrationParameters{};
        const auto localPointCloudRegistrationResult = Zivid::Experimental::Toolbox::localPointCloudRegistration(
            unorganizedPointCloud1LPCR, unorganizedPointCloud2LPCR, registrationParams);

        if(!localPointCloudRegistrationResult.converged())
        {
            throw std::runtime_error("Registration did not converge...");
        }

        const auto pointCloud1ToPointCloud2Transform = localPointCloudRegistrationResult.transform();

        std::cout << "Stitching and displaying point clouds" << std::endl;
        Zivid::UnorganizedPointCloud finalPointCloud;
        finalPointCloud.extend(unorganizedPointCloud1);
        const auto unorganizedPointCloud2Transformed =
            unorganizedPointCloud2.transformed(pointCloud1ToPointCloud2Transform.toMatrix());
        finalPointCloud.extend(unorganizedPointCloud2Transformed);
        const auto stitchedPointCloudOpen3D = copyToOpen3D(finalPointCloud);
        visualizePointCloud(stitchedPointCloudOpen3D);

        std::cout << "Voxel-downsampling the stitched point cloud" << std::endl;
        finalPointCloud = finalPointCloud.voxelDownsampled(2.0, 1);
        std::cout << "visualize the overllaped point clouds" << std::endl;
        const auto stitchedDownampledPointCloudOpen3D = copyToOpen3D(finalPointCloud);
        visualizePointCloud(stitchedDownampledPointCloudOpen3D);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
