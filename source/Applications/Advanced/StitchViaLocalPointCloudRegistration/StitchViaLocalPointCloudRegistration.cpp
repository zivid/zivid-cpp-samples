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
#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <Eigen/Core>

#include <cmath>
#include <filesystem>
#include <iostream>

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
        visualizePointCloud(unorganizedNotStitchedPointCloud);

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
        const auto unorganizedPointCloud2Transformed =
            unorganizedPointCloud2.transformed(pointCloud1ToPointCloud2Transform.toMatrix());

        std::cout << "Stitching and displaying painted point clouds to evaluate stitching quality" << std::endl;
        Zivid::UnorganizedPointCloud finalPointCloud;
        finalPointCloud.extend(unorganizedPointCloud1);
        finalPointCloud.extend(unorganizedPointCloud2Transformed);

        Zivid::UnorganizedPointCloud paintedFinalPointCloud;
        paintedFinalPointCloud.extend(unorganizedPointCloud1.paintedUniformColor(Zivid::ColorRGBA{ 255, 0, 0, 255 }));
        paintedFinalPointCloud.extend(
            unorganizedPointCloud2Transformed.paintedUniformColor(Zivid::ColorRGBA{ 0, 255, 0, 255 }));
        visualizePointCloud(paintedFinalPointCloud);

        std::cout << "Voxel-downsampling the stitched point cloud" << std::endl;
        finalPointCloud = finalPointCloud.voxelDownsampled(2.0, 1);
        std::cout << "Visualize the overlapped point clouds" << std::endl;
        visualizePointCloud(finalPointCloud);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
