/*
Use transformation matrices from Multi-Camera calibration to transform point clouds into single coordinate frame, from a ZDF files.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.
*/

#include <Zivid/Experimental/PointCloudExport.h>
#include <Zivid/Zivid.h>

#include <Eigen/Core>
#include <clipp.h>
#include <open3d/Open3D.h>

#include <cmath>
#include <iostream>
#include <vector>

namespace
{
    Zivid::UnorganizedPointCloud getTransformedPointClouds(
        const std::vector<std::string> &zdfFileList,
        const std::vector<std::string> &transformationMatrixFilesList)
    {
        std::string fileExtension;
        std::string serialNumber;

        Zivid::UnorganizedPointCloud stitchedPointCloud;
        auto numberOfPointClouds = 0;

        for(const auto &zdfFileName : zdfFileList)
        {
            const auto frame = Zivid::Frame(zdfFileName);
            serialNumber = frame.cameraInfo().serialNumber().toString();
            std::cout << "Searching in " << zdfFileName << std::endl;

            for(const auto &yamlFileName : transformationMatrixFilesList)
            {
                if(serialNumber
                   == yamlFileName.substr(
                       yamlFileName.find_last_of("\\/") + 1,
                       (yamlFileName.find_last_of('.')) - (yamlFileName.find_last_of("\\/") + 1)))
                {
                    Zivid::Matrix4x4 transformationMatrixZivid(yamlFileName);
                    Zivid::UnorganizedPointCloud currentPointCloud = frame.pointCloud().toUnorganizedPointCloud();

                    stitchedPointCloud.extend(currentPointCloud.transform(transformationMatrixZivid));
                    numberOfPointClouds++;
                    break;
                }
                if(transformationMatrixFilesList.back() == yamlFileName)
                {
                    throw std::runtime_error("You are missing a YAML file named " + serialNumber + ".yaml!");
                }
            }
        }
        if(numberOfPointClouds < 2)
        {
            throw std::runtime_error(
                "Require minimum two matching transformation and frames, got " + std::to_string(numberOfPointClouds));
        }

        return stitchedPointCloud;
    }

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
        Zivid::Application zivid;

        std::string stitchedPointCloudFileName;
        auto zdfFileList = std::vector<std::string>{};
        auto transformationMatrixFilesList = std::vector<std::string>{};
        auto saveStitched = false;
        auto cli =
            (clipp::required("-zdf") & clipp::values("ZDF filenames", zdfFileList) % "List of ZDF files to stitch.",

             clipp::required("-yaml")
                 & clipp::values("YAML filenames", transformationMatrixFilesList)
                       % "List of YAML files containing the corresponding transformation matrices.",

             clipp::required("-o", "--output-file").set(saveStitched)
                 & clipp::value("Output point cloud (PLY) file name", stitchedPointCloudFileName)
                       % "Save the stitched point cloud to a file with this name. (.ply)");

        if(!parse(argc, argv, cli))
        {
            auto fmt = clipp::doc_formatting{}.alternatives_min_split_size(1).surround_labels("\"", "\"");
            std::cout << "SYNOPSIS:" << std::endl;
            std::cout << clipp::usage_lines(cli, "StitchByTransformationFromFile", fmt) << std::endl;
            std::cout << "OPTIONS:" << std::endl;
            std::cout << clipp::documentation(cli) << std::endl;
            throw std::runtime_error("No file provided.");
        }

        const auto stitchedPointCloud = getTransformedPointClouds(zdfFileList, transformationMatrixFilesList);

        std::cout << "Voxel-downsampling the stitched point cloud" << std::endl;
        const auto finalPointCloud = stitchedPointCloud.voxelDownsampled(0.5, 1);

        std::cout << "Copying the stitched point cloud to Open3D" << std::endl;
        const auto pointCloudOpen3D = copyToOpen3D(finalPointCloud);

        std::cout << "Visualizing the stitched point cloud (" << pointCloudOpen3D.GetPointPositions().GetLength()
                  << " data points)" << std::endl;
        visualizePointCloud(pointCloudOpen3D);

        if(saveStitched)
        {
            const std::string &fileName = stitchedPointCloudFileName;
            std::cout << "Saving " << finalPointCloud.size() << " data points to " << fileName << std::endl;

            using PLY = Zivid::Experimental::PointCloudExport::FileFormat::PLY;
            const auto colorSpace = Zivid::Experimental::PointCloudExport::ColorSpace::sRGB;
            Zivid::Experimental::PointCloudExport::exportUnorganizedPointCloud(
                finalPointCloud, PLY{ fileName, PLY::Layout::unordered, colorSpace });
        }
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
