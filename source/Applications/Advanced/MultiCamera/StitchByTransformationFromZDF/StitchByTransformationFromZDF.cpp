/*
Use transformation matrices from Multi-Camera calibration to transform point clouds into single coordinate frame, from a ZDF files.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.
*/

#include <Zivid/Experimental/PointCloudExport.h>
#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <Eigen/Core>
#include <clipp.h>

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

        std::cout << "Visualizing the stitched point cloud (" << finalPointCloud.size() << " data points)" << std::endl;
        visualizePointCloud(finalPointCloud);

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
