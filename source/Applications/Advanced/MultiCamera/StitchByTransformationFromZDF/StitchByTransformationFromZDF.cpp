/*
Use transformation matrices from Multi-Camera calibration to transform point clouds into single coordinate frame, from a ZDF files.
*/

#include <clipp.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Zivid/Zivid.h>

#include <cmath>
#include <iostream>
#include <vector>

namespace
{
    class TransformationMatrixAndFrameMap
    {
    public:
        TransformationMatrixAndFrameMap(Zivid::Matrix4x4 transformationMatrix, Zivid::Frame frame)
            : mTransformationMatrix(transformationMatrix)
            , mFrame(std::move(frame))
        {}

        const Zivid::Matrix4x4 mTransformationMatrix;
        const Zivid::Frame mFrame;
    };

    std::vector<TransformationMatrixAndFrameMap> getTransformationMatricesAndFramesFromZDF(
        const std::vector<std::string> &transformationMatricesfileList)
    {
        std::string fileExtension;
        std::string serialNumber;

        auto transformsMappedToFrames = std::vector<TransformationMatrixAndFrameMap>{};
        for(const auto &zdfFileName : transformationMatricesfileList)
        {
            fileExtension = zdfFileName.substr(zdfFileName.find_last_of('.') + 1);
            if(fileExtension == "zdf")
            {
                const auto frame = Zivid::Frame(zdfFileName);
                serialNumber = frame.cameraInfo().serialNumber().toString();
                for(const auto &yamlFileName : transformationMatricesfileList)
                {
                    std::cout << "Searching in " << yamlFileName << std::endl;
                    if(serialNumber
                       == yamlFileName.substr(
                           yamlFileName.find_last_of("\\/") + 1,
                           (yamlFileName.find_last_of('.')) - (yamlFileName.find_last_of("\\/") + 1)))
                    {
                        Zivid::Matrix4x4 transformationMatrixZivid(yamlFileName);
                        transformsMappedToFrames.emplace_back(transformationMatrixZivid, frame);
                        break;
                    }
                    if(transformationMatricesfileList.back() == yamlFileName)
                    {
                        throw std::runtime_error("You are missing a YAML file named " + serialNumber + ".yaml!");
                    }
                }
            }
        }
        if(transformsMappedToFrames.size() < 2)
        {
            throw std::runtime_error(
                "Require minimum two matching transformation and frames, got "
                + std::to_string(transformsMappedToFrames.size()));
        }

        return transformsMappedToFrames;
    }

    const auto rgbList = std::array<std::uint32_t, 16>{
        0xFFB300, // Vivid Yellow
        0x803E75, // Strong Purple
        0xFF6800, // Vivid Orange
        0xA6BDD7, // Very Light Blue
        0xC10020, // Vivid Red
        0x007D34, // Vivid Green
        0x00538A, // Strong Blue
        0x232C16, // Dark Olive Green
        0x53377A, // Strong Violet
        0xFF8E00, // Vivid Orange Yellow
        0xB32851, // Strong Purplish Red
        0xF4C800, // Vivid Greenish Yellow
        0x93AA00, // Vivid Yellowish Green
        0x593315, // Deep Yellowish Brown
        0xF13A13, // Vivid Reddish Orange
        0x7F180D, // Strong Reddish Brown
    };
} // namespace

int main(int argc, char **argv)
{
    try
    {
        Zivid::Application zivid;

        std::string stitchedPointCloudFileName;
        auto transformationMatricesAndZdfFileList = std::vector<std::string>{};
        auto useRGB = true;
        auto saveStitched = false;
        auto cli =
            (clipp::values("File Names", transformationMatricesAndZdfFileList)
                 % "List of ZDF files to stitch and list of YAML files containing the transformation matrix.",
             clipp::option("-m", "--mono-chrome").set(useRGB, false) % "Color each point cloud with unique color.",
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

        const auto transformsMappedToFrames =
            getTransformationMatricesAndFramesFromZDF(transformationMatricesAndZdfFileList);

        // Loop through frames to find final size
        auto maxNumberOfPoints = 0;
        for(const auto &frameMap : transformsMappedToFrames)
        {
            maxNumberOfPoints += frameMap.mFrame.pointCloud().width() * frameMap.mFrame.pointCloud().height();
        }

        // Stitch frames
        // Creating a PointCloud structure
        pcl::PointCloud<pcl::PointXYZRGB> stitchedPointCloud;

        // Filling in the cloud data, unorganized skipping NaNs
        stitchedPointCloud.points.resize(maxNumberOfPoints);

        size_t validPoints = 0;
        for(size_t i = 0; i < transformsMappedToFrames.size(); i++)
        {
            auto pointCloud = transformsMappedToFrames.at(i).mFrame.pointCloud();

            // Transform point cloud
            pointCloud.transform(transformsMappedToFrames.at(i).mTransformationMatrix);

            // Stitch, and add color
            const auto rgba = pointCloud.copyColorsRGBA();
            const auto xyz = pointCloud.copyPointsXYZ();
            for(size_t j = 0; j < pointCloud.size(); j++)
            {
                if(!isnan(xyz(j).x))
                {
                    stitchedPointCloud.points[validPoints].x =
                        xyz(j).x; // NOLINT(cppcoreguidelines-pro-type-union-access)
                    stitchedPointCloud.points[validPoints].y =
                        xyz(j).y; // NOLINT(cppcoreguidelines-pro-type-union-access)
                    stitchedPointCloud.points[validPoints].z =
                        xyz(j).z; // NOLINT(cppcoreguidelines-pro-type-union-access)
                    if(useRGB)
                    {
                        stitchedPointCloud.points[validPoints].r =
                            rgba(j).r; // NOLINT(cppcoreguidelines-pro-type-union-access)
                        stitchedPointCloud.points[validPoints].g =
                            rgba(j).g; // NOLINT(cppcoreguidelines-pro-type-union-access)
                        stitchedPointCloud.points[validPoints].b =
                            rgba(j).b; // NOLINT(cppcoreguidelines-pro-type-union-access)
                    }
                    else
                    {
                        stitchedPointCloud.points[validPoints].rgb = rgbList.at(i);
                    }
                    validPoints++;
                }
            }
        }
        // Remove unused memory (would have been occupied by NaNs)
        stitchedPointCloud.points.resize(validPoints);
        std::cout << "Got " << validPoints << " out of " << maxNumberOfPoints << " points" << std::endl;

        // Simple Cloud Visualization
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZRGB>);
        *cloudPTR = stitchedPointCloud;

        std::cout << "Run the PCL visualizer. Block until window closes" << std::endl;
        pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
        viewer.showCloud(cloudPTR);
        std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
        std::cout << "Press q to me exit the viewer application" << std::endl;
        while(!viewer.wasStopped())
        {
        }
        if(saveStitched)
        {
            std::cerr << "Saving " << stitchedPointCloud.points.size()
                      << " data points to " + stitchedPointCloudFileName << std::endl;
            pcl::io::savePLYFileBinary(stitchedPointCloudFileName, stitchedPointCloud);
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
