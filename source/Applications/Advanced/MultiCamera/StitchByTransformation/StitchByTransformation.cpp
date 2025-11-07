/*
Use transformation matrices from Multi-Camera calibration to transform point clouds into single coordinate frame, from connected cameras.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.
*/

#include <Zivid/Experimental/PointCloudExport.h>
#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <Eigen/Core>
#include <clipp.h>

#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace
{
    std::string sanitizedModelName(const Zivid::Camera &camera)
    {
        using Model = Zivid::CameraInfo::Model::ValueType;
        switch(camera.info().model().value())
        {
            case Model::zividTwo: return "Zivid_Two_M70";
            case Model::zividTwoL100: return "Zivid_Two_L100";
            case Model::zivid2PlusM130: return "Zivid_Two_Plus_M130";
            case Model::zivid2PlusM60: return "Zivid_Two_Plus_M60";
            case Model::zivid2PlusL110: return "Zivid_Two_Plus_L110";
            case Model::zivid2PlusMR130: return "Zivid_Two_Plus_MR130";
            case Model::zivid2PlusMR60: return "Zivid_Two_Plus_MR60";
            case Model::zivid2PlusLR110: return "Zivid_Two_Plus_LR110";
            case Model::zivid3XL250: return "Zivid_Three_XL250";
            case Model::zividOnePlusSmall: return "Zivid_One_Plus_Small";
            case Model::zividOnePlusMedium: return "Zivid_One_Plus_Medium";
            case Model::zividOnePlusLarge: return "Zivid_One_Plus_Large";

            default: throw std::runtime_error("Unhandled camera model: " + camera.info().model().toString());
        }
    }

    std::map<std::string, Zivid::Matrix4x4> getTransformationMatricesFromYAML(
        const std::vector<std::string> &transformationMatricesfileList,
        std::vector<Zivid::Camera> &cameras)
    {
        std::map<std::string, Zivid::Matrix4x4> transformsMappedToCameras;
        for(const auto &camera : cameras)
        {
            const auto serialNumber = camera.info().serialNumber().toString();
            for(const auto &fileName : transformationMatricesfileList)
            {
                if(serialNumber
                   == fileName.substr(
                       fileName.find_last_of("\\/") + 1,
                       (fileName.find_last_of('.')) - (fileName.find_last_of("\\/") + 1)))
                {
                    Zivid::Matrix4x4 transformationMatrixZivid(fileName);
                    transformsMappedToCameras[serialNumber] = transformationMatrixZivid;
                    break;
                }
                if(transformationMatricesfileList.back() == fileName)
                {
                    throw std::runtime_error("You are missing a YAML file named " + serialNumber + ".yaml!");
                }
            }
        }
        return transformsMappedToCameras;
    }

    std::vector<Zivid::Camera> connectToAllAvailableCameras(const std::vector<Zivid::Camera> &cameras)
    {
        std::vector<Zivid::Camera> connectedCameras;
        for(auto camera : cameras)
        {
            if(camera.state().status() == Zivid::CameraState::Status::available)
            {
                std::cout << "Connecting to camera: " << camera.info().serialNumber() << std::endl;
                camera.connect();
                connectedCameras.push_back(camera);
            }
            else
            {
                std::cout << "Camera " << camera.info().serialNumber() << "is not available. "
                          << "Camera status: " << camera.state().status() << std::endl;
            }
        }
        return connectedCameras;
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

        auto transformationMatricesfileList = std::vector<std::string>{};
        std::string stitchedPointCloudFileName;
        auto saveStitched = false;
        auto cli = (clipp::values("File Names", transformationMatricesfileList)
                        % "List of YAML files containing the corresponding transformation matrices.",
                    clipp::required("-o", "--output-file").set(saveStitched)
                        % "Save the stitched point cloud to a file with this name. (.ply)")
                   & clipp::value("Output point cloud (PLY) file name", stitchedPointCloudFileName);

        if(!parse(argc, argv, cli))
        {
            std::cout << "SYNOPSIS:" << std::endl;
            std::cout << clipp::usage_lines(cli, "StitchByTransformation") << std::endl;
            std::cout << "OPTIONS:" << std::endl;
            std::cout << clipp::documentation(cli) << std::endl;
            throw std::runtime_error("No file provided.");
        }

        auto cameras = zivid.cameras();
        std::cout << "Number of cameras found: " << cameras.size() << std::endl;

        auto connectedCameras = connectToAllAvailableCameras(cameras);
        const auto transformsMappedToCameras =
            getTransformationMatricesFromYAML(transformationMatricesfileList, connectedCameras);

        // Capture from all cameras
        Zivid::UnorganizedPointCloud stitchedPointCloud;

        for(auto &camera : connectedCameras)
        {
            const auto settingsPath = std::string(ZIVID_SAMPLE_DATA_DIR) + "/Settings/" + sanitizedModelName(camera)
                                      + "_ManufacturingSpecular.yml";
            std::cout << "Imaging from camera: " << camera.info().serialNumber() << std::endl;
            const auto frame = camera.capture2D3D(Zivid::Settings(settingsPath));
            const auto unorganizedPointCloud = frame.pointCloud().toUnorganizedPointCloud();
            const auto transformationMatrix = transformsMappedToCameras.at(camera.info().serialNumber().toString());
            const auto transformedUnorganizedPointCloud = unorganizedPointCloud.transformed(transformationMatrix);
            stitchedPointCloud.extend(transformedUnorganizedPointCloud);
        }

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
