/*
Use transformation matrices from Multi-Camera calibration to transform point clouds into single coordinate frame, from connected cameras.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.
*/

#include <Zivid/Experimental/PointCloudExport.h>
#include <Zivid/Experimental/SettingsInfo.h>
#include <Zivid/Zivid.h>

#include <Eigen/Core>
#include <clipp.h>
#include <open3d/Open3D.h>

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
