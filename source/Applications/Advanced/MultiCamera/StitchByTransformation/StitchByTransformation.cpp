/*
Use transformation matrices from Multi-Camera calibration to transform point clouds into single coordinate frame, from connected cameras.
*/

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <clipp.h>

#include <Zivid/Experimental/SettingsInfo.h>
#include <Zivid/Zivid.h>
#include <cmath>
#include <iostream>
#include <vector>

namespace
{
    Zivid::Frame assistedCapture(Zivid::Camera &camera)
    {
        const auto parameters = Zivid::CaptureAssistant::SuggestSettingsParameters{
            Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency::none,
            Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{std::chrono::milliseconds{800}}};
        const auto settings = Zivid::CaptureAssistant::suggestSettings(camera, parameters);
        return camera.capture(settings);
    }

    class TransformationMatrixAndCameraMap
    {
    public:
        TransformationMatrixAndCameraMap(Zivid::Matrix4x4 transformationMatrix, Zivid::Camera &camera)
            : mTransformationMatrix(transformationMatrix), mCamera(camera)
        {
        }

        const Zivid::Matrix4x4 mTransformationMatrix;
        Zivid::Camera &mCamera;
    };

    std::vector<TransformationMatrixAndCameraMap> getTransformationMatricesFromYAML(
        const std::vector<std::string> &transformationMatricesfileList,
        std::vector<Zivid::Camera> &cameras)
    {
        auto transformsMappedToCameras = std::vector<TransformationMatrixAndCameraMap>{};
        for (auto &camera : cameras)
        {
            const auto serialNumber = camera.info().serialNumber().toString();
            for (const auto &fileName : transformationMatricesfileList)
            {
                if (serialNumber == fileName.substr(
                                        fileName.find_last_of("\\/") + 1,
                                        (fileName.find_last_of('.')) - (fileName.find_last_of("\\/") + 1)))
                {
                    Zivid::Matrix4x4 transformationMatrixZivid(fileName);
                    transformsMappedToCameras.emplace_back(transformationMatrixZivid, camera);
                    break;
                }
                if (transformationMatricesfileList.back() == fileName)
                {
                    throw std::runtime_error("You are missing a YAML file named " + serialNumber + ".yaml!");
                }
            }
        }
        return transformsMappedToCameras;
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

        auto transformationMatricesfileList = std::vector<std::string>{};
        std::string stitchedPointCloudFileName;
        auto useRGB = true;
        auto saveStitched = false;
        auto cli =
            (clipp::values("File Names", transformationMatricesfileList) % "List of YAML files containing the transformation matrix.",
             clipp::option("-m", "--mono-chrome").set(useRGB, false) % "Color each point cloud with unique color.",
             clipp::required("-o", "--output-file").set(saveStitched) & clipp::value("Output point cloud (PLY) file name", stitchedPointCloudFileName) % "Save the stitched point cloud to a file with this name. (.ply)");

        if (!parse(argc, argv, cli))
        {
            std::cout << "SYNOPSIS:" << std::endl;
            std::cout << clipp::usage_lines(cli, "StitchByTransformation") << std::endl;
            std::cout << "OPTIONS:" << std::endl;
            std::cout << clipp::documentation(cli) << std::endl;
            throw std::runtime_error("No file provided.");
        }

        auto cameras = zivid.cameras();
        std::cout << "Number of cameras found: " << cameras.size() << std::endl;

        for (auto &camera : cameras)
        {
            std::cout << "Connecting camera: " << camera.info().serialNumber() << std::endl;
            camera.connect();
        }

        const auto transformsMappedToCameras =
            getTransformationMatricesFromYAML(transformationMatricesfileList, cameras);

        // Load camera settings
        const auto settingsFile = "./Configs/camera_settings.yml";
        std::cout << "Loading settings from file: " << settingsFile << std::endl;
        const auto settingsFromFile = Zivid::Settings(settingsFile);

        // Capture from all cameras
        auto frames = std::vector<Zivid::Frame>();
        auto maxNumberOfPoints = 0;
        for (const auto &transformAndCamera : transformsMappedToCameras)
        {
            std::cout << "Imaging from camera: " << transformAndCamera.mCamera.info().serialNumber() << std::endl;
            const auto frame = transformAndCamera.mCamera.capture(settingsFromFile); 
            // const auto frame = assistedCapture(transformAndCamera.mCamera);
            const auto resolution =
                Zivid::Experimental::SettingsInfo::resolution(transformAndCamera.mCamera.info(), frame.settings());
            maxNumberOfPoints += resolution.size();
            frames.push_back(frame);
        }

        // Stitch frames
        // Creating a PointCloud structure
        pcl::PointCloud<pcl::PointXYZRGB> stitchedPointCloud;

        // Filling in the cloud data, unorganized skipping NaNs
        stitchedPointCloud.points.resize(maxNumberOfPoints);

        size_t validPoints = 0;
        for (size_t i = 0; i < transformsMappedToCameras.size(); i++)
        {
            auto pointCloud = frames.at(i).pointCloud();

            // Transform point cloud
            pointCloud.transform(transformsMappedToCameras.at(i).mTransformationMatrix);

            // Stitch, and add color
            const auto rgba = pointCloud.copyColorsRGBA();
            const auto xyz = pointCloud.copyPointsXYZ();
            for (size_t j = 0; j < pointCloud.size(); j++)
            {
                if (!isnan(xyz(j).x))
                {
                    stitchedPointCloud.points[validPoints].x =
                        xyz(j).x; // NOLINT(cppcoreguidelines-pro-type-union-access)
                    stitchedPointCloud.points[validPoints].y =
                        xyz(j).y; // NOLINT(cppcoreguidelines-pro-type-union-access)
                    stitchedPointCloud.points[validPoints].z =
                        xyz(j).z; // NOLINT(cppcoreguidelines-pro-type-union-access)
                    if (useRGB)
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

        // std::cout << "Run the PCL visualizer. Block until window closes" << std::endl;
        // pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
        // viewer.showCloud(cloudPTR);
        // std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
        // std::cout << "Press q to me exit the viewer application" << std::endl;
        // while(!viewer.wasStopped())
        // {
        // }
        // printf("Viewer closed\n");
        if (saveStitched)
        {
            std::cerr << "Saving " << stitchedPointCloud.points.size()
                      << " data points to " + stitchedPointCloudFileName << std::endl;
            pcl::io::savePLYFileBinary(stitchedPointCloudFileName, stitchedPointCloud);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
