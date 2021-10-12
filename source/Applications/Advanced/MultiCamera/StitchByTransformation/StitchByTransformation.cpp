#include <opencv2/core/core.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <clipp.h>

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
            Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{ std::chrono::milliseconds{ 800 } }
        };
        const auto settings = Zivid::CaptureAssistant::suggestSettings(camera, parameters);
        return camera.capture(settings);
    }

    class TransformationMatrixAndCameraMap
    {
    public:
        TransformationMatrixAndCameraMap(Zivid::Matrix4x4 transformationMatrix, Zivid::Camera &camera)
            : mTransformationMatrix(transformationMatrix)
            , mCamera(camera)
        {}

        const Zivid::Matrix4x4 mTransformationMatrix;
        Zivid::Camera &mCamera;
    };

    Zivid::Camera &getCameraBySerialNumber(std::vector<Zivid::Camera> &cameras,
                                           const Zivid::CameraInfo::SerialNumber &serialNumber)
    {
        auto cameraIterator = std::find_if(cameras.begin(), cameras.end(), [&](Zivid::Camera &camera) {
            return (camera.info().serialNumber() == serialNumber);
        });
        if(cameraIterator == cameras.end())
        {
            throw std::runtime_error("Camera " + serialNumber.toString() + " not connected");
        }
        return *cameraIterator;
    }

    std::vector<TransformationMatrixAndCameraMap> getTransformationMatricesFromYAML(const std::string &path,
                                                                                    std::vector<Zivid::Camera> &cameras)
    {
        cv::FileStorage fileStorageIn;
        if(!fileStorageIn.open(path, cv::FileStorage::Mode::READ))
        {
            throw std::runtime_error("Could not open " + path + ". Please run this sample from the build directory");
        }
        auto transformsMappedToCameras = std::vector<TransformationMatrixAndCameraMap>{};
        for(size_t i = 0; i < cameras.size(); i++)
        {
            const auto serialNumber =
                Zivid::CameraInfo::SerialNumber(fileStorageIn["SerialNumber_" + std::to_string(i)].string());
            std::cout << "SerialNumber_" << std::to_string(i) << ": " << serialNumber << std::endl;
            const auto transformNode = fileStorageIn["TransformationMatrix_" + std::to_string(i)];
            const auto &cvMat = transformNode.mat();
            const auto transformationMatrix = Zivid::Matrix4x4(cvMat.ptr<float>(0), cvMat.ptr<float>(0) + 16);
            transformsMappedToCameras.emplace_back(transformationMatrix,
                                                   getCameraBySerialNumber(cameras, serialNumber));
            std::cout << "TransformationMatrix_" << i << ":" << std::endl;
            std::cout << transformationMatrix << std::endl;
        }

        fileStorageIn.release();

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
        // Find and connect all cameras
        Zivid::Application zivid;

        std::string transformationMatricesFileName;
        std::string stitchedPointCloudFileName;
        auto useRGB = true;
        auto saveStitched = false;
        auto cli =
            (clipp::value("Transformation Matrices File Name", transformationMatricesFileName)
                 % "Path to .yaml file with all transformation matrices",
             clipp::option("-m", "--mono-chrome").set(useRGB, false) % "Color each point cloud with unique color.",
             clipp::option("-o", "--outputfile").set(saveStitched)
                 & clipp::value("Transformation Matrices File Name", stitchedPointCloudFileName)
                       % "Save the stitched point cloud to a file with this name.");

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

        for(auto &camera : cameras)
        {
            std::cout << "Connecting camera: " << camera.info().serialNumber() << std::endl;
            camera.connect();
        }

        const auto transformsMappedToCameras =
            getTransformationMatricesFromYAML(transformationMatricesFileName, cameras);

        // Capture from all cameras
        auto frames = std::vector<Zivid::Frame>();
        auto maxNumberOfPoints = 0;
        for(const auto &transformAndCamera : transformsMappedToCameras)
        {
            std::cout << "Imaging from camera: " << transformAndCamera.mCamera.info().serialNumber() << std::endl;
            const auto frame = assistedCapture(transformAndCamera.mCamera);
            maxNumberOfPoints += frame.pointCloud().size();
            frames.push_back(frame);
        }

        // Stitch frames
        // Creating a PointCloud structure
        pcl::PointCloud<pcl::PointXYZRGB> stitchedPointCloud;

        // Filling in the cloud data, unorganized skipping NaNs
        stitchedPointCloud.points.resize(maxNumberOfPoints);

        size_t validPoints = 0;
        for(size_t i = 0; i < transformsMappedToCameras.size(); i++)
        {
            auto pointCloud = frames.at(i).pointCloud();

            // Transform point cloud
            pointCloud.transform(transformsMappedToCameras.at(i).mTransformationMatrix);

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
        {}
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
}
