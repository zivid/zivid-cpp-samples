/*
Stitch multiple point clouds captured with a robot mounted camera.

The sample simulates a camera capturing the point clouds at different robot poses.
The point clouds are pre-aligned using the robot's pose and a hand-eye calibration transform.
The resulting stitched point cloud is displayed and saved to a PLY file.

The sample demonstrates stitching of a small and a big object.
The small object fits within the camera's field of view and the result of stitching is a full
point cloud of the object, seen from different angles, i.e, from the front, back, left, and right sides.
The big object does not fit within the camera's field of view, so the stitching is done to extend the
field of view of the camera, and see the object in full.

The resulting stitched point cloud is voxel downsampled if the `--full-resolution` flag is not set.

Dataset: https://support.zivid.com/en/latest/api-reference/samples/sample-data.html

Extract the content into:
    - Windows:   %ProgramData%/Zivid/StitchingPointClouds/
    - Linux:     /usr/share/Zivid/data/StitchingPointClouds/

    StitchingPointClouds/
        ├── SmallObject/
        └── BigObject/

Each of these folders must contain ZDF captures, robot poses, and a hand-eye transform file.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.
*/

#include <Zivid/Experimental/LocalPointCloudRegistrationParameters.h>
#include <Zivid/Experimental/PointCloudExport.h>
#include <Zivid/Experimental/Toolbox/PointCloudRegistration.h>
#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <clipp.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <filesystem>
#include <iostream>
#include <regex>
#include <string>

namespace
{
    struct RegistrationResults
    {
        Zivid::Matrix4x4 baseToCameraTransform;
        Zivid::Matrix4x4 previousToCurrentTransform;
    };

    void visualizePointCloud(const Zivid::UnorganizedPointCloud &unorganizedPointCloud)
    {
        Zivid::Visualization::Visualizer visualizer;

        visualizer.showMaximized();
        visualizer.show(unorganizedPointCloud);
        visualizer.resetToFit();

        std::cout << "Running visualizer. Blocking until window closes." << std::endl;
        visualizer.run();
    }

    Eigen::Affine3f zividToEigen(const Zivid::Matrix4x4 &zividMatrix)
    {
        Eigen::Matrix4f eigenMatrix;
        for(std::size_t row = 0; row < Zivid::Matrix4x4::rows; row++)
        {
            for(std::size_t column = 0; column < Zivid::Matrix4x4::cols; column++)
            {
                eigenMatrix(row, column) = zividMatrix(row, column);
            }
        }
        Eigen::Affine3f eigenTransform{ eigenMatrix };
        return eigenTransform;
    }

    Zivid::Matrix4x4 eigenToZivid(const Eigen::Affine3f &eigenTransform)
    {
        Eigen::Matrix4f eigenMatrix = eigenTransform.matrix();
        Zivid::Matrix4x4 zividMatrix;
        for(Eigen::Index row = 0; row < eigenMatrix.rows(); row++)
        {
            for(Eigen::Index column = 0; column < eigenMatrix.cols(); column++)
            {
                zividMatrix(row, column) = eigenMatrix(row, column);
            }
        }
        return zividMatrix;
    }

    void getZDFAndPoses(
        const std::filesystem::path &directory,
        std::vector<std::filesystem::path> &zdfFilePaths,
        std::vector<std::filesystem::path> &poseFilePaths)
    {
        const std::regex zdfPattern(R"(capture_.*\.zdf)");
        const std::regex posePattern(R"(robot_pose_.*\.yaml)");

        for(const auto &entry : std::filesystem::recursive_directory_iterator(directory))
        {
            if(!std::filesystem::is_regular_file(entry))
            {
                continue;
            }

            const auto &path = entry.path();
            const std::string filename = path.filename().string();

            if(std::regex_match(filename, zdfPattern))
            {
                zdfFilePaths.push_back(path);
            }
            else if(std::regex_match(filename, posePattern))
            {
                poseFilePaths.push_back(path);
            }
        }

        if(zdfFilePaths.empty())
        {
            throw std::runtime_error("No ZDF files found.");
        }

        if(poseFilePaths.empty())
        {
            throw std::runtime_error("No robot pose files found.");
        }

        if(!std::filesystem::exists(directory / "hand_eye_transform.yaml"))
        {
            throw std::runtime_error("Missing hand_eye_transform.yaml.");
        }

        if(zdfFilePaths.size() != poseFilePaths.size())
        {
            throw std::runtime_error("Number of ZDF files and robot pose files do not match.");
        }

        std::sort(zdfFilePaths.begin(), zdfFilePaths.end());
        std::sort(poseFilePaths.begin(), poseFilePaths.end());
    }

    Zivid::UnorganizedPointCloud stitchPointClouds(const std::filesystem::path &directory, const bool fullResolution)
    {
        std::vector<std::filesystem::path> zdfFilePaths;
        std::vector<std::filesystem::path> poseFilePaths;

        getZDFAndPoses(directory, zdfFilePaths, poseFilePaths);

        Zivid::Matrix4x4 handEyeTransform((directory / "hand_eye_transform.yaml").string());

        auto previousToCurrentTransform = Zivid::Matrix4x4::identity();
        Zivid::UnorganizedPointCloud accumulatedPointCloud;

        auto registrationParameters = Zivid::Experimental::LocalPointCloudRegistrationParameters{
            Zivid::Experimental::LocalPointCloudRegistrationParameters::MaxCorrespondenceDistance{ 2 }
        };

        std::vector<RegistrationResults> poseTransforms;

        for(std::size_t index = 0; index < zdfFilePaths.size(); ++index)
        {
            const Zivid::Matrix4x4 robotPose(poseFilePaths[index].string());
            const Zivid::Frame frame(zdfFilePaths[index].string());

            const auto baseToCameraTransform = eigenToZivid(zividToEigen(robotPose) * zividToEigen(handEyeTransform));
            Zivid::UnorganizedPointCloud organizedPointCloudInBaseFrame =
                frame.pointCloud().toUnorganizedPointCloud().voxelDownsampled(1.0, 2).transform(baseToCameraTransform);

            if(index != 0)
            {
                const auto registrationResult = Zivid::Experimental::Toolbox::localPointCloudRegistration(
                    accumulatedPointCloud, organizedPointCloudInBaseFrame, registrationParameters);

                if(!registrationResult.converged())
                {
                    throw std::runtime_error(
                        "Registration did not converge for the point cloud -> " + zdfFilePaths[index].string());
                }

                previousToCurrentTransform = registrationResult.transform().toMatrix();
                accumulatedPointCloud.transform(previousToCurrentTransform.inverse());

                std::cout << (index + 1) << " out of " << zdfFilePaths.size()
                          << (fullResolution ? " point clouds aligned." : " point clouds stitched.") << std::endl;
            }
            poseTransforms.push_back({ baseToCameraTransform, previousToCurrentTransform });
            accumulatedPointCloud.extend(organizedPointCloudInBaseFrame);
        }

        Zivid::UnorganizedPointCloud finalPointCloud;
        if(!fullResolution)
        {
            //Downsampling the final result for efficiency
            finalPointCloud = accumulatedPointCloud.voxelDownsampled(1.0, 2);
        }
        else
        {
            for(std::size_t index = 0; index < poseTransforms.size(); ++index)
            {
                const std::filesystem::path &zdf = zdfFilePaths[index];
                const Zivid::Frame frame(zdf.string());
                RegistrationResults registrationResult = poseTransforms[index];

                frame.pointCloud().transform(registrationResult.baseToCameraTransform);
                finalPointCloud.transform(registrationResult.previousToCurrentTransform.inverse());
                finalPointCloud.extend(frame.pointCloud().toUnorganizedPointCloud());

                if(index > 0)
                {
                    std::cout << (index + 1) << " out of " << zdfFilePaths.size() << " point clouds stitched."
                              << std::endl;
                }
            }
        }
        return finalPointCloud;
    }
} // namespace

int main(int argc, char **argv)
{
    try
    {
        bool fullResolution = false;
        bool showHelp = false;

        auto cli = clipp::group(
            (clipp::option("-h", "--help").set(showHelp) % "Show help message",
             clipp::option("--full-resolution").set(fullResolution)
                 % "Use full resolution for stitching. If not set, downsampling is applied."));

        if(!clipp::parse(argc, argv, cli) || showHelp)
        {
            auto fmt = clipp::doc_formatting{}.alternatives_min_split_size(1).surround_labels("\"", "\"");
            std::cout << "SYNOPSIS:" << std::endl;
            std::cout << clipp::usage_lines(cli, "StitchUsingRobotMountedCamera", fmt) << std::endl;
            std::cout << "OPTIONS:" << std::endl;
            std::cout << clipp::documentation(cli) << std::endl;

            if(showHelp)
            {
                return 0;
            }
            throw std::runtime_error("Command-line parsing failed.");
        }

        Zivid::Application app;

        // Ensure the dataset is extracted to the correct location depending on the operating system:
        //   - Windows:   %ProgramData%/Zivid/StitchingPointClouds/
        //   - Linux:     /usr/share/Zivid/data/StitchingPointClouds/
        //   StitchingPointClouds/
        //     ├── SmallObject/
        //     └── BigObject/
        // Each folder must include:
        //   - capture_*.zdf
        //   - robot_pose_*.yaml
        //   - hand_eye_transform.yaml
        const auto smallObjectDir =
            std::filesystem::path(ZIVID_SAMPLE_DATA_DIR) / "StitchingPointClouds" / "SmallObject";
        const auto bigObjectDir = std::filesystem::path(ZIVID_SAMPLE_DATA_DIR) / "StitchingPointClouds" / "BigObject";

        if(!std::filesystem::exists(smallObjectDir) || !std::filesystem::exists(bigObjectDir))
        {
            std::ostringstream oss;
            oss << "Missing dataset folders.\n"
                << "Make sure 'StitchingPointClouds/SmallObject' and 'StitchingPointClouds/BigObject' exist at "
                << ZIVID_SAMPLE_DATA_DIR << ".\n\n"
                << "You can download the dataset (StitchingPointClouds.zip) from:\n"
                << "https://support.zivid.com/en/latest/api-reference/samples/sample-data.html";

            throw std::runtime_error(oss.str());
        }

        //Small object
        std::cout << "Stitching small object..." << std::endl;
        const Zivid::UnorganizedPointCloud finalPointCloudSmallObject =
            stitchPointClouds(smallObjectDir, fullResolution);

        visualizePointCloud(finalPointCloudSmallObject);

        const auto fileNameSmall = "StitchedPointCloudSmallObject.ply";
        Zivid::Experimental::PointCloudExport::FileFormat::PLY plyFileSmall{
            fileNameSmall,
            Zivid::Experimental::PointCloudExport::FileFormat::PLY::Layout::unordered,
            Zivid::Experimental::PointCloudExport::ColorSpace::sRGB
        };
        std::cout << "Exporting point cloud to file: " << plyFileSmall.fileName() << std::endl;
        Zivid::Experimental::PointCloudExport::exportUnorganizedPointCloud(finalPointCloudSmallObject, plyFileSmall);

        //Big object
        std::cout << "Stitching big object..." << std::endl;
        const Zivid::UnorganizedPointCloud finalPointCloudBigObject = stitchPointClouds(bigObjectDir, fullResolution);

        visualizePointCloud(finalPointCloudBigObject);

        const auto fileNameBig = "StitchedPointCloudBigObject.ply";
        Zivid::Experimental::PointCloudExport::FileFormat::PLY plyFileBig{
            fileNameBig,
            Zivid::Experimental::PointCloudExport::FileFormat::PLY::Layout::unordered,
            Zivid::Experimental::PointCloudExport::ColorSpace::sRGB
        };
        std::cout << "Exporting point cloud to file: " << plyFileBig.fileName() << std::endl;
        Zivid::Experimental::PointCloudExport::exportUnorganizedPointCloud(finalPointCloudBigObject, plyFileBig);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
