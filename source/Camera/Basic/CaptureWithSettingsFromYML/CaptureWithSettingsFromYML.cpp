/*
Capture images and point clouds, with and without color, from the Zivid camera with settings from YML file.

Choose whether to get the image in the linear RGB or the sRGB color space.

The YML files for this sample can be found under the main Zivid sample instructions.
*/

#include <Zivid/Experimental/PointCloudExport.h>
#include <Zivid/Zivid.h>
#include <clipp.h>
#include <iostream>
#include <string>

namespace
{
    std::string settingsFolder(const Zivid::Camera &camera)
    {
        switch(camera.info().model().value())
        {
            case Zivid::CameraInfo::Model::ValueType::zividTwo:
            case Zivid::CameraInfo::Model::ValueType::zividTwoL100: return "zivid2";
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusM130:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusM60:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusL110: return "zivid2Plus";
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusMR130:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusMR60:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusLR110: return "zivid2Plus/R";
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusSmall:
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusMedium:
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusLarge: break;

            default: throw std::runtime_error("Unhandled enum value '" + camera.info().model().toString() + "'");
        }
        throw std::invalid_argument("Invalid camera model");
    }
} // namespace

int main(int argc, char *argv[])
{
    try
    {
        bool linearRgb = false;
        std::string settingsPath;
        bool showHelp = false;

        auto cli =
            (clipp::option("-h", "--help").set(showHelp) % "Show help message",
             clipp::option("--settings-path")
                 & clipp::value("path", settingsPath) % "Path to the camera settings YML file",
             clipp::option("--linear-rgb").set(linearRgb) % "Use linear RGB instead of sRGB");

        if(!clipp::parse(argc, argv, cli) || showHelp)
        {
            auto fmt = clipp::doc_formatting{}.alternatives_min_split_size(1).surround_labels("\"", "\"");
            std::cout << "USAGE:" << std::endl;
            std::cout << clipp::usage_lines(cli, argv[0], fmt) << std::endl;
            std::cout << "OPTIONS:" << std::endl;
            std::cout << clipp::documentation(cli) << std::endl;
            return showHelp ? EXIT_SUCCESS : EXIT_FAILURE;
        }

        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Loading settings from file " << std::endl;
        if(settingsPath.empty())
        {
            const auto settingsFile =
                std::string(ZIVID_SAMPLE_DATA_DIR) + "/Settings/" + settingsFolder(camera) + "/Settings01.yml";
            settingsPath = settingsFile;
        }

        const auto settings = Zivid::Settings(settingsPath);

        std::cout << "Capturing 2D frame" << std::endl;
        {
            const auto frame2D = camera.capture2D(settings);

            const auto pixelRow = 100;
            const auto pixelCol = 50;

            if(linearRgb)
            {
                const auto imageRGBA = frame2D.imageRGBA();
                const auto imageFile = "ImageRGBA_linear.png";
                std::cout << "Saving 2D color image (Linear RGB) to file: " << imageFile << std::endl;
                imageRGBA.save(imageFile);

                std::cout << "Extracting 2D pixel array" << std::endl;
                const auto pixelArrayRGBA = imageRGBA(pixelRow, pixelCol);
                std::cout << "Color at pixel (" << pixelRow << "," << pixelCol
                          << "):  R:" << std::to_string(pixelArrayRGBA.r) << "  G:" << std::to_string(pixelArrayRGBA.g)
                          << "  B:" << std::to_string(pixelArrayRGBA.b) << "  A:" << std::to_string(pixelArrayRGBA.a)
                          << std::endl;
            }
            else
            {
                const auto imageSRGB = frame2D.imageRGBA_SRGB();
                const auto imageFile = "ImageRGBA_sRGB.png";
                std::cout << "Saving 2D color image (sRGB color space) to file: " << imageFile << std::endl;
                imageSRGB.save(imageFile);

                std::cout << "Extracting 2D pixel array" << std::endl;
                const auto pixelArraySRGB = imageSRGB(pixelRow, pixelCol);
                std::cout << "Color at pixel (" << pixelRow << "," << pixelCol
                          << "):  R:" << std::to_string(pixelArraySRGB.r) << "  G:" << std::to_string(pixelArraySRGB.g)
                          << "  B:" << std::to_string(pixelArraySRGB.b) << "  A:" << std::to_string(pixelArraySRGB.a)
                          << std::endl;
            }
        }

        std::cout << "Capturing 3D frame" << std::endl;
        {
            const auto frame3D = camera.capture3D(settings);
            const auto dataFile = "Frame3D.zdf";
            std::cout << "Saving frame to file: " << dataFile << std::endl;
            frame3D.save(dataFile);

            const auto dataFilePly = "PointCloudWithoutColor.ply";
            std::cout << "Exporting point cloud (default pink colored points) to file: " << dataFilePly << std::endl;
            frame3D.save(dataFilePly);
        }

        std::cout << "Capturing 2D3D frame" << std::endl;
        {
            const auto frame = camera.capture2D3D(settings);

            const auto dataFile = "Frame.zdf";
            std::cout << "Saving frame to file: " << dataFile << std::endl;
            frame.save(dataFile);

            Zivid::Experimental::PointCloudExport::FileFormat::PLY plyFile{
                "PointCloudWithColor.ply",
                Zivid::Experimental::PointCloudExport::FileFormat::PLY::Layout::ordered,
                Zivid::Experimental::PointCloudExport::ColorSpace::sRGB
            };
            std::cout << "Exporting point cloud to file: " << plyFile.fileName() << std::endl;
            Zivid::Experimental::PointCloudExport::exportFrame(frame, plyFile);
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
