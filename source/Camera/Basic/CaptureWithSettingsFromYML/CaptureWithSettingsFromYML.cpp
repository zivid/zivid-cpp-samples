/*
Capture images and point clouds, with or without color, from the Zivid camera with settings from YML file.

The YML files for this sample can be found under the main Zivid sample instructions.
*/

#include <Zivid/Experimental/PointCloudExport.h>
#include <Zivid/Zivid.h>

#include <iostream>

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

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Loading settings from file" << std::endl;
        const auto settingsFile =
            std::string(ZIVID_SAMPLE_DATA_DIR) + "/Settings/" + settingsFolder(camera) + "/Settings01.yml";
        const auto settings = Zivid::Settings(settingsFile);

        std::cout << "Capturing 2D frame" << std::endl;
        {
            const auto frame2D = camera.capture2D(settings);

            const auto imageSRGB = frame2D.imageSRGB();
            const auto imageFile = "ImageSRGB.png";
            std::cout << "Saving 2D color image (sRGB color space) to file: " << imageFile << std::endl;
            imageSRGB.save(imageFile);

            // More information about linear RGB and sRGB color spaces is available at:
            // https://support.zivid.com/en/latest/reference-articles/color-spaces-and-output-formats.html#color-spaces

            const auto pixelRow = 100;
            const auto pixelCol = 50;
            const auto pixelSRGB = imageSRGB(pixelRow, pixelCol);
            std::cout << "Color at pixel (" << pixelRow << "," << pixelCol << "):  R:" << std::to_string(pixelSRGB.r)
                      << "  G:" << std::to_string(pixelSRGB.g) << "  B:" << std::to_string(pixelSRGB.b)
                      << "  A:" << std::to_string(pixelSRGB.a) << std::endl;
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
