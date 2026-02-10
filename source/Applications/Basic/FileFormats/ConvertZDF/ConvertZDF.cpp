/*
Convert point cloud data from a ZDF file to your preferred format
If a directory is provided, all ZDF files in the directory will be converted

Available formats:
    PLY, PCD, XYZ, CSV, TXT - 3D point cloud
    PNG, JPG, BMP - 2D RGB image
*/

#include <Zivid/Experimental/PointCloudExport.h>
#include <Zivid/Zivid.h>

#include <clipp.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace
{
    using ColorSpace = Zivid::Experimental::PointCloudExport::ColorSpace;
    using namespace Zivid::Experimental::PointCloudExport::FileFormat;

    std::string toLower(std::string str)
    {
        std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) { return std::tolower(c); });
        return str;
    }

    bool contains(const std::vector<std::string> &vec, const std::vector<std::string> &values)
    {
        return std::all_of(values.begin(), values.end(), [&](const std::string &value) {
            return std::find(vec.begin(), vec.end(), toLower(value)) != vec.end();
        });
    }

    template<typename ColorType>
    void writeColorsToFile(
        std::ofstream &file,
        const Zivid::Array2D<ColorType> &colors,
        const Zivid::Array2D<Zivid::PointXYZ> &points,
        const Zivid::Array2D<Zivid::SNR> &snrs,
        size_t size)
    {
        for(size_t i = 0; i < size; ++i)
        {
            const auto &point = points(i);
            const auto &color = colors(i);
            const auto &snr = snrs(i);

            if(!point.isNaN())
            {
                file << point.x << "," << point.y << "," << point.z << "," << static_cast<int>(color.r) << ","
                     << static_cast<int>(color.g) << "," << static_cast<int>(color.b) << ","
                     << static_cast<int>(color.a) << "," << snr.value << "\n";
            }
        }
    }

    template<typename ImageType>
    void save2DImage(
        const ImageType &image2D,
        const ImageType &image2DInPointCloudResolution,
        const std::filesystem::path &fileName,
        const std::filesystem::path &fileNamePointCloudResolution)
    {
        std::cout << "Saving the frame to " << fileName << " and " << fileNamePointCloudResolution << std::endl;
        image2D.save(fileName.string());
        image2DInPointCloudResolution.save(fileNamePointCloudResolution.string());
    }

    void
    flattenAndSavePointCloud(const Zivid::PointCloud &pointCloud, const std::filesystem::path &filePath, bool linearRgb)
    {
        std::ofstream file(filePath);
        if(!file.is_open())
        {
            throw std::runtime_error("Failed to open file: " + filePath.string());
        }

        file << std::fixed << std::setprecision(3);

        const auto points = pointCloud.copyPointsXYZ();
        const auto snrs = pointCloud.copySNRs();

        if(linearRgb)
        {
            const auto colors = pointCloud.copyColorsRGBA();
            writeColorsToFile(file, colors, points, snrs, pointCloud.size());
        }
        else
        {
            const auto colors = pointCloud.copyColorsRGBA_SRGB();
            writeColorsToFile(file, colors, points, snrs, pointCloud.size());
        }
    }

    void convertTo3D(
        const Zivid::Frame &frame,
        const std::filesystem::path &filePath,
        const std::vector<std::string> &fileFormats,
        bool linearRgb,
        bool unordered)
    {
        for(const auto &format : fileFormats)
        {
            const auto fileNameWithExtension = filePath.parent_path() / (filePath.stem().string() + "." + format);
            const auto colorSpace = linearRgb ? ColorSpace::linearRGB : ColorSpace::sRGB;

            std::cout << "Saving the frame to " << fileNameWithExtension << std::endl;

            if(format == "ply")
            {
                const auto layout = unordered ? PLY::Layout::unordered : PLY::Layout::ordered;
                Zivid::Experimental::PointCloudExport::exportFrame(
                    frame, PLY{ fileNameWithExtension.string(), layout, colorSpace });
            }
            else if(format == "pcd")
            {
                if(!unordered)
                {
                    std::cout
                        << "NOTE: If you have configured the config file for PCD, points will be ordered. "
                        << "If not they will be unordered. See "
                        << "https://support.zivid.com/en/latest/reference-articles/point-cloud-structure-and-output-formats.html#organized-pcd-format"
                        << " for more information." << std::endl;
                }
                Zivid::Experimental::PointCloudExport::exportFrame(
                    frame, PCD{ fileNameWithExtension.string(), colorSpace });
            }
            else if(format == "xyz")
            {
                Zivid::Experimental::PointCloudExport::exportFrame(
                    frame, XYZ{ fileNameWithExtension.string(), colorSpace });
            }
            else if(format == "csv" || format == "txt")
            {
                flattenAndSavePointCloud(frame.pointCloud(), fileNameWithExtension, linearRgb);
            }
        }
    }

    void convertTo2D(
        const Zivid::Frame &frame,
        const std::filesystem::path &filePath,
        const std::vector<std::string> &fileFormats,
        bool linearRgb)
    {
        for(const auto &format : fileFormats)
        {
            const auto fileName = filePath.parent_path() / (filePath.stem().string() + "." + format);
            const auto fileNamePointCloudResolution =
                filePath.parent_path() / (filePath.stem().string() + "_point_cloud_resolution." + format);

            if(linearRgb)
            {
                save2DImage(
                    frame.frame2D()->imageRGBA(),
                    frame.pointCloud().copyImageRGBA(),
                    fileName,
                    fileNamePointCloudResolution);
            }
            else
            {
                save2DImage(
                    frame.frame2D()->imageRGBA_SRGB(),
                    frame.pointCloud().copyImageRGBA_SRGB(),
                    fileName,
                    fileNamePointCloudResolution);
            }
        }
    }
} // namespace

int main(int argc, char **argv)
{
    try
    {
        std::string inputPath;
        std::vector<std::string> formats3DSelected;
        std::vector<std::string> formats2DSelected;
        bool convertAll = false;
        bool linearRgb = false;
        bool unordered = false;
        bool showHelp = false;
        const std::vector<std::string> formats3D = { "ply", "pcd", "xyz", "csv", "txt" };
        const std::vector<std::string> formats2D = { "jpg", "png", "bmp" };

        auto cli =
            (clipp::option("-h", "--help").set(showHelp) % "Show this help message",
             clipp::value("path", inputPath) % "File/directory holding ZDF file(s)",
             clipp::option("-a", "--all").set(convertAll) % "Convert to all formats (default if no formats specified)",
             clipp::option("--3d")
                 & clipp::values("formats3D", formats3DSelected)
                       % "3D format(s) to convert to (ply, pcd, xyz, csv, txt)",
             clipp::option("--2d")
                 & clipp::values("formats2D", formats2DSelected) % "2D format(s) to convert to (jpg, png, bmp)",
             clipp::option("--linearRGB").set(linearRgb)
                 % "Use linear RGB color space instead of sRGB for selected format(s)",
             clipp::option("--unordered").set(unordered)
                 % "Save point clouds as unordered instead of ordered (PLY, PCD)");

        if(!clipp::parse(argc, argv, cli) || showHelp || inputPath.empty() || !contains(formats3D, formats3DSelected)
           || !contains(formats2D, formats2DSelected))
        {
            auto fmt = clipp::doc_formatting{}.alternatives_min_split_size(1).surround_labels("\"", "\"");
            std::cout << "Convert from a ZDF to your preferred format\n\n";
            std::cout << "SYNOPSIS:\n";
            std::cout << clipp::usage_lines(cli, "ConvertZDF", fmt) << "\n\n";
            std::cout << "OPTIONS:\n";
            std::cout << clipp::documentation(cli) << "\n";
            std::cout << "\nExample:\n";
            std::cout << "  ConvertZDF Zivid3D.zdf --3d ply xyz csv --2d jpg png\n";
            return showHelp ? EXIT_FAILURE : EXIT_SUCCESS;
        }

        const std::filesystem::path path(inputPath);
        if(!std::filesystem::exists(path))
        {
            throw std::runtime_error(inputPath + " does not exist");
        }

        Zivid::Application zivid;

        std::vector<std::pair<Zivid::Frame, std::filesystem::path>> frames;

        std::cout << "Reading point cloud(s) from: " << inputPath << std::endl;

        if(std::filesystem::is_directory(path))
        {
            for(const auto &entry : std::filesystem::directory_iterator(path))
            {
                if(entry.path().extension() == ".zdf")
                {
                    frames.emplace_back(Zivid::Frame(entry.path().string()), entry.path());
                }
            }
        }
        else
        {
            frames.emplace_back(Zivid::Frame(inputPath), path);
        }

        if(frames.empty())
        {
            throw std::runtime_error(inputPath + " does not contain any ZDF files");
        }

        // If no formats specified or --all is set, convert to all formats
        if(convertAll || (formats3DSelected.empty() && formats2DSelected.empty()))
        {
            formats3DSelected = formats3D;
            formats2DSelected = formats2D;
        }

        for(const auto &[frame, filePath] : frames)
        {
            if(!formats3DSelected.empty())
            {
                convertTo3D(frame, filePath, formats3DSelected, linearRgb, unordered);
            }

            if(!formats2DSelected.empty())
            {
                convertTo2D(frame, filePath, formats2DSelected, linearRgb);
            }
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
