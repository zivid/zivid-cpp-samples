/*
Capture a point cloud, with colors, using Zivid SDK, transform it to a Halcon point cloud and save it using Halcon C++ SDK.
*/

#include <Zivid/Zivid.h>
#include <halconcpp/HalconCpp.h>

#include <algorithm>
#include <chrono>
#include <iostream>

namespace
{
    Zivid::Settings get2DAnd3DSettings(const Zivid::Camera &camera)
    {
        auto settings = Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} },
                                         Zivid::Settings::Color{ Zivid::Settings2D{
                                             Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } } } };

        auto model = camera.info().model();
        switch(model.value())
        {
            case Zivid::CameraInfo::Model::ValueType::zividTwo:
            case Zivid::CameraInfo::Model::ValueType::zividTwoL100:
            {
                settings.set(Zivid::Settings::Sampling::Pixel::all);
                settings.color().value().set(Zivid::Settings2D::Sampling::Pixel::all);
                break;
            }
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusM130:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusM60:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusL110:
            {
                settings.set(Zivid::Settings::Sampling::Pixel::blueSubsample2x2);
                settings.color().value().set(Zivid::Settings2D::Sampling::Pixel::blueSubsample2x2);
                break;
            }
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusMR130:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusMR60:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusLR110:
            {
                settings.set(Zivid::Settings::Sampling::Pixel::by2x2);
                settings.color().value().set(Zivid::Settings2D::Sampling::Pixel::by2x2);
                break;
            }
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusSmall:
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusMedium:
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusLarge:
            {
                throw std::runtime_error("Unsupported camera model '" + model.toString() + "'");
            }
            default: throw std::runtime_error("Unhandled enum value '" + model.toString() + "'");
        }

        return settings;
    }

    void savePointCloud(const HalconCpp::HObjectModel3D &model, const std::string &fileName)
    {
        model.WriteObjectModel3d(
            HalconCpp::HString{ "ply" },
            HalconCpp::HString{ fileName.c_str() },
            HalconCpp::HString{ "invert_normals" },
            HalconCpp::HString{ "false" });
    }

    HalconCpp::HObjectModel3D zividToHalconPointCloud(const Zivid::Frame &frame)
    {
        const auto pointCloud = frame.pointCloud();
        const auto width = pointCloud.width();
        const auto height = pointCloud.height();

        const auto pointsXYZ = pointCloud.copyPointsXYZ();
        const auto normalsXYZ = pointCloud.copyNormalsXYZ();
        const auto colorsRGBA = frame.frame2D().value().imageRGBA_SRGB();

        if(colorsRGBA.height() != height || colorsRGBA.width() != width)
        {
            throw std::runtime_error("Color image size does not match point cloud size");
        }

        int numberOfValidPoints =
            std::count_if(pointsXYZ.data(), pointsXYZ.data() + pointsXYZ.size(), [](const Zivid::PointXYZ &point) {
                return (!point.isNaN());
            });

        // Initializing HTuples which are later filled with data from the Zivid point cloud.
        // tupleXYZMapping is of shape [width, height, rows[], cols[]], and is used for creating xyz mapping.
        // See more at: https://www.mvtec.com/doc/halcon/13/en/set_object_model_3d_attrib.html

        HalconCpp::HTuple tuplePointsX, tuplePointsY, tuplePointsZ, tupleNormalsX, tupleNormalsY, tupleNormalsZ,
            tupleColorsR, tupleColorsB, tupleColorsG, tupleRow, tupleCol, tupleXYZMapping;

        tuplePointsX[numberOfValidPoints - 1] = (float)0.0;
        tuplePointsY[numberOfValidPoints - 1] = (float)0.0;
        tuplePointsZ[numberOfValidPoints - 1] = (float)0.0;
        tupleNormalsX[numberOfValidPoints - 1] = (float)0.0;
        tupleNormalsY[numberOfValidPoints - 1] = (float)0.0;
        tupleNormalsZ[numberOfValidPoints - 1] = (float)0.0;
        tupleColorsR[numberOfValidPoints - 1] = (Hlong)0;
        tupleColorsG[numberOfValidPoints - 1] = (Hlong)0;
        tupleColorsB[numberOfValidPoints - 1] = (Hlong)0;

        tupleXYZMapping[2 * numberOfValidPoints + 2 - 1] = (Hlong)0;
        tupleXYZMapping[0] = (Hlong)width;
        tupleXYZMapping[1] = (Hlong)height;

        int validPointIndex = 0;

        for(size_t i = 0; i < height; ++i)
        {
            for(size_t j = 0; j < width; ++j)
            {
                const auto &point = pointsXYZ(i, j);
                const auto &normal = normalsXYZ(i, j);
                const auto &color = colorsRGBA(i, j);

                if(!isnan(point.x))
                {
                    tuplePointsX.DArr()[validPointIndex] = point.x;
                    tuplePointsY.DArr()[validPointIndex] = point.y;
                    tuplePointsZ.DArr()[validPointIndex] = point.z;
                    tupleColorsR.LArr()[validPointIndex] = color.r;
                    tupleColorsG.LArr()[validPointIndex] = color.g;
                    tupleColorsB.LArr()[validPointIndex] = color.b;
                    tupleXYZMapping.LArr()[2 + validPointIndex] = i;
                    tupleXYZMapping.LArr()[2 + numberOfValidPoints + validPointIndex] = j;

                    if(!isnan(normal.x))
                    {
                        tupleNormalsX.DArr()[validPointIndex] = normal.x;
                        tupleNormalsY.DArr()[validPointIndex] = normal.y;
                        tupleNormalsZ.DArr()[validPointIndex] = normal.z;
                    }

                    validPointIndex++;
                }
            }
        }

        std::cout << "Constructing ObjectModel3D based on XYZ data" << std::endl;
        HalconCpp::HObjectModel3D objectModel3D(tuplePointsX, tuplePointsY, tuplePointsZ);

        std::cout << "Mapping ObjectModel3D data" << std::endl;
        HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "xyz_mapping", "object", tupleXYZMapping);

        std::cout << "Adding normals to ObjectModel3D" << std::endl;
        HalconCpp::HTuple normalsAttribNames, normalsAttribValues;
        normalsAttribNames.Append("point_normal_x");
        normalsAttribNames.Append("point_normal_y");
        normalsAttribNames.Append("point_normal_z");

        normalsAttribValues.Append(tupleNormalsX);
        normalsAttribValues.Append(tupleNormalsY);
        normalsAttribValues.Append(tupleNormalsZ);

        HalconCpp::SetObjectModel3dAttribMod(objectModel3D, normalsAttribNames, "points", normalsAttribValues);

        std::cout << "Adding RGB to ObjectModel3D" << std::endl;
        HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "red", "points", tupleColorsR);
        HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "green", "points", tupleColorsG);
        HalconCpp::SetObjectModel3dAttribMod(objectModel3D, "blue", "points", tupleColorsB);

        return objectModel3D;
    }
} // namespace

int main()
{
    try
    {
        std::cout << "Connecting to camera" << std::endl;
        Zivid::Application zivid;
        auto camera = zivid.connectCamera();

        std::cout << "Configuring capture settings" << std::endl;
        const auto settings = get2DAnd3DSettings(camera);

        std::cout << "Capturing frame" << std::endl;
        const auto frame = camera.capture2D3D(settings);

        std::cout << "Converting to Halcon point cloud" << std::endl;
        const auto halconPointCloud = zividToHalconPointCloud(frame);

        const auto pointCloudFile = "Zivid3D.ply";
        std::cout << "Saving point cloud to file: " << pointCloudFile << std::endl;
        savePointCloud(halconPointCloud, pointCloudFile);
    }

    catch(HalconCpp::HException &except)
    {
        std::cerr << "Error: " << except.ErrorMessage() << std::endl;
        return EXIT_FAILURE;
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
