/*
Capture a point cloud, with colors, using Zivid SDK, transform it to a Halcon point cloud and save it using Halcon C++ SDK.
*/

#include <Zivid/Zivid.h>
#include <halconcpp/HalconCpp.h>

#include <algorithm>
#include <iostream>

namespace
{
    std::string presetPath(const Zivid::Camera &camera)
    {
        const std::string presetsPath = std::string(ZIVID_SAMPLE_DATA_DIR) + "/Settings";

        switch(camera.info().model().value())
        {
            case Zivid::CameraInfo::Model::ValueType::zividTwo:
            {
                return presetsPath + "/Zivid_Two_M70_ManufacturingSpecular.yml";
            }
            case Zivid::CameraInfo::Model::ValueType::zividTwoL100:
            {
                return presetsPath + "/Zivid_Two_L100_ManufacturingSpecular.yml";
            }
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusM130:
            {
                return presetsPath + "/Zivid_Two_Plus_M130_ConsumerGoodsQuality.yml";
            }
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusM60:
            {
                return presetsPath + "/Zivid_Two_Plus_M60_ConsumerGoodsQuality.yml";
            }
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusL110:
            {
                return presetsPath + "/Zivid_Two_Plus_L110_ConsumerGoodsQuality.yml";
            }
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusMR130:
            {
                return presetsPath + "/Zivid_Two_Plus_MR130_ConsumerGoodsQuality.yml";
            }
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusMR60:
            {
                return presetsPath + "/Zivid_Two_Plus_MR60_ConsumerGoodsQuality.yml";
            }
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusLR110:
            {
                return presetsPath + "/Zivid_Two_Plus_LR110_ConsumerGoodsQuality.yml";
            }
            case Zivid::CameraInfo::Model::ValueType::zivid3XL250:
            {
                return presetsPath + "/Zivid_Three_XL250_DepalletizationQuality.yml";
            }
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusSmall:
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusMedium:
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusLarge: break;

            default: throw std::runtime_error("Unhandled enum value '" + camera.info().model().toString() + "'");
        }
        throw std::invalid_argument("Invalid camera model");
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
            throw std::runtime_error("the 2D image resolution must match the 3D point cloud resolution");
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
        const auto settingsPath = presetPath(camera);
        const auto settings = Zivid::Settings(settingsPath);

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
