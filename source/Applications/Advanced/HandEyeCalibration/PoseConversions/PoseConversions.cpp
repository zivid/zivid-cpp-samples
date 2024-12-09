/*
Convert to/from Transformation Matrix (Rotation Matrix + Translation Vector)

Zivid primarily operate with a (4x4) transformation matrix. This example shows how to use Eigen to
convert to and from: AxisAngle, Rotation Vector, Roll-Pitch-Yaw, Quaternion

The convenience functions from this example can be reused in applicable applications. The YAML files for this sample can
be found under the main instructions for Zivid samples.
*/

#include <Zivid/Zivid.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace
{
    enum class RotationConvention
    {
        zyxIntrinsic,
        xyzExtrinsic,
        xyzIntrinsic,
        zyxExtrinsic,
        nofROT
    };
    constexpr size_t nofRotationConventions = static_cast<size_t>(RotationConvention::nofROT);

    struct RollPitchYaw
    {
        RotationConvention convention;
        Eigen::Array3f rollPitchYaw;
    };

    std::string toString(RotationConvention convention)
    {
        switch(convention)
        {
            case RotationConvention::xyzIntrinsic: return "xyzIntrinsic";
            case RotationConvention::xyzExtrinsic: return "xyzExtrinsic";
            case RotationConvention::zyxIntrinsic: return "zyxIntrinsic";
            case RotationConvention::zyxExtrinsic: return "zyxExtrinsic";
            case RotationConvention::nofROT: break;
            default: throw std::runtime_error{ "Unhandled rotation convention " };
        }

        throw std::invalid_argument("Invalid RotationConvention");
    }

    // The following function converts Roll-Pitch-Yaw angles in radians to Rotation Matrix.
    // This function takes an array of angles, and a rotation convention, as input parameters.
    // Whether the axes are moving (intrinsic) or fixed (extrinsic) is defined by the rotation convention.
    // The order of elements in the output array follow the convention, i.e., for ZYX and zyx, the first
    // element of the array is rotation around z-axis, for XYZ and xyz, the first element is rotation
    // around x-axis.
    Eigen::Matrix3f rollPitchYawToRotationMatrix(const Eigen::Array3f &rollPitchYaw, const RotationConvention &rotation)
    {
        switch(rotation)
        {
            case RotationConvention::xyzIntrinsic:
                return (Eigen::AngleAxisf(rollPitchYaw[0], Eigen::Vector3f::UnitX())
                        * Eigen::AngleAxisf(rollPitchYaw[1], Eigen::Vector3f::UnitY())
                        * Eigen::AngleAxisf(rollPitchYaw[2], Eigen::Vector3f::UnitZ()))
                    .matrix();
            case RotationConvention::zyxExtrinsic:
                return (Eigen::AngleAxisf(rollPitchYaw[2], Eigen::Vector3f::UnitX())
                        * Eigen::AngleAxisf(rollPitchYaw[1], Eigen::Vector3f::UnitY())
                        * Eigen::AngleAxisf(rollPitchYaw[0], Eigen::Vector3f::UnitZ()))
                    .matrix();
            case RotationConvention::zyxIntrinsic:
                return (Eigen::AngleAxisf(rollPitchYaw[0], Eigen::Vector3f::UnitZ())
                        * Eigen::AngleAxisf(rollPitchYaw[1], Eigen::Vector3f::UnitY())
                        * Eigen::AngleAxisf(rollPitchYaw[2], Eigen::Vector3f::UnitX()))
                    .matrix();
            case RotationConvention::xyzExtrinsic:
                return (Eigen::AngleAxisf(rollPitchYaw[2], Eigen::Vector3f::UnitZ())
                        * Eigen::AngleAxisf(rollPitchYaw[1], Eigen::Vector3f::UnitY())
                        * Eigen::AngleAxisf(rollPitchYaw[0], Eigen::Vector3f::UnitX()))
                    .matrix();
            case RotationConvention::nofROT: break;
            default: throw std::runtime_error{ "Unhandled rotation convention" };
        }

        throw std::invalid_argument("Invalid orientation");
    }

    void rollPitchYawListToRotationMatrix(const std::vector<RollPitchYaw> &rpyList)
    {
        Eigen::IOFormat matrixFmt(4, 0, ", ", "\n", "[", "]", "[", "]");
        for(const auto &rotation : rpyList)
        {
            std::cout << "Rotation Matrix from Roll-Pitch-Yaw angles (" << toString(rotation.convention)
                      << "):" << std::endl;
            const auto rotationMatrixFromRollPitchYaw =
                rollPitchYawToRotationMatrix(rotation.rollPitchYaw, rotation.convention);
            std::cout << rotationMatrixFromRollPitchYaw.format(matrixFmt) << std::endl;
        }
    }

    // The following function converts Rotation Matrix to Roll-Pitch-Yaw angles in radians.
    // Whether the axes are moving (intrinsic) or fixed (extrinsic) is defined by the rotation convention.
    // The order of elements in the output array follow the convention, i.e., for ZYX and zyx, the first
    // element of the array is rotation around z-axis, for XYZ and xyz, the first element is rotation
    // around x-axis.
    // Eigen only supports intrinsic Euler systems for simplicity. If you want to use extrinsic Euler systems,
    // Eigen recoments just to use the equal intrinsic opposite order for axes and angles, i.e., axes (A,B,C)
    // becomes (C,B,A), and angles (a,b,c) becomes (c,b,a).
    Eigen::Array3f rotationMatrixToRollPitchYaw(
        const Eigen::Matrix3f &rotationMatrix,
        const RotationConvention &convention)
    {
        switch(convention)
        {
            case RotationConvention::zyxExtrinsic: return rotationMatrix.eulerAngles(0, 1, 2).reverse();
            case RotationConvention::xyzIntrinsic: return rotationMatrix.eulerAngles(0, 1, 2);
            case RotationConvention::xyzExtrinsic: return rotationMatrix.eulerAngles(2, 1, 0).reverse();
            case RotationConvention::zyxIntrinsic: return rotationMatrix.eulerAngles(2, 1, 0);
            case RotationConvention::nofROT: break;
            default: throw std::runtime_error{ "Unhandled rotation convention" };
        }

        throw std::invalid_argument("Invalid rotation");
    }

    std::vector<RollPitchYaw> rotationMatrixToRollPitchYawList(const Eigen::Matrix3f &rotationMatrix)
    {
        const Eigen::IOFormat vectorFmt(4, 0, ", ", "", "", "", "[", "]");
        std::vector<RollPitchYaw> rpyList;
        for(size_t i = 0; i < nofRotationConventions; i++)
        {
            RotationConvention convention{ static_cast<RotationConvention>(i) };
            std::cout << "Roll-Pitch-Yaw angles (" << toString(convention) << "):" << std::endl;
            rpyList.push_back({ convention, rotationMatrixToRollPitchYaw(rotationMatrix, convention) });
            std::cout << rpyList[i].rollPitchYaw.format(vectorFmt) << std::endl;
        }
        return rpyList;
    }

    Eigen::Vector3f rotationMatrixToRotationVector(const Eigen::Matrix3f &rotationMatrix)
    {
        const Eigen::AngleAxisf axisAngle(rotationMatrix);
        return axisAngle.angle() * axisAngle.axis();
    }

    Eigen::Matrix3f rotationVectorToRotationMatrix(const Eigen::Vector3f &rotationVector)
    {
        Eigen::AngleAxisf axisAngle(rotationVector.norm(), rotationVector.normalized());
        return axisAngle.toRotationMatrix();
    }

    void printHeader(const std::string &txt)
    {
        const std::string asterixLine = "****************************************************************";
        std::cout << asterixLine << "\n* " << txt << std::endl << asterixLine << std::endl;
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
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << std::setprecision(4);
        Eigen::IOFormat matrixFormatRules(4, 0, ", ", "\n", "[", "]", "[", "]");
        Eigen::IOFormat vectorFormatRules(4, 0, ", ", "", "", "", "[", "]");
        printHeader("This example shows conversions to/from Transformation Matrix");

        Zivid::Matrix4x4 transformationMatrixZivid(std::string(ZIVID_SAMPLE_DATA_DIR) + "/RobotTransform.yaml");
        const Eigen::Affine3f transformationMatrix = zividToEigen(transformationMatrixZivid);
        std::cout << transformationMatrix.matrix().format(matrixFormatRules) << std::endl;

        // Extract Rotation Matrix and Translation Vector from Transformation Matrix
        const Eigen::Matrix3f rotationMatrix = transformationMatrix.linear();
        const Eigen::Vector3f translationVector = transformationMatrix.translation();
        std::cout << "RotationMatrix:\n" << rotationMatrix.format(matrixFormatRules) << std::endl;
        std::cout << "TranslationVector:\n" << translationVector.format(vectorFormatRules) << std::endl;

        /*
         * Convert from Rotation Matrix (Zivid) to other representations of orientation (Robot)
         */
        printHeader("Convert from Zivid (Rotation Matrix) to Robot");
        const Eigen::AngleAxisf axisAngle(rotationMatrix);
        std::cout << "AxisAngle:\n"
                  << axisAngle.axis().format(vectorFormatRules) << ", " << axisAngle.angle() << std::endl;
        const Eigen::Vector3f rotationVector = rotationMatrixToRotationVector(rotationMatrix);
        std::cout << "Rotation Vector:\n" << rotationVector.format(vectorFormatRules) << std::endl;
        const Eigen::Quaternionf quaternion(rotationMatrix);
        std::cout << "Quaternion:\n" << quaternion.coeffs().format(vectorFormatRules) << std::endl;
        const auto rpyList = rotationMatrixToRollPitchYawList(rotationMatrix);

        /*
         * Convert to Rotation Matrix (Zivid) from other representations of orientation (Robot)
         */
        printHeader("Convert from Robot to Zivid (Rotation Matrix)");
        const Eigen::Matrix3f rotationMatrixFromAxisAngle = axisAngle.toRotationMatrix();
        std::cout << "Rotation Matrix from Axis Angle:\n"
                  << rotationMatrixFromAxisAngle.format(matrixFormatRules) << std::endl;
        const Eigen::Matrix3f rotationMatrixFromRotationVector = rotationVectorToRotationMatrix(rotationVector);
        std::cout << "Rotation Matrix from Rotation Vector:\n"
                  << rotationMatrixFromRotationVector.format(matrixFormatRules) << std::endl;
        const Eigen::Matrix3f rotationMatrixFromQuaternion = quaternion.toRotationMatrix();
        std::cout << "Rotation Matrix from Quaternion:\n"
                  << rotationMatrixFromQuaternion.format(matrixFormatRules) << std::endl;
        rollPitchYawListToRotationMatrix(rpyList);

        // Combine Rotation Matrix with Translation Vector to form Transformation Matrix
        Eigen::Affine3f transformationMatrixFromQuaternion(rotationMatrixFromQuaternion);
        transformationMatrixFromQuaternion.translation() = translationVector;
        Zivid::Matrix4x4 transformationMatrixFromQuaternionZivid = eigenToZivid(transformationMatrixFromQuaternion);
        transformationMatrixFromQuaternionZivid.save("RobotTransformOut.yaml");
    }

    catch(const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
