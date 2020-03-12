/*
Example to show conversions to/from Transformation Matrix

Zivid primarily operate with a (4x4) Transformation Matrix (Rotation Matrix + Translation Vector).
This example shows how to use Eigen to convert to and from:
  AxisAngle, Rotation Vector, Roll-Pitch-Yaw, Quaternion

 It provides convenience functions that can be reused in applicable applications.
*/

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <iomanip>
#include <iostream>

enum class RotationConvention
{
    ZYX_Intrinsic,
    XYZ_Extrinsic,
    XYZ_Intrinsic,
    ZYX_Extrinsic,
    NOF_ROT
};
constexpr size_t nofRotationConventions = static_cast<size_t>(RotationConvention::NOF_ROT);

struct RollPitchYaw
{
    RotationConvention convention;
    Eigen::Array3d rollPitchYaw;
};

Eigen::Affine3d getTransformationMatrixFromYAML(const std::string &path);
void saveTransformationMatrixToYAML(const Eigen::Affine3d &, const std::string &path);
cv::Mat eigenToCv(const Eigen::MatrixXd &);
Eigen::MatrixXd cvToEigen(const cv::Mat &);
Eigen::Array3d rotationMatrixToRollPitchYaw(const Eigen::Matrix3d &rotationMatrix, const RotationConvention &rotation);
Eigen::Matrix3d rollPitchYawToRotationMatrix(const Eigen::Array3d &rollPitchYaw, const RotationConvention &rotation);
Eigen::Vector3d rotationMatrixToRotationVector(const Eigen::Matrix3d &rotationMatrix);
std::array<RollPitchYaw, nofRotationConventions> rotationMatrixToRollPitchYawList(
    const Eigen::Matrix3d &rotationMatrix);
Eigen::Matrix3d rotationVectorToRotationMatrix(const Eigen::Vector3d &rotationVector);
void rollPitchYawListToRotationMatrix(const std::array<RollPitchYaw, nofRotationConventions> &rpyList);
std::string toString(RotationConvention convention);
void printHeader(const std::string &txt);

int main()
{
    try
    {
        std::cout << std::setprecision(4);
        Eigen::IOFormat MatrixFmt(4, 0, ", ", "\n", "[", "]", "[", "]");
        Eigen::IOFormat VectorFmt(4, 0, ", ", "", "", "", "[", "]");
        printHeader("This example shows conversions to/from Transformation Matrix");

        const auto transformationMatrix = getTransformationMatrixFromYAML("robotTransform.yaml");
        std::cout << transformationMatrix.matrix().format(MatrixFmt) << std::endl;

        // Extract Rotation Matrix and Translation Vector from Transformation Matrix
        std::cout << "RotationMatrix:\n" << transformationMatrix.linear().format(MatrixFmt) << std::endl;
        std::cout << "TranslationVector:\n" << transformationMatrix.translation().format(VectorFmt) << std::endl;

        /*
         * Convert from Rotation Matrix (Zivid) to other representations of orientation (Robot)
         */
        printHeader("Convert from Zivid (Rotation Matrix) to Robot");
        const Eigen::AngleAxisd axisAngle(transformationMatrix.linear());
        std::cout << "AxisAngle:\n" << axisAngle.axis().format(VectorFmt) << ", " << axisAngle.angle() << std::endl;
        const auto rotationVector = rotationMatrixToRotationVector(transformationMatrix.linear());
        std::cout << "Rotation Vector:\n" << rotationVector.format(VectorFmt) << std::endl;
        const Eigen::Quaterniond quaternion(transformationMatrix.linear());
        std::cout << "Quaternion:\n" << quaternion.coeffs().format(VectorFmt) << std::endl;
        const auto rpyList = rotationMatrixToRollPitchYawList(transformationMatrix.linear());

        /*
         * Convert to Rotation Matrix (Zivid) from other representations of orientation (Robot)
         */
        printHeader("Convert from Robot to Zivid (Rotation Matrix)");
        const auto rotationMatrixFromAxisAngle = axisAngle.toRotationMatrix();
        std::cout << "Rotation Matrix from Axis Angle:\n" << rotationMatrixFromAxisAngle.format(MatrixFmt) << std::endl;
        const auto rotationMatrixFromRotationVector = rotationVectorToRotationMatrix(rotationVector);
        std::cout << "Rotation Matrix from Rotation Vector:\n"
                  << rotationMatrixFromRotationVector.format(MatrixFmt) << std::endl;
        const auto rotationMatrixFromQuaternion = quaternion.toRotationMatrix();
        std::cout << "Rotation Matrix from Quaternion:\n"
                  << rotationMatrixFromQuaternion.format(MatrixFmt) << std::endl;
        rollPitchYawListToRotationMatrix(rpyList);

        // Combine Rotation Matrix with Translation Vector to form Transformation Matrix
        Eigen::Affine3d transformationMatrixFromQuaternion(rotationMatrixFromQuaternion);
        transformationMatrixFromQuaternion.translation() = transformationMatrix.translation();
        saveTransformationMatrixToYAML(transformationMatrixFromQuaternion, "robotTransformOut.yaml");
    }

    catch(const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}

void rollPitchYawListToRotationMatrix(const std::array<RollPitchYaw, nofRotationConventions> &rpyList)
{
    Eigen::IOFormat MatrixFmt(4, 0, ", ", "\n", "[", "]", "[", "]");
    for(const auto &rotation : rpyList)
    {
        std::cout << "Rotation Matrix from Roll-Pitch-Yaw angles (" << toString(rotation.convention)
                  << "):" << std::endl;
        const auto rotationMatrixFromRollPitchYaw =
            rollPitchYawToRotationMatrix(rotation.rollPitchYaw, rotation.convention);
        std::cout << rotationMatrixFromRollPitchYaw.format(MatrixFmt) << std::endl;
    }
}

std::array<RollPitchYaw, nofRotationConventions> rotationMatrixToRollPitchYawList(const Eigen::Matrix3d &rotationMatrix)
{
    Eigen::IOFormat VectorFmt(4, 0, ", ", "", "", "", "[", "]");
    std::array<RollPitchYaw, nofRotationConventions> rpyList;
    for(size_t i = 0; i < nofRotationConventions; i++)
    {
        const RotationConvention convention{ static_cast<RotationConvention>(i) };
        std::cout << "Roll-Pitch-Yaw angles (" << toString(convention) << "):" << std::endl;
        rpyList[i] = { convention, rotationMatrixToRollPitchYaw(rotationMatrix, convention) };
        std::cout << rpyList[i].rollPitchYaw.format(VectorFmt) << std::endl;
    }
    return rpyList;
}

Eigen::Affine3d getTransformationMatrixFromYAML(const std::string &path)
{
    cv::FileStorage fileStorageIn;
    if(!fileStorageIn.open(path, cv::FileStorage::Mode::READ))
    {
        throw std::runtime_error("Could not open " + path + ". Please run this sample from the build directory");
    }
    const auto poseStateNode = fileStorageIn["PoseState"];
    std::cout << "Getting PoseState:" << std::endl;
    if(poseStateNode.empty())
    {
        fileStorageIn.release();
        throw std::runtime_error("PoseState node not found in file");
    }
    auto transformationMatrix = Eigen::Affine3d(static_cast<Eigen::Matrix4d>(cvToEigen(poseStateNode.mat())));
    fileStorageIn.release();

    return transformationMatrix;
}

void saveTransformationMatrixToYAML(const Eigen::Affine3d &transformationMatrix, const std::string &path)
{
    // Save Transformation Matrix to .YAML file
    cv::FileStorage fileStorageOut;
    if(!fileStorageOut.open(path, cv::FileStorage::Mode::WRITE))
    {
        throw std::runtime_error("Could not open robotTransformOut.yaml for writing");
    }
    fileStorageOut.write("TransformationMatrixFromQuaternion", eigenToCv(transformationMatrix.matrix()));
    fileStorageOut.release();
}

cv::Mat eigenToCv(const Eigen::MatrixXd &eigenMat)
{
    cv::Mat cvMat(static_cast<int>(eigenMat.rows()), static_cast<int>(eigenMat.cols()), CV_64FC1, cv::Scalar(0));

    cv::eigen2cv(eigenMat, cvMat);

    return cvMat;
}

Eigen::MatrixXd cvToEigen(const cv::Mat &cvMat)
{
    Eigen::MatrixXd eigenMat(cvMat.rows, cvMat.cols);

    cv::cv2eigen(cvMat, eigenMat);

    return eigenMat;
}

Eigen::Vector3d rotationMatrixToRotationVector(const Eigen::Matrix3d &rotationMatrix)
{
    const Eigen::AngleAxisd axisAngle(rotationMatrix);
    return axisAngle.angle() * axisAngle.axis();
}

Eigen::Matrix3d rotationVectorToRotationMatrix(const Eigen::Vector3d &rotationVector)
{
    Eigen::AngleAxisd axisAngle(rotationVector.norm(), rotationVector.normalized());
    return axisAngle.toRotationMatrix();
}

std::string toString(RotationConvention convention)
{
    switch(convention)
    {
        case RotationConvention::XYZ_Intrinsic: return "XYZ_Intrinsic";
        case RotationConvention::XYZ_Extrinsic: return "XYZ_Extrinsic";
        case RotationConvention::ZYX_Intrinsic: return "ZYX_Intrinsic";
        case RotationConvention::ZYX_Extrinsic: return "ZYX_Extrinsic";
        case RotationConvention::NOF_ROT: break;
    }

    throw std::invalid_argument("Invalid RotationConvention");
}

// The following function converts Rotation Matrix to Roll-Pitch-Yaw angles in radians.
// The rotation convention we use here is that Roll is a rotation about x-axis,
// Pitch is a rotation about y-axis and Yaw is a rotation about z-axis.
// Whether the axes are moving (intrinsic) or fixed (extrinsic) is defined by the rotation convention.
// The array is ordered by Roll, Pitch and then Yaw.
Eigen::Array3d rotationMatrixToRollPitchYaw(const Eigen::Matrix3d &rotationMatrix, const RotationConvention &rotation)
{
    switch(rotation)
    {
        case RotationConvention::XYZ_Intrinsic: return rotationMatrix.eulerAngles(0, 1, 2);
        case RotationConvention::XYZ_Extrinsic: return rotationMatrix.eulerAngles(2, 1, 0).reverse();
        case RotationConvention::ZYX_Intrinsic: return rotationMatrix.eulerAngles(2, 1, 0).reverse();
        case RotationConvention::ZYX_Extrinsic: return rotationMatrix.eulerAngles(0, 1, 2);
        case RotationConvention::NOF_ROT: break;
    }

    throw std::invalid_argument("Invalid rotation");
}

// The following function converts Roll-Pitch-Yaw angles in radians to Rotation Matrix.
// This function takes an array of roll, pitch and yaw angles, and a rotation convention, as input parameters.
// For Roll-Pitch-Yaw we define that roll is a rotation about x-axis, pitch is a rotation about y-axis
// and yaw is a rotation about z-axis.
// Whether the axes are moving (intrinsic) or fixed (extrinsic) is defined by the rotation convention.
// The array is ordered by Roll, Pitch and then Yaw.
Eigen::Matrix3d rollPitchYawToRotationMatrix(const Eigen::Array3d &rollPitchYaw, const RotationConvention &rotation)
{
    switch(rotation)
    {
        case RotationConvention::XYZ_Intrinsic:
        case RotationConvention::ZYX_Extrinsic:
            return (Eigen::AngleAxisd(rollPitchYaw[0], Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(rollPitchYaw[1], Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(rollPitchYaw[2], Eigen::Vector3d::UnitZ()))
                .matrix();
        case RotationConvention::ZYX_Intrinsic:
        case RotationConvention::XYZ_Extrinsic:
            return (Eigen::AngleAxisd(rollPitchYaw[2], Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(rollPitchYaw[1], Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(rollPitchYaw[0], Eigen::Vector3d::UnitX()))
                .matrix();
        case RotationConvention::NOF_ROT: break;
    }

    throw std::invalid_argument("Invalid orientation");
}

void printHeader(const std::string &txt)
{
    const std::string asterixLine = "****************************************************************";
    std::cout << asterixLine << "\n* " << txt << std::endl << asterixLine << std::endl;
}