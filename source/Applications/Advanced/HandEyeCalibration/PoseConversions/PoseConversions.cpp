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

struct Representations
{
    Eigen::AngleAxisd axisAngle;
    Eigen::Vector3d rotationVector;
    Eigen::Quaterniond quaternion;
    std::array<RollPitchYaw, nofRotationConventions> rotations;
};

Eigen::Affine3d getTransformationMatrixFromYAML(const std::string &path);
void saveTransformationMatrixToYAML(const Eigen::Affine3d &, const std::string &path);
cv::Mat eigenToCv(const Eigen::MatrixXd &);
Eigen::MatrixXd cvToEigen(const cv::Mat &);
Eigen::Array3d rotationMatrixToRollPitchYaw(const Eigen::Matrix3d &rotationMatrix, const RotationConvention &rotation);
Eigen::Matrix3d rollPitchYawToRotationMatrix(const Eigen::Array3d &rollPitchYaw, const RotationConvention &rotation);
std::string toString(RotationConvention convention);
Representations zividToRobot(const Eigen::Affine3d &);
Eigen::Affine3d robotToZivid(const Representations &, const Eigen::Vector3d &);

int main()
{
    try
    {
        std::cout << std::setprecision(4);
        std::cout << "This example shows conversions to/from Transformation Matrix" << std::endl;

        const auto transformationMatrix = getTransformationMatrixFromYAML("robotTransform.yaml");
        std::cout << transformationMatrix.matrix() << std::endl;

        // Extract Rotation Matrix and Translation Vector from Transformation Matrix
        std::cout << "RotationMatrix:\n" << transformationMatrix.linear() << std::endl;
        std::cout << "TranslationVector:\n" << transformationMatrix.translation() << std::endl;

        // Convert from Zivid to Robot (Transformation Matrix --> any format)
        const auto robotRotationRepresentations = zividToRobot(transformationMatrix);

        // Convert from Robot to Zivid (any format --> Rotation Matrix)
        const auto transformationMatrix2 =
            robotToZivid(robotRotationRepresentations, transformationMatrix.translation());

        // Combine Rotation Matrix with Translation Vector to form Transformation Matrix
        saveTransformationMatrixToYAML(transformationMatrix2, "robotTransformOut.yaml");
    }

    catch(const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}

Eigen::Affine3d getTransformationMatrixFromYAML(const std::string &path)
{
    std::cout << "Opening .YAML file which contains transformation matrix (PoseState node)" << std::endl;
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

Representations zividToRobot(const Eigen::Affine3d &transformationMatrix)
{
    Representations robotRepresentations;
    std::cout << "\nConverting Rotation Matrix to Axis-Angle" << std::endl;
    const Eigen::AngleAxisd axisAngle(transformationMatrix.linear());
    std::cout << "Axis:\n" << axisAngle.axis() << std::endl;
    std::cout << "Angle:\n" << axisAngle.angle() << std::endl;

    // Axis-angle to Rotation Vector
    std::cout << "\nConverting Axis-Angle to Rotation Vector:" << std::endl;
    robotRepresentations.rotationVector = axisAngle.angle() * axisAngle.axis();
    std::cout << robotRepresentations.rotationVector << std::endl;

    // Rotation Matrix to Quaternion
    std::cout << "\nConverting Rotation Matrix to Quaternion:" << std::endl;
    const Eigen::Quaterniond quaternion(transformationMatrix.linear());
    robotRepresentations.quaternion = quaternion;
    std::cout << robotRepresentations.quaternion.coeffs() << std::endl;

    // Rotation Matrix to Roll-Pitch-Yaw
    for(int i = 0; i < nofRotationConventions; i++)
    {
        const RotationConvention convention{ static_cast<RotationConvention>(i) };
        std::cout << "\nConverting Rotation Matrix to Roll-Pitch-Yaw angles (" << toString(convention)
                  << "):" << std::endl;
        robotRepresentations.rotations[i] = { convention,
                                              rotationMatrixToRollPitchYaw(transformationMatrix.linear(), convention) };
        std::cout << robotRepresentations.rotations[i].rollPitchYaw << std::endl;
    }

    return robotRepresentations;
}

Eigen::Affine3d robotToZivid(const Representations &representations, const Eigen::Vector3d &translationVector)
{
    // Roll-Pitch-Yaw to Rotation Matrix
    for(const auto &rotation : representations.rotations)
    {
        std::cout << "\nConverting Roll-Pitch-Yaw angles (" << toString(rotation.convention)
                  << ") to Rotation Matrix:" << std::endl;
        const Eigen::Matrix3d rotationMatrixFromRollPitchYaw =
            rollPitchYawToRotationMatrix(rotation.rollPitchYaw, rotation.convention);
        std::cout << rotationMatrixFromRollPitchYaw << std::endl;
    }

    // Rotation Vector to Axis-angle
    std::cout << "\nConverting Rotation Vector to Axis-Angle" << std::endl;
    const Eigen::AngleAxisd axisAngle(representations.rotationVector.norm(),
                                      representations.rotationVector.normalized());
    std::cout << "Axis:\n" << axisAngle.axis() << std::endl;
    std::cout << "Angle:\n" << axisAngle.angle() << std::endl;

    // Axis-Angle to Quaternion
    std::cout << "\nConverting Axis-Angle to Quaternion:" << std::endl;
    const Eigen::Quaterniond quaternion(axisAngle);
    std::cout << quaternion.coeffs() << std::endl;

    // Quaternion to Rotation Matrix
    std::cout << "\nConverting Quaternion to Rotation Matrix:" << std::endl;
    const auto rotationMatrixFromQuaternion = quaternion.toRotationMatrix();
    std::cout << rotationMatrixFromQuaternion << std::endl;

    Eigen::Affine3d transformationMatrix(rotationMatrixFromQuaternion);
    transformationMatrix.translation() = translationVector;

    return transformationMatrix;
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
