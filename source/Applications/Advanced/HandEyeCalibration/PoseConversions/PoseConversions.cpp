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

#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

#include <iomanip>
#include <iostream>

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
        Eigen::Array3d rollPitchYaw;
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
        }

        throw std::invalid_argument("Invalid RotationConvention");
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
            case RotationConvention::xyzIntrinsic:
            case RotationConvention::zyxExtrinsic:
                return (Eigen::AngleAxisd(rollPitchYaw[0], Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(rollPitchYaw[1], Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(rollPitchYaw[2], Eigen::Vector3d::UnitZ()))
                    .matrix();
            case RotationConvention::zyxIntrinsic:
            case RotationConvention::xyzExtrinsic:
                return (Eigen::AngleAxisd(rollPitchYaw[2], Eigen::Vector3d::UnitZ())
                        * Eigen::AngleAxisd(rollPitchYaw[1], Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(rollPitchYaw[0], Eigen::Vector3d::UnitX()))
                    .matrix();
            case RotationConvention::nofROT: break;
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
    // The rotation convention we use here is that Roll is a rotation about x-axis,
    // Pitch is a rotation about y-axis and Yaw is a rotation about z-axis.
    // Whether the axes are moving (intrinsic) or fixed (extrinsic) is defined by the rotation convention.
    // The array is ordered by Roll, Pitch and then Yaw.
    Eigen::Array3d rotationMatrixToRollPitchYaw(
        const Eigen::Matrix3d &rotationMatrix,
        const RotationConvention &convention)
    {
        switch(convention)
        {
            case RotationConvention::zyxExtrinsic:
            case RotationConvention::xyzIntrinsic: return rotationMatrix.eulerAngles(0, 1, 2);
            case RotationConvention::xyzExtrinsic:
            case RotationConvention::zyxIntrinsic: return rotationMatrix.eulerAngles(2, 1, 0).reverse();
            case RotationConvention::nofROT: break;
        }

        throw std::invalid_argument("Invalid rotation");
    }

    std::vector<RollPitchYaw> rotationMatrixToRollPitchYawList(const Eigen::Matrix3d &rotationMatrix)
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

    Eigen::MatrixXd cvToEigen(const cv::Mat &cvMat)
    {
        Eigen::MatrixXd eigenMat(cvMat.rows, cvMat.cols);

        cv::cv2eigen(cvMat, eigenMat);

        return eigenMat;
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

    cv::Mat eigenToCv(const Eigen::MatrixXd &eigenMat)
    {
        cv::Mat cvMat(static_cast<int>(eigenMat.rows()), static_cast<int>(eigenMat.cols()), CV_64FC1, cv::Scalar(0));

        cv::eigen2cv(eigenMat, cvMat);

        return cvMat;
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

    void printHeader(const std::string &txt)
    {
        const std::string asterixLine = "****************************************************************";
        std::cout << asterixLine << "\n* " << txt << std::endl << asterixLine << std::endl;
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

        const auto transformationMatrix =
            getTransformationMatrixFromYAML(std::string(ZIVID_SAMPLE_DATA_DIR) + "/RobotTransform.yaml");
        std::cout << transformationMatrix.matrix().format(matrixFormatRules) << std::endl;

        // Extract Rotation Matrix and Translation Vector from Transformation Matrix
        std::cout << "RotationMatrix:\n" << transformationMatrix.linear().format(matrixFormatRules) << std::endl;
        std::cout << "TranslationVector:\n"
                  << transformationMatrix.translation().format(vectorFormatRules) << std::endl;

        /*
         * Convert from Rotation Matrix (Zivid) to other representations of orientation (Robot)
         */
        printHeader("Convert from Zivid (Rotation Matrix) to Robot");
        const Eigen::AngleAxisd axisAngle(transformationMatrix.linear());
        std::cout << "AxisAngle:\n"
                  << axisAngle.axis().format(vectorFormatRules) << ", " << axisAngle.angle() << std::endl;
        const auto rotationVector = rotationMatrixToRotationVector(transformationMatrix.linear());
        std::cout << "Rotation Vector:\n" << rotationVector.format(vectorFormatRules) << std::endl;
        const Eigen::Quaterniond quaternion(transformationMatrix.linear());
        std::cout << "Quaternion:\n" << quaternion.coeffs().format(vectorFormatRules) << std::endl;
        const auto rpyList = rotationMatrixToRollPitchYawList(transformationMatrix.linear());

        /*
         * Convert to Rotation Matrix (Zivid) from other representations of orientation (Robot)
         */
        printHeader("Convert from Robot to Zivid (Rotation Matrix)");
        const auto rotationMatrixFromAxisAngle = axisAngle.toRotationMatrix();
        std::cout << "Rotation Matrix from Axis Angle:\n"
                  << rotationMatrixFromAxisAngle.format(matrixFormatRules) << std::endl;
        const auto rotationMatrixFromRotationVector = rotationVectorToRotationMatrix(rotationVector);
        std::cout << "Rotation Matrix from Rotation Vector:\n"
                  << rotationMatrixFromRotationVector.format(matrixFormatRules) << std::endl;
        const auto rotationMatrixFromQuaternion = quaternion.toRotationMatrix();
        std::cout << "Rotation Matrix from Quaternion:\n"
                  << rotationMatrixFromQuaternion.format(matrixFormatRules) << std::endl;
        rollPitchYawListToRotationMatrix(rpyList);

        // Combine Rotation Matrix with Translation Vector to form Transformation Matrix
        Eigen::Affine3d transformationMatrixFromQuaternion(rotationMatrixFromQuaternion);
        transformationMatrixFromQuaternion.translation() = transformationMatrix.translation();
        saveTransformationMatrixToYAML(transformationMatrixFromQuaternion, "RobotTransformOut.yaml");
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
