/*
Perform Hand-Eye calibration.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.
*/

#include <Zivid/Application.h>
#include <Zivid/Calibration/Detector.h>
#include <Zivid/Calibration/HandEye.h>
#include <Zivid/Calibration/Pose.h>
#include <Zivid/Exception.h>
#include <Zivid/Experimental/Calibration/InfieldCorrection.h>
#include <Zivid/Zivid.h>

#include <iostream>

namespace
{
    enum class CommandType
    {
        AddPose,
        Calibrate,
        Unknown
    };

    std::string getInput()
    {
        std::string command;
        std::getline(std::cin, command);
        return command;
    }

    CommandType enterCommand()
    {
        std::cout << "Enter command, p (to add robot pose) or c (to perform calibration):";
        const auto command = getInput();

        if(command == "P" || command == "p")
        {
            return CommandType::AddPose;
        }
        if(command == "C" || command == "c")
        {
            return CommandType::Calibrate;
        }
        return CommandType::Unknown;
    }

    Zivid::Calibration::Pose enterRobotPose(size_t index)
    {
        std::cout << "Enter pose with id (a line with 16 space separated values describing 4x4 row-major matrix) : "
                  << index << std::endl;
        std::stringstream input(getInput());
        float element{ 0 };
        std::vector<float> transformElements;
        for(size_t i = 0; i < 16 && input >> element; ++i)
        {
            transformElements.emplace_back(element);
        }

        const auto robotPose{ Zivid::Matrix4x4{ transformElements.cbegin(), transformElements.cend() } };
        std::cout << "The following pose was entered: \n" << robotPose << std::endl;

        return robotPose;
    }

    Zivid::Calibration::HandEyeOutput performCalibration(
        const std::vector<Zivid::Calibration::HandEyeInput> &handEyeInput)
    {
        while(true)
        {
            std::cout << "Enter type of calibration, eth (for eye-to-hand) or eih (for eye-in-hand): ";
            const auto calibrationType = getInput();
            if(calibrationType == "eth" || calibrationType == "ETH")
            {
                std::cout << "Performing eye-to-hand calibration" << std::endl;
                std::cout << "The resulting transform is the camera pose in robot base frame" << std::endl;
                return Zivid::Calibration::calibrateEyeToHand(handEyeInput);
            }
            if(calibrationType == "eih" || calibrationType == "EIH")
            {
                std::cout << "Performing eye-in-hand calibration" << std::endl;
                std::cout << "The resulting transform is the camera pose in flange (end-effector) frame" << std::endl;
                return Zivid::Calibration::calibrateEyeInHand(handEyeInput);
            }
            std::cout << "Entered uknown method" << std::endl;
        }
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera{ zivid.connectCamera() };

        size_t currentPoseId{ 0 };
        bool calibrate{ false };

        std::cout << "Zivid primarily operates with a (4x4) transformation matrix. To convert" << std::endl;
        std::cout << "from axis-angle, rotation vector, roll-pitch-yaw, or quaternion, check out" << std::endl;
        std::cout << "our PoseConversions sample." << std::endl;

        std::vector<Zivid::Calibration::HandEyeInput> handEyeInput;
        do
        {
            switch(enterCommand())
            {
                case CommandType::AddPose:
                {
                    try
                    {
                        const auto robotPose = enterRobotPose(currentPoseId);

                        const auto frame = Zivid::Experimental::Calibration::captureCalibrationBoard(camera);

                        std::cout << "Detecting checkerboard in point cloud" << std::endl;
                        const auto detectionResult = Zivid::Calibration::detectFeaturePoints(frame.pointCloud());

                        if(detectionResult.valid())
                        {
                            std::cout << "Calibration board detected " << std::endl;
                            handEyeInput.emplace_back(Zivid::Calibration::HandEyeInput{ robotPose, detectionResult });
                            currentPoseId++;
                        }
                        else
                        {
                            std::cout
                                << "Failed to detect calibration board, ensure that the entire board is in the view of the camera"
                                << std::endl;
                        }
                    }
                    catch(const std::exception &e)
                    {
                        std::cout << "Error: " << Zivid::toString(e) << std::endl;
                        continue;
                    }
                    break;
                }
                case CommandType::Calibrate:
                {
                    calibrate = true;
                    break;
                }
                case CommandType::Unknown:
                {
                    std::cout << "Error: Unknown command" << std::endl;
                    break;
                }
            }
        } while(!calibrate);

        const auto calibrationResult{ performCalibration(handEyeInput) };

        std::cout << "Zivid primarily operates with a (4x4) transformation matrix. To convert" << std::endl;
        std::cout << "to axis-angle, rotation vector, roll-pitch-yaw, or quaternion, check out" << std::endl;
        std::cout << "our PoseConversions sample." << std::endl;

        if(calibrationResult.valid())
        {
            std::cout << "Hand-Eye calibration OK\n"
                      << "Result:\n"
                      << calibrationResult << std::endl;
        }
        else
        {
            std::cout << "Hand-Eye calibration FAILED" << std::endl;
            return EXIT_FAILURE;
        }
    }
    catch(const std::exception &e)
    {
        std::cerr << "\nError: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

