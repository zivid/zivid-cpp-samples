/*
Perform Hand-Eye calibration.
*/

#include <Zivid/Application.h>
#include <Zivid/Calibration/Detector.h>
#include <Zivid/Calibration/HandEye.h>
#include <Zivid/Calibration/Pose.h>
#include <Zivid/Exception.h>
#include <Zivid/Zivid.h>

#include <iostream>
#include <stdexcept>

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
        std::cout << "Enter command, p (to add robot pose) or c (to perform calibration): ";
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

    Zivid::Frame assistedCapture(Zivid::Camera &camera)
    {
        const auto parameters = Zivid::CaptureAssistant::SuggestSettingsParameters{
            Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency::none,
            Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{ std::chrono::milliseconds{ 800 } }
        };
        const auto settings = Zivid::CaptureAssistant::suggestSettings(camera, parameters);
        return camera.capture2D3D(settings);
    }

    std::string markersToString(const std::vector<int> &markerIds)
    {
        std::ostringstream oss;
        for(const auto &id : markerIds)
        {
            oss << id << " ";
        }
        return oss.str();
    }

    void handleAddPose(
        size_t &currentPoseId,
        std::vector<Zivid::Calibration::HandEyeInput> &handEyeInput,
        Zivid::Camera &camera,
        const std::string &calibrationObject)
    {
        const auto robotPose = enterRobotPose(currentPoseId);

        std::cout << "Detecting calibration object in point cloud" << std::endl;
        if(calibrationObject == "c")
        {
            const auto frame = Zivid::Calibration::captureCalibrationBoard(camera);
            const auto detectionResult = Zivid::Calibration::detectCalibrationBoard(frame);

            if(detectionResult.valid())
            {
                std::cout << "Calibration board detected " << std::endl;
                handEyeInput.emplace_back(robotPose, detectionResult);
                currentPoseId++;
            }
            else
            {
                std::cout
                    << "Failed to detect calibration board, ensure that the entire board is in the view of the camera"
                    << std::endl;
            }
        }
        else if(calibrationObject == "m")
        {
            const auto frame = assistedCapture(camera);

            auto markerDictionary = Zivid::Calibration::MarkerDictionary::aruco4x4_50;
            std::vector<int> markerIds = { 1, 2, 3 };

            std::cout << "Detecting arUco marker IDs " << markersToString(markerIds) << "from the dictionary "
                      << markerDictionary << std::endl;
            auto detectionResult = Zivid::Calibration::detectMarkers(frame, markerIds, markerDictionary);

            if(detectionResult.valid())
            {
                std::cout << "ArUco marker(s) detected: " << detectionResult.detectedMarkers().size() << std::endl;
                handEyeInput.emplace_back(robotPose, detectionResult);
                currentPoseId++;
            }
            else
            {
                std::cout
                    << "Failed to detect any ArUco markers, ensure that at least one ArUco marker is in the view of the camera"
                    << std::endl;
            }
        }
    }

    std::vector<Zivid::Calibration::HandEyeInput> readHandEyeInputs(Zivid::Camera &camera)
    {
        size_t currentPoseId{ 0 };
        bool calibrate{ false };

        std::string calibrationObject;
        while(true)
        {
            std::cout
                << "Enter calibration object you are using, m (for ArUco marker(s)) or c (for Zivid checkerboard): "
                << std::endl;
            calibrationObject = getInput();
            if(calibrationObject == "m" || calibrationObject == "c")
            {
                break;
            }
        }

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
                        handleAddPose(currentPoseId, handEyeInput, camera, calibrationObject);
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
                default: throw std::runtime_error{ "Unhandled command type" };
            }
        } while(!calibrate);
        return handEyeInput;
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
                std::cout << "Performing eye-to-hand calibration with " << handEyeInput.size() << " dataset pairs"
                          << std::endl;
                std::cout << "The resulting transform is the camera pose in robot base frame" << std::endl;
                return Zivid::Calibration::calibrateEyeToHand(handEyeInput);
            }
            if(calibrationType == "eih" || calibrationType == "EIH")
            {
                std::cout << "Performing eye-in-hand calibration with " << handEyeInput.size() << " dataset pairs"
                          << std::endl;
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

        const auto handEyeInput{ readHandEyeInputs(camera) };

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

