/*
This example shows how to perform Hand-Eye calibration.
*/

#include <Zivid/Application.h>
#include <Zivid/Calibration/Detector.h>
#include <Zivid/Calibration/HandEye.h>
#include <Zivid/Calibration/Pose.h>
#include <Zivid/Exception.h>
#include <Zivid/Zivid.h>

#include <iostream>

namespace
{
    enum class CommandType
    {
        cmdAddPose,
        cmdCalibrate,
        cmdUnknown
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
            return CommandType::cmdAddPose;
        }
        if(command == "C" || command == "c")
        {
            return CommandType::cmdCalibrate;
        }
        return CommandType::cmdUnknown;
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
        return camera.capture(settings);
    }

    Zivid::Calibration::HandEyeOutput performCalibration(const std::vector<Zivid::Calibration::HandEyeInput> &input)
    {
        while(true)
        {
            std::cout << "Enter type of calibration, eth (for eye-to-hand) or eih (for eye-in-hand): ";
            const auto calibrationType = getInput();
            if(calibrationType == "eth" || calibrationType == "ETH")
            {
                std::cout << "Performing eye-to-hand calibration" << std::endl;
                return Zivid::Calibration::calibrateEyeToHand(input);
            }
            if(calibrationType == "eih" || calibrationType == "EIH")
            {
                std::cout << "Performing eye-in-hand calibration" << std::endl;
                return Zivid::Calibration::calibrateEyeInHand(input);
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

        size_t currPoseId{ 0 };
        bool calibrate{ false };
        std::vector<Zivid::Calibration::HandEyeInput> input;
        do
        {
            switch(enterCommand())
            {
                case CommandType::cmdAddPose:
                {
                    try
                    {
                        const auto robotPose = enterRobotPose(currPoseId);

                        const auto frame = assistedCapture(camera);

                        std::cout << "Detecting checkerboard in point cloud" << std::endl;
                        const auto result = Zivid::Calibration::detectFeaturePoints(frame.pointCloud());
                        if(result)
                        {
                            std::cout << "OK" << std::endl;
                            input.emplace_back(Zivid::Calibration::HandEyeInput{ robotPose, result });
                            currPoseId++;
                        }
                        else
                        {
                            std::cout << "FAILED" << std::endl;
                        }
                    }
                    catch(const std::exception &e)
                    {
                        std::cout << "Error: " << Zivid::toString(e) << std::endl;
                        continue;
                    }
                    break;
                }
                case CommandType::cmdCalibrate:
                {
                    calibrate = true;
                    break;
                }
                case CommandType::cmdUnknown:
                {
                    std::cout << "Error: Unknown command" << std::endl;
                    break;
                }
            }
        } while(!calibrate);

        const auto calibrationResult{ performCalibration(input) };

        if(calibrationResult.valid())
        {
            std::cout << "Hand-eye calibration OK\n"
                      << "Result:\n"
                      << calibrationResult << std::endl;
        }
        else
        {
            std::cout << "Hand-eye calibration FAILED" << std::endl;
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
