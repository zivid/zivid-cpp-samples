#include <Zivid/Application.h>
#include <Zivid/Exception.h>
#include <Zivid/HandEye/Calibrate.h>
#include <Zivid/HandEye/Detector.h>
#include <Zivid/HandEye/Pose.h>

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

    Zivid::HandEye::Pose enterRobotPose(size_t index)
    {
        std::cout << "Enter pose with id (a line with 16 space separated values describing 4x4 row-major matrix) : "
                  << index << std::endl;
        std::stringstream input(getInput());
        double element{ 0 };
        std::vector<double> transformElements;
        for(size_t i = 0; i < 16 && input >> element; ++i)
        {
            transformElements.emplace_back(element);
        }

        const auto robotPose{ Zivid::Matrix4d{ transformElements.cbegin(), transformElements.cend() } };
        std::cout << "The following pose was entered: \n" << robotPose << std::endl;

        return robotPose;
    }

    Zivid::Frame acquireCheckerboardFrame(Zivid::Camera &camera)
    {
        std::cout << "Capturing checkerboard image... " << std::flush;
        auto settings{ Zivid::Settings{} };
        settings.set(Zivid::Settings::Iris{ 17 });
        settings.set(Zivid::Settings::Gain{ 1.0 });
        settings.set(Zivid::Settings::Brightness{ 1.0 });
        settings.set(Zivid::Settings::ExposureTime{ std::chrono::microseconds{ 20000 } });
        settings.set(Zivid::Settings::Filters::Gaussian::Enabled::yes);
        camera.setSettings(settings);
        auto frame = camera.capture();
        std::cout << "OK" << std::endl;

        return frame;
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera..." << std::endl;
        auto camera{ zivid.connectCamera() };

        size_t currPoseId{ 0 };
        bool calibrate{ false };
        std::vector<Zivid::HandEye::CalibrationInput> input;
        do
        {
            switch(enterCommand())
            {
                case CommandType::cmdAddPose:
                {
                    try
                    {
                        const auto robotPose = enterRobotPose(currPoseId);

                        const auto frame = acquireCheckerboardFrame(camera);

                        std::cout << "Detecting checkerboard square centers... " << std::flush;
                        const auto result = Zivid::HandEye::detectFeaturePoints(frame.getPointCloud());
                        if(result)
                        {
                            std::cout << "OK" << std::endl;
                            input.emplace_back(Zivid::HandEye::CalibrationInput{ robotPose, result });
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

        std::cout << "Performing hand-eye calibration ... " << std::flush;
        const auto calibrationResult{ Zivid::HandEye::calibrateEyeToHand(input) };
        if(calibrationResult)
        {
            std::cout << "OK\n"
                      << "Result:\n"
                      << calibrationResult << std::endl;
        }
        else
        {
            std::cerr << "\nFAILED" << std::endl;
            return EXIT_FAILURE;
        }
    }
    catch(const std::exception &e)
    {
        std::cerr << "\nError: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
