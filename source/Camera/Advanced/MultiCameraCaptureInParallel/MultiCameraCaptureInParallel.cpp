/*
Capture point clouds with multiple cameras in parallel.
*/

#include <Zivid/Zivid.h>

#include <chrono>
#include <future>
#include <iostream>
#include <mutex>
#include <vector>

namespace
{
    void captureInThread(Zivid::Camera &camera, std::mutex &m)
    {
        const auto parameters = Zivid::CaptureAssistant::SuggestSettingsParameters{
            Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency::none,
            Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{ std::chrono::milliseconds{ 800 } }
        };
        const auto settings = Zivid::CaptureAssistant::suggestSettings(camera, parameters);

        // Capturing frame
        const auto frame = camera.capture(settings);

        // Saving frame to file
        std::unique_lock<std::mutex> lock(m);
        frame.save("Frame_" + camera.info().serialNumber().value() + ".zdf");
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Finding cameras" << std::endl;
        auto cameras = zivid.cameras();
        std::cout << "Number of cameras found: " << cameras.size() << std::endl;

        for(auto &camera : cameras)
        {
            std::cout << "Connecting to camera: " << camera.info().serialNumber().value() << std::endl;
            camera.connect();
        }

        std::cout << "Create one task for each camera, capture frame and save to file" << std::endl;
        std::vector<std::future<void>> container;
        std::mutex fileSaveMutex;

        for(auto &camera : cameras)
        {
            std::cout << "Starting task with camera: " << camera.info().serialNumber().value() << std::endl;
            container.emplace_back(
                std::async(std::launch::async, captureInThread, std::ref(camera), std::ref(fileSaveMutex)));
        }

        for(size_t i = 0; i < cameras.size(); ++i)
        {
            std::cout << "Wait for task " << i << " to finish" << std::endl;
            container[i].get();
        }
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }
}
