/*
Poll the camera health check from a separate thread while capturing, printing statuses and values each second.
*/

#include <Zivid/Zivid.h>

#include <chrono>
#include <future>
#include <iostream>
#include <thread>

namespace
{
    void printHealthcheck(const Zivid::CameraHealth &health)
    {
        const auto &temperature = health.temperature();
        std::cout << "Overall:                " << health.overall() << std::endl;
        std::cout << "  Max transfer speed:   " << health.maxTransferSpeed().status() << " ("
                  << health.maxTransferSpeed().value() << " Mbps)" << std::endl;
        std::cout << "  Temperature (DMD):    " << temperature.dmd().status() << " (" << temperature.dmd().value()
                  << " C)" << std::endl;
        std::cout << "  Temperature (LED):    " << temperature.led().status() << " (" << temperature.led().value()
                  << " C)" << std::endl;
        std::cout << "  Temperature (Lens):   " << temperature.lens().status() << " (" << temperature.lens().value()
                  << " C)" << std::endl;
        std::cout << "  Fan:                  " << health.fan().status() << " (" << health.fan().value() << ")"
                  << std::endl;
        std::cout << "  Memory:               " << health.memory().status() << " (" << health.memory().value()
                  << " errors)" << std::endl;
        std::cout << "  Infield verification: " << health.infieldVerification().status() << " ("
                  << health.infieldVerification().value() << ")" << std::endl;
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        const auto pollInterval = std::chrono::seconds(1);
        std::promise<void> stop;
        auto stopRequested = stop.get_future();

        auto polling = std::async(std::launch::async, [&] {
            do
            {
                printHealthcheck(camera.checkHealth());
                std::cout << std::endl;
            } while(stopRequested.wait_for(pollInterval) == std::future_status::timeout);
        });

        const auto settings = Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} } };
        const auto captureCycle = std::chrono::seconds(5);
        const size_t numberOfCaptures = 5;

        for(size_t i = 1; i <= numberOfCaptures; ++i)
        {
            const auto frame = camera.capture3D(settings);
            std::cout << "Captured frame " << i << " of " << numberOfCaptures << std::endl;
            if(i < numberOfCaptures)
            {
                std::this_thread::sleep_for(captureCycle);
            }
        }

        stop.set_value();
        polling.get();
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
