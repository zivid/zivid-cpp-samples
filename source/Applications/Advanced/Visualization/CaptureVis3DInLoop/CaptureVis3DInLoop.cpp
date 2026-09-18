/*
Capture point clouds, with color, from the Zivid camera, and visualize them in a loop.
*/

#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <atomic>
#include <exception>
#include <iostream>
#include <thread>

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Creating default settings" << std::endl;
        const auto settings =
            Zivid::Settings{ Zivid::Settings::Engine::phase,
                             Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} },
                             Zivid::Settings::Color{ Zivid::Settings2D{
                                 Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } } } };

        std::cout << "Capturing frame" << std::endl;
        const auto frame = camera.capture2D3D(settings);
        std::cout << "Settings:" << frame.settings() << std::endl;

        std::cout << "Setting up visualization" << std::endl;
        auto visualizer = Zivid::Visualization::Visualizer();

        std::cout << "Visualizing point cloud" << std::endl;
        visualizer.showMaximized();
        visualizer.show(frame);
        visualizer.resetToFit();

        std::atomic_bool visualizerRunning{ true };
        std::exception_ptr captureException;

        std::thread captureThread([&camera, &settings, &visualizer, &visualizerRunning, &captureException]() {
            try
            {
                while(visualizerRunning)
                {
                    const auto newFrame = camera.capture2D3D(settings);
                    if(visualizerRunning)
                    {
                        visualizer.show(newFrame);
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
            }
            catch(...)
            {
                captureException = std::current_exception();
                visualizer.close();
            }
        });

        std::cout << "Running visualizer. Blocking until window closes." << std::endl;
        visualizer.run();
        visualizerRunning = false;
        captureThread.join();
        if(captureException)
        {
            std::rethrow_exception(captureException);
        }
        std::cout << "Visualizer closed" << std::endl;
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
