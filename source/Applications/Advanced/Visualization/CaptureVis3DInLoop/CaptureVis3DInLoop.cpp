/*
Capture point clouds, with color, from the Zivid camera, and visualize them in a loop.
*/

#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <future>
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
        auto frame = camera.capture2D3D(settings);
        std::cout << "Settings:" << frame.settings() << std::endl;

        std::cout << "Setting up visualization" << std::endl;
        std::atomic_bool visualizerRunning{ false };
        auto visualizerPromise = std::promise<Zivid::Visualization::Visualizer *>();
        auto visualizerFuture = visualizerPromise.get_future();

        std::thread visualizationThread([&frame, &visualizerPromise, &visualizerRunning]() {
            auto visualizer = Zivid::Visualization::Visualizer();

            // Pass the visualizer to the main thread
            visualizerPromise.set_value(&visualizer);

            std::cout << "Visualizing point cloud" << std::endl;
            visualizer.showMaximized();
            visualizer.show(frame);
            visualizer.resetToFit();

            std::cout << "Running visualizer. Blocking until window closes." << std::endl;
            visualizerRunning = true;
            visualizer.run();
            visualizerRunning = false;
        });

        // Get the visualizer handle in the main thread
        auto visualizerHandle = visualizerFuture.get();

        while(!visualizerRunning)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        while(visualizerRunning)
        {
            frame = camera.capture2D3D(settings);
            if(!visualizerRunning)
            {
                break;
            }
            visualizerHandle->show(frame);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        visualizationThread.join();
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
