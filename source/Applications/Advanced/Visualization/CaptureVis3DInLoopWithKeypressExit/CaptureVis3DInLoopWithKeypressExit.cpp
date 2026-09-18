/*
Capture point clouds, with color, from the Zivid camera, and visualize them in a loop. Press 'q' to exit.
*/

#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <atomic>
#include <exception>
#include <iostream>
#include <thread>
#ifdef _WIN32
#    include <conio.h> // for _kbhit() and _getch()
#else
#    include <poll.h>
#    include <termios.h>
#    include <unistd.h>
#endif

namespace
{
    int getKeyNonBlocking()
    {
#ifdef _WIN32
        if(_kbhit()) return _getch();
        return -1;
#else
        // Use poll() to check if input is available without blocking
        // Use termios to set terminal to raw mode for single character input
        static bool initialized = false;
        static struct termios oldt;
        static struct termios newt;
        if(!initialized)
        {
            tcgetattr(STDIN_FILENO, &oldt);
            newt = oldt;
            newt.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &newt);
            initialized = true;
        }

        struct pollfd pfd = { STDIN_FILENO, POLLIN, 0 };
        int pollResult = poll(&pfd, 1, 0);
        if(pollResult > 0)
        {
            if((pfd.revents & POLLIN) != 0)
            {
                int ch = getchar();
                return ch;
            }
        }

        return -1;
#endif
    }
} // namespace

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
                std::cout << "Press 'q' in the terminal to quit " << std::endl;
                while(visualizerRunning)
                {
                    if(getKeyNonBlocking() == 'q')
                    {
                        std::cout << "Closing application because user pressed 'q'" << std::endl;
                        visualizer.close();
                        break;
                    }
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

        std::cout << "Running visualizer. Blocking until the window closes or 'q' is pressed." << std::endl;
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
