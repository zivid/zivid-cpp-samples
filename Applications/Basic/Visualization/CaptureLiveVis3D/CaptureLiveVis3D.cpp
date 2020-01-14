#include <Zivid/CloudVisualizer.h>
#include <Zivid/Zivid.h>
#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Setting up visualization" << std::endl;
        Zivid::CloudVisualizer vis;
        zivid.setDefaultComputeDevice(vis.computeDevice());

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Start live capturing of frames" << std::endl;
        auto resetToFit = true;
        camera.setFrameCallback([&vis, &resetToFit](Zivid::Frame frame) {
            vis.show(frame);
            if(resetToFit)
            {
                vis.resetToFit();
                resetToFit = false;
            }
        });
        camera.startLive();

        std::cout << "Run the visualizer. Block until window closes" << std::endl;
        vis.run();

        std::cout << "Stopping live capturing" << std::endl;
        camera.stopLive();
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}