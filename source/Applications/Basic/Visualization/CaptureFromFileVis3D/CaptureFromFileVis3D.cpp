// Please make sure that Zivid sample data has been selected during installation of Zivid software.
// Latest version of Zivid software (including samples) can be found at http://zivid.com/software/.

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

        auto zdfFile = "MiscObjects.zdf";

        std::cout << "Initializing camera emulation using file: " << zdfFile << std::endl;
        auto camera = zivid.createFileCamera(zdfFile);

        std::cout << "Capture a frame" << std::endl;
        auto frame = camera.capture();

        std::cout << "Display the frame" << std::endl;
        vis.showMaximized();
        vis.show(frame);
        vis.resetToFit();

        std::cout << "Run the visualizer. Block until window closes" << std::endl;
        vis.run();
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
