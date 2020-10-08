/*
This example shows how to downsample point cloud from ZDF file. The ZDF file for this sample can be found under the
main instructions for Zivid samples.
*/

#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <iostream>

namespace
{
    void visualizePointCloud(const Zivid::PointCloud &pointCloud)
    {
        std::cout << "Setting up visualization" << std::endl;
        Zivid::Visualization::Visualizer visualizer;
        std::cout << "Visualizing point cloud" << std::endl;
        visualizer.showMaximized();
        visualizer.show(pointCloud);
        visualizer.resetToFit();

        std::cout << "Running visualizer. Blocking until window closes" << std::endl;
        visualizer.run();
    }

} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::string dataFile = std::string(ZIVID_SAMPLE_DATA_DIR) + "/Zivid3D.zdf";
        std::cout << "Reading ZDF frame from file: " << dataFile << std::endl;
        const auto frame = Zivid::Frame(dataFile);

        auto pointCloud = frame.pointCloud();
        std::cout << "Before downsampling: " << pointCloud.size() << " data points" << std::endl;

        visualizePointCloud(pointCloud);

        std::cout << "Downsampling point cloud" << std::endl;
        pointCloud.downsample(Zivid::PointCloud::Downsampling::by2x2);

        std::cout << "After downsampling: " << pointCloud.size() << " data points" << std::endl;

        visualizePointCloud(pointCloud);
    }

    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        if(std::cin.get() == '\n')
        {
            return EXIT_FAILURE;
        }
    }
}
