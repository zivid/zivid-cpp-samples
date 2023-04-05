/*
Downsample point cloud from a ZDF file.

The ZDF files for this sample can be found under the main instructions for Zivid samples.
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

        std::cout << "Running visualizer. Blocking until window closes." << std::endl;
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

        std::cout << "Getting point cloud from frame" << std::endl;
        auto pointCloud = frame.pointCloud();

        std::cout << "Size of point cloud before downsampling: " << pointCloud.size() << " data points" << std::endl;

        visualizePointCloud(pointCloud);

        std::cout << "Downsampling point cloud" << std::endl;
        std::cout << "This does not modify the current point cloud but returns" << std::endl;
        std::cout << "the downsampled point cloud as a new point cloud instance." << std::endl;
        auto downsampledPointCloud = pointCloud.downsampled(Zivid::PointCloud::Downsampling::by2x2);

        std::cout << "Size of point cloud after downsampling: " << downsampledPointCloud.size() << " data points"
                  << std::endl;

        std::cout << "Downsampling point cloud (in-place)" << std::endl;
        std::cout << "This modifies the current point cloud." << std::endl;
        pointCloud.downsample(Zivid::PointCloud::Downsampling::by2x2);

        std::cout << "Size of point cloud after downsampling: " << pointCloud.size() << " data points" << std::endl;

        visualizePointCloud(pointCloud);
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
