/*
This example shows how to import a Zivid point cloud from a .ZDF file, iterate through, and extract individual points.
*/

#include <Zivid/Zivid.h>

#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        const std::string filename = "Zivid3D.zdf";
        std::cout << "Reading " << filename << " point cloud" << std::endl;
        const Zivid::Frame frame = Zivid::Frame(filename);

        // Extracting point cloud from the frame
        const auto pointCloud = frame.getPointCloud();

        std::cout << "Point cloud information:" << std::endl;
        std::cout << "Number of points: " << pointCloud.size() << "\n"
                  << "Height: " << pointCloud.height() << ", Width: " << pointCloud.width() << std::endl;

        // Iterating over the point cloud and displaying X, Y, Z, R, G, B, and Contrast for central 10 x 10 pixels
        const size_t pixelsToDisplay = 10;
        for(size_t i = (pointCloud.height() - pixelsToDisplay) / 2; i < (pointCloud.height() + pixelsToDisplay) / 2;
            i++)
        {
            for(size_t j = (pointCloud.width() - pixelsToDisplay) / 2; j < (pointCloud.width() + pixelsToDisplay) / 2;
                j++)
            {
                const auto &point = pointCloud(i, j);

                std::cout << std::setprecision(1) << std::fixed << "Values at pixel (" << i << ", " << j << "):"
                          << "    X:" << point.x << "  Y:" << point.y << "  Z:" << point.z
                          << "    R:" << static_cast<int>(point.red()) << "  G:" << static_cast<int>(point.green())
                          << "  B:" << static_cast<int>(point.blue()) << "    Contrast:" << point.contrast << std::endl;
            }
        }
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
