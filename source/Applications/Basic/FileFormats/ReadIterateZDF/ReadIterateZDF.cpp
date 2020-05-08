/*
This example shows how to read point cloud data from a ZDF file, iterate through it, and extract individual points.
*/

#include <Zivid/Zivid.h>

#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        const auto dataFile = std::string(ZIVID_SAMPLE_DATA_DIR) + "/Zivid3D.zdf";
        std::cout << "Reading ZDF frame from file: " << dataFile << std::endl;
        const Zivid::Frame frame = Zivid::Frame(dataFile);

        // Extracting point cloud from the frame
        const auto pointCloud = frame.pointCloud();
        const auto data = pointCloud.copyData<Zivid::PointXYZColorRGBA>();
        const auto snr = pointCloud.copySNRs();

        std::cout << "Point cloud information:" << std::endl;
        std::cout << "Number of points: " << pointCloud.size() << "\n"
                  << "Height: " << pointCloud.height() << ", Width: " << pointCloud.width() << std::endl;

        // Iterating over the point cloud and displaying X, Y, Z, R, G, B, and Contrast for central 10 x 10 pixels
        const size_t pixelsToDisplay = 10;
        std::cout << "Iterating over point cloud and extracting X, Y, Z, R, G, B, and Contrast for central "
                  << pixelsToDisplay << " x " << pixelsToDisplay << " pixels " << std::endl;
        const size_t iStart = (pointCloud.height() - pixelsToDisplay) / 2;
        const size_t iEnd = (pointCloud.height() + pixelsToDisplay) / 2;
        const size_t jStart = (pointCloud.width() - pixelsToDisplay) / 2;
        const size_t jEnd = (pointCloud.width() + pixelsToDisplay) / 2;
        for(size_t i = iStart; i < iEnd; i++)
        {
            for(size_t j = jStart; j < jEnd; j++)
            {
                const auto &point = data(i, j);
                const auto &pointSnr = snr(i, j);

                std::cout << std::setprecision(1) << std::fixed << "Values at pixel (" << i << "," << j << "):   "
                          << "X:" << std::left << std::setfill(' ') << std::setw(8) << point.point.x
                          << "Y:" << std::setw(8) << point.point.y << "Z:" << std::setw(8) << point.point.z
                          << "R:" << std::setw(8) << std::to_string(point.color.r) << "G:" << std::setw(8)
                          << std::to_string(point.color.g) << "B:" << std::setw(8) << std::to_string(point.color.b)
                          << "SNR:" << std::setw(8) << pointSnr.value << std::endl;
            }
        }
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
