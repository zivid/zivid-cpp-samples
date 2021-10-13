/*
Capture Zivid point clouds, compute normals and print a subset.

For scenes with high dynamic range we combine multiple acquisitions to get an HDR point cloud.
*/

#include <Zivid/Zivid.h>

#include <iostream>

namespace
{
    void printNormals(int radius,
                      const Zivid::Array2D<Zivid::NormalXYZ> &normals,
                      const int numOfRows,
                      const int numOfCols)
    {
        const auto lineSeparator = std::string(50, '-');
        std::cout << lineSeparator << std::endl;
        for(int row = (numOfRows / 2 - radius); row < (numOfRows / 2 + radius); row++)
        {
            for(int col = (numOfCols / 2 - radius); col < (numOfCols / 2 + radius); col++)
            {
                std::cout << "Normals (" << row << "," << col << "): " << normals(row, col) << std::endl;
            }
        }
        std::cout << lineSeparator << std::endl;
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Configuring settings" << std::endl;
        Zivid::Settings settings;
        for(const auto aperture : { 9.57, 4.76, 2.59 })
        {
            std::cout << "Adding acquisition with aperture = " << aperture << std::endl;
            const auto acquisitionSettings = Zivid::Settings::Acquisition{
                Zivid::Settings::Acquisition::Aperture{ aperture },
            };
            settings.acquisitions().emplaceBack(acquisitionSettings);
        }

        std::cout << "Capturing frame (HDR)" << std::endl;
        const auto frame = camera.capture(settings);
        const auto pointCloud = frame.pointCloud();

        std::cout << "Computing normals and copying them to CPU memory" << std::endl;
        const auto normals = pointCloud.copyData<Zivid::NormalXYZ>();

        const int radiusOfPixelsToPrint(5);
        std::cout << "Printing normals for the central ";
        std::cout << radiusOfPixelsToPrint * 2 << " x " << radiusOfPixelsToPrint * 2 << " pixels" << std::endl;
        printNormals(radiusOfPixelsToPrint, normals, pointCloud.height(), pointCloud.width());
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }
}
