/*
Capture Zivid point clouds, compute normals and print a subset.
*/

#include <Zivid/Zivid.h>

#include <iostream>

namespace
{
    void
    printNormals(int radius, const Zivid::Array2D<Zivid::NormalXYZ> &normals, const int numOfRows, const int numOfCols)
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
        Zivid::Settings settings = Zivid::Settings{
            Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{ Zivid::Settings::Acquisition{} } },
            Zivid::Settings::Color{
                Zivid::Settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } } }
        };

        std::cout << "Capturing frame" << std::endl;
        const auto frame = camera.capture3D(settings);
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

    return EXIT_SUCCESS;
}
