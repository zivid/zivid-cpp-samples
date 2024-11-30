/*
Transform point cloud data from millimeters to meters.

The ZDF file for this sample can be found under the main instructions for Zivid samples.

*/

#include <Zivid/Zivid.h>

#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        const auto dataFile = std::string(ZIVID_SAMPLE_DATA_DIR) + "/CalibrationBoardInCameraOrigin.zdf";
        std::cout << "Reading " << dataFile << " point cloud" << std::endl;
        auto frame = Zivid::Frame(dataFile);
        auto pointCloud = frame.pointCloud();

        const auto millimetersToMetersTransform =
            Zivid::Matrix4x4{ { 0.001F, 0, 0, 0 }, { 0, 0.001F, 0, 0 }, { 0, 0, 0.001F, 0 }, { 0, 0, 0, 1 } };

        std::cout << "Transforming point cloud from mm to m" << std::endl;
        pointCloud.transform(millimetersToMetersTransform);

        const auto transformedFile = "FrameInMeters.zdf";
        std::cout << "Saving transformed point cloud to file: " << transformedFile << std::endl;
        frame.save(transformedFile);
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
