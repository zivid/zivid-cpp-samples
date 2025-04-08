/*
Capture point clouds with multiple cameras sequentially with interleaved processing.
*/

#include <Zivid/Zivid.h>

#include <chrono>
#include <future>
#include <iostream>
#include <mutex>
#include <vector>

namespace
{
    Zivid::Array2D<Zivid::PointXYZColorRGBA_SRGB> processAndSaveInThread(const Zivid::Frame &frame)
    {
        const auto pointCloud = frame.pointCloud();
        auto data = pointCloud.copyData<Zivid::PointXYZColorRGBA_SRGB>();

        // This is where you should run your processing

        const auto dataFile = "Frame_" + frame.cameraInfo().serialNumber().value() + ".zdf";
        std::cout << "Saving frame to file: " << dataFile << std::endl;
        frame.save(dataFile);

        return data;
    }

    std::vector<Zivid::Camera> connectToAllAvailableCameras(const std::vector<Zivid::Camera> &cameras)
    {
        std::vector<Zivid::Camera> connectedCameras;
        for(auto camera : cameras)
        {
            if(camera.state().status() == Zivid::CameraState::Status::available)
            {
                std::cout << "Connecting to camera: " << camera.info().serialNumber() << std::endl;
                camera.connect();
                connectedCameras.push_back(camera);
            }
            else
            {
                std::cout << "Camera " << camera.info().serialNumber() << "is not available. "
                          << "Camera status: " << camera.state().status() << std::endl;
            }
        }
        return connectedCameras;
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Finding cameras" << std::endl;
        auto cameras = zivid.cameras();
        std::cout << "Number of cameras found: " << cameras.size() << std::endl;

        auto connectedCameras = connectToAllAvailableCameras(cameras);

        std::vector<std::future<Zivid::Array2D<Zivid::PointXYZColorRGBA_SRGB>>> futureData;

        for(auto &camera : connectedCameras)
        {
            std::cout << "Capturing frame with camera: " << camera.info().serialNumber() << std::endl;
            const auto settings =
                Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} },
                                 Zivid::Settings::Color{ Zivid::Settings2D{
                                     Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } } } };
            const auto frame = camera.capture2D3D(settings);

            std::cout << "Starting to process and save (in a separate thread) the frame captured with camera: "
                      << frame.cameraInfo().serialNumber().value() << std::endl;
            futureData.emplace_back(std::async(std::launch::async, processAndSaveInThread, frame));
        }

        // This is where the scene can move relative to the cameras

        std::vector<Zivid::Array2D<Zivid::PointXYZColorRGBA_SRGB>> allData;

        for(size_t i = 0; i < cameras.size(); ++i)
        {
            std::cout << "Waiting for processing and saving to finish for camera " << cameras[i].info().serialNumber()
                      << std::endl;
            const auto data = futureData[i].get();
            allData.push_back(data);
        }

        // This is where all data is available for further processing, e.g., stitching
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
