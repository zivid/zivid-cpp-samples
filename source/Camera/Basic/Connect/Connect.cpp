/*
Connect to a Zivid camera using the different available methods.

Replace the IP address and serial number in the code with the ones of your camera.
*/

#include <Zivid/Zivid.h>

#include <iostream>

namespace
{
    void printDiscoveredCameras(Zivid::Application &zivid)
    {
        std::cout << "Discovered cameras:" << std::endl;
        for(const auto &camera : zivid.cameras())
        {
            std::cout << "Serial number: " << camera.info().serialNumber()
                      << ", IP address: " << camera.state().network().ipv4().address() << std::endl;
        }
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        printDiscoveredCameras(zivid);

        std::cout << "The serial number, IP address and hostname below are placeholders. "
                     "Replace them with the ones of your camera."
                  << std::endl;

        {
            std::cout << "Connecting to the first available camera" << std::endl;
            auto camera = zivid.connectCamera();
            camera.disconnect();
        }

        {
            std::cout << "Connecting to the camera with a specific serial number" << std::endl;
            auto camera = zivid.connectCamera(Zivid::CameraInfo::SerialNumber{ "2020C0DE" });
            camera.disconnect();
        }

        {
            std::cout << "Connecting to the camera at a specific IP address" << std::endl;
            auto camera = zivid.connectCamera(Zivid::CameraAddress{ "172.28.60.5" });
            camera.disconnect();
        }

        {
            std::cout << "Connecting to the camera at a specific hostname" << std::endl;
            // The default hostname format is "zivid-<serial-number>.local".
            // The hostname cannot be read or set through the SDK.
            auto camera = zivid.connectCamera(Zivid::CameraAddress{ "zivid-2020C0DE.local" });
            camera.disconnect();
        }

        std::cout << "Connecting to all available cameras" << std::endl;
        std::vector<Zivid::Camera> connectedCameras;
        for(auto &camera : zivid.cameras())
        {
            if(camera.state().status() == Zivid::CameraState::Status::available)
            {
                std::cout << "Connecting to camera: " << camera.info().serialNumber() << std::endl;
                camera.connect();
                connectedCameras.push_back(camera);
            }
            else
            {
                std::cout << "Camera " << camera.info().serialNumber() << " is not available. "
                          << "Camera status: " << camera.state().status() << std::endl;
            }
        }
        for(auto &camera : connectedCameras)
        {
            camera.disconnect();
        }
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
