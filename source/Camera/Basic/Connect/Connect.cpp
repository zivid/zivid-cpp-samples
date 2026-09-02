/*
Connect to a Zivid camera using the different available methods.

Replace the IP address, serial number and hostname in the code with the ones of your camera,
or provide them with --serial, --ip and --hostname.
*/

#include <Zivid/Zivid.h>

#include <clipp.h>

#include <iostream>

namespace
{
    struct CameraIdentifiers
    {
        std::string serialNumber{ "2020C0DE" };
        std::string ipAddress{ "172.28.60.5" };
        std::string hostname{ "zivid-2020C0DE.local" };
    };

    CameraIdentifiers parseOptions(int argc, char **argv)
    {
        CameraIdentifiers identifiers;
        auto cli =
            (clipp::option("--serial")
                 & clipp::value("serial number", identifiers.serialNumber).doc("Serial number of the camera"),
             clipp::option("--ip") & clipp::value("ip address", identifiers.ipAddress).doc("IP address of the camera"),
             clipp::option("--hostname")
                 & clipp::value("hostname", identifiers.hostname).doc("Hostname of the camera"));

        if(!clipp::parse(argc, argv, cli))
        {
            std::cout << clipp::usage_lines(cli, "Connect") << std::endl;
            throw std::runtime_error{ "Invalid usage" };
        }
        return identifiers;
    }

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

int main(int argc, char **argv)
{
    try
    {
        const auto identifiers = parseOptions(argc, argv);

        Zivid::Application zivid;

        printDiscoveredCameras(zivid);

        {
            std::cout << "Connecting to the first available camera" << std::endl;
            auto camera = zivid.connectCamera();
            camera.disconnect();
        }

        {
            std::cout << "Connecting to the camera with serial number " << identifiers.serialNumber << std::endl;
            auto camera = zivid.connectCamera(Zivid::CameraInfo::SerialNumber{ identifiers.serialNumber });
            camera.disconnect();
        }

        {
            std::cout << "Connecting to the camera at IP address " << identifiers.ipAddress << std::endl;
            auto camera = zivid.connectCamera(Zivid::CameraAddress{ identifiers.ipAddress });
            camera.disconnect();
        }

        {
            std::cout << "Connecting to the camera at hostname " << identifiers.hostname << std::endl;
            // The default hostname format is "zivid-<serial-number>.local".
            // The hostname cannot be read or set through the SDK.
            auto camera = zivid.connectCamera(Zivid::CameraAddress{ identifiers.hostname });
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
