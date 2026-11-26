/*
Connect to a Zivid camera.

Without arguments, the sample connects to the first available camera. To connect to a specific camera, provide its
serial number with --serial, its IP address with --ip, or its hostname with --hostname. To connect to all available
cameras, use --all. The sample connects with every method you provide, and fails if any of them fails.

Examples:
    Connect --serial 2020C0DE
    Connect --ip 172.28.60.5
    Connect --hostname zivid-2020C0DE.local
    Connect --serial 2020C0DE --ip 172.28.60.5
    Connect --all
*/

#include <Zivid/Zivid.h>

#include <clipp.h>

#include <iostream>
#include <optional>
#include <vector>

namespace
{
    struct ConnectOptions
    {
        std::optional<std::string> serialNumber;
        std::optional<std::string> ipAddress;
        std::optional<std::string> hostname;
        bool allCameras{ false };
    };

    ConnectOptions parseOptions(int argc, char **argv)
    {
        bool useSerialNumber{ false };
        bool useIpAddress{ false };
        bool useHostname{ false };
        std::string serialNumber;
        std::string ipAddress;
        std::string hostname;
        bool allCameras{ false };

        auto cli =
            (clipp::option("--serial").set(useSerialNumber)
                 & clipp::value("serial number", serialNumber).doc("Connect to the camera with this serial number"),
             clipp::option("--ip").set(useIpAddress)
                 & clipp::value("ip address", ipAddress).doc("Connect to the camera at this IP address"),
             clipp::option("--hostname").set(useHostname)
                 & clipp::value("hostname", hostname).doc("Connect to the camera at this hostname"),
             clipp::option("--all").set(allCameras).doc("Connect to all available cameras"));

        if(!clipp::parse(argc, argv, cli))
        {
            std::cout << clipp::usage_lines(cli, "Connect") << std::endl;
            throw std::runtime_error{ "Invalid usage" };
        }

        return ConnectOptions{
            useSerialNumber ? std::optional{ serialNumber } : std::nullopt,
            useIpAddress ? std::optional{ ipAddress } : std::nullopt,
            useHostname ? std::optional{ hostname } : std::nullopt,
            allCameras,
        };
    }

    void printDiscoveredCameras(Zivid::Application &zivid)
    {
        std::cout << "Discovered cameras:" << std::endl;
        for(const auto &camera : zivid.cameras())
        {
            std::cout << "Serial number: " << camera.info().serialNumber()
                      << ", IP address: " << camera.state().network().ipv4().address()
                      << ", hostname: " << camera.state().network().hostname() << std::endl;
        }
    }

    void connectToFirstAvailableCamera(Zivid::Application &zivid)
    {
        std::cout << "Connecting to the first available camera" << std::endl;
        auto camera = zivid.connectCamera();
        camera.disconnect();
    }

    void connectBySerialNumber(Zivid::Application &zivid, const std::string &serialNumber)
    {
        std::cout << "Connecting to the camera with serial number " << serialNumber << std::endl;
        auto camera = zivid.connectCamera(Zivid::CameraInfo::SerialNumber{ serialNumber });
        camera.disconnect();
    }

    void connectByIpAddress(Zivid::Application &zivid, const std::string &ipAddress)
    {
        std::cout << "Connecting to the camera at IP address " << ipAddress << std::endl;
        auto camera = zivid.connectCamera(Zivid::CameraAddress{ ipAddress });
        camera.disconnect();
    }

    void connectByHostname(Zivid::Application &zivid, const std::string &hostname)
    {
        std::cout << "Connecting to the camera at hostname " << hostname << std::endl;
        // The default hostname format is "zivid-<serial-number>"; connect to it with the ".local" suffix.
        // The hostname is set through the network configuration, see the NetworkConfiguration sample.
        auto camera = zivid.connectCamera(Zivid::CameraAddress{ hostname });
        camera.disconnect();
    }

    void connectToAllAvailableCameras(Zivid::Application &zivid)
    {
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
} // namespace

int main(int argc, char **argv)
{
    try
    {
        const auto options = parseOptions(argc, argv);

        Zivid::Application zivid;

        printDiscoveredCameras(zivid);

        const bool noMethodGiven =
            !options.serialNumber && !options.ipAddress && !options.hostname && !options.allCameras;
        if(noMethodGiven)
        {
            connectToFirstAvailableCamera(zivid);
        }
        if(options.serialNumber)
        {
            connectBySerialNumber(zivid, *options.serialNumber);
        }
        if(options.ipAddress)
        {
            connectByIpAddress(zivid, *options.ipAddress);
        }
        if(options.hostname)
        {
            connectByHostname(zivid, *options.hostname);
        }
        if(options.allCameras)
        {
            connectToAllAvailableCameras(zivid);
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
