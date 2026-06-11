/*
 * Automatically configure the IP addresses of connected cameras to match the network of the user's PC.
 * 
 * Usage:
 * - By default, the program applies the new configuration directly to the cameras.
 * - Use the [--display-only] argument to simulate the configuration and display the
 *   proposed IP addresses without making actual changes.
 */

#include <Zivid/Zivid.h>
#include <clipp.h>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>

namespace
{
    std::vector<std::string> splitIpAddress(const std::string &str, char delimiter)
    {
        std::vector<std::string> parts;
        std::istringstream userInput(str);
        for(std::string part; std::getline(userInput, part, delimiter);)
        {
            parts.push_back(part);
        }
        return parts;
    }

    struct UserNetworkInfo
    {
        std::string address;
        std::string mask;
    };

    UserNetworkInfo getUsersLocalInterfaceNetworkConfiguration(const Zivid::Camera &camera)
    {
        Zivid::CameraState::Network::LocalInterfaces localInterfaces = camera.state().network().localInterfaces();

        if(localInterfaces.isEmpty())
        {
            throw std::runtime_error(
                "No user local interface detected from the camera " + camera.info().serialNumber().toString());
        }

        if(localInterfaces.size() > 1)
        {
            throw std::runtime_error(
                "More than one local interface detected from the camera " + camera.info().serialNumber().toString()
                + ". Please, reorganize your network.");
        }

        if(localInterfaces.at(0).ipv4().subnets().isEmpty())
        {
            throw std::runtime_error("No valid subnets found for camera " + camera.info().serialNumber().toString());
        }

        if(localInterfaces.at(0).ipv4().subnets().size() > 1)
        {
            throw std::runtime_error(
                "More than one ip address found for the local interface from the camera "
                + camera.info().serialNumber().toString());
        }

        auto subnet = localInterfaces.at(0).ipv4().subnets().at(0);
        return { subnet.address().toString(), subnet.mask().toString() };
    }
} // namespace

int main(int argc, char **argv)
{
    try
    {
        bool displayOnly = false;

        auto cli = clipp::group(
            clipp::option("--display-only").set(displayOnly)
            % "Only display the new network configurations of the camera(s) without applying changes");

        // Parse arguments
        if(!clipp::parse(argc, argv, cli))
        {
            // Formatting for CLI documentation
            auto fmt = clipp::doc_formatting{}.alternatives_min_split_size(1).surround_labels("\"", "\"");
            std::cout << "USAGE:" << std::endl;
            std::cout << clipp::usage_lines(cli, argv[0], fmt) << std::endl;
            std::cout << "OPTIONS:" << std::endl;
            std::cout << clipp::documentation(cli) << std::endl;
            throw std::runtime_error("Command-line parsing failed.");
        }

        Zivid::Application zivid;
        auto cameras = zivid.cameras();

        if(cameras.empty())
        {
            throw std::runtime_error("Failed to connect to camera. No cameras found.");
        }

        std::map<std::string, std::vector<Zivid::Camera>> localInterfaceIpToCameras;

        for(auto &camera : cameras)
        {
            try
            {
                const auto [localInterfaceIpAddress, localInterfaceSubnetMask] =
                    getUsersLocalInterfaceNetworkConfiguration(camera);

                auto ipAddressOctets = splitIpAddress(localInterfaceIpAddress, '.');

                int nextIpAddressLastOctet = std::stoi(ipAddressOctets.back());

                // Identifying the last octet of the new ip address for the current camera
                if(localInterfaceIpToCameras.find(localInterfaceIpAddress) == localInterfaceIpToCameras.end())
                {
                    nextIpAddressLastOctet += 1;
                }
                else
                {
                    nextIpAddressLastOctet += localInterfaceIpToCameras[localInterfaceIpAddress].size() + 1;
                }

                localInterfaceIpToCameras[localInterfaceIpAddress].push_back(camera);

                const std::string remainingIpAddressOctets =
                    ipAddressOctets[0] + "." + ipAddressOctets[1] + "." + ipAddressOctets[2];

                const Zivid::NetworkConfiguration newConfig(
                    Zivid::NetworkConfiguration::IPV4(
                        Zivid::NetworkConfiguration::IPV4::Mode::manual,
                        Zivid::NetworkConfiguration::IPV4::Address(
                            remainingIpAddressOctets + "." + std::to_string(nextIpAddressLastOctet)),
                        Zivid::NetworkConfiguration::IPV4::SubnetMask(localInterfaceSubnetMask)));

                if(displayOnly)
                {
                    std::cout << "Current camera serial number : " << camera.info().serialNumber() << "\n"
                              << "Current camera " << camera.networkConfiguration()
                              << "Current local interface detected: " << localInterfaceIpAddress
                              << "\nSimulated new camera address ip: " << remainingIpAddressOctets << "."
                              << std::to_string(nextIpAddressLastOctet) << "\n\n"
                              << std::endl;
                }
                else
                {
                    std::cout << "Applying network configuration to camera with serial number: "
                              << camera.info().serialNumber() << "\n"
                              << "Current local interface detected: " << localInterfaceIpAddress << std::endl;

                    camera.applyNetworkConfiguration(newConfig);

                    std::cout << "New camera network configuration: " << camera.networkConfiguration() << "\n\n"
                              << std::endl;
                }
            }
            catch(const std::exception &e)
            {
                std::cerr << "Error when configuring camera: " << camera.networkConfiguration() << " " << e.what()
                          << std::endl;
            }
        }
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
