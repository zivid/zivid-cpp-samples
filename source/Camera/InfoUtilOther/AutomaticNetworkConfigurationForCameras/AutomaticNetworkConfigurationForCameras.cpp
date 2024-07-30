/*
Automatically set the IP addresses of any number of cameras to be in the same subnet as the provided IP address of the network interface.
*/

#include <Zivid/Zivid.h>
#include <clipp.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    void assertUserInput(const std::string &ipAddress, const std::string &subnetMask)
    {
        (void)Zivid::NetworkConfiguration::IPV4::Address{ ipAddress };
        (void)Zivid::NetworkConfiguration::IPV4::SubnetMask{ subnetMask };
    }

    std::vector<std::string> splitUserInput(const std::string &str, char delimiter)
    {
        std::vector<std::string> parts;
        std::istringstream userInput(str);
        for(std::string part; std::getline(userInput, part, delimiter);)
        {
            parts.push_back(part);
        }
        return parts;
    }

    std::tuple<std::string, std::string> parseOptions(int argc, char **argv)
    {
        std::string ipAddress;
        std::string subnetMask = "255.255.255.0";

        auto cli = clipp::group(
            clipp::required("--interface-ipv4") & clipp::value("IP address of the PC network interface", ipAddress),
            clipp::option("--subnet-mask") & clipp::value("Network subnet mask (default: 255.255.255.0)", subnetMask));

        if(!clipp::parse(argc, argv, cli))
        {
            auto fmt = clipp::doc_formatting{}.alternatives_min_split_size(1).surround_labels("\"", "\"");
            std::cout << "SYNOPSIS:" << std::endl;
            std::cout << clipp::usage_lines(cli, argv[0], fmt) << std::endl;
            std::cout << "OPTIONS:" << std::endl;
            std::cout << clipp::documentation(cli) << std::endl;
            throw std::runtime_error("Command-line parsing failed");
        }

        assertUserInput(ipAddress, subnetMask);

        return std::make_tuple(ipAddress, subnetMask);
    }
} // namespace

int main(int argc, char **argv)
{
    try
    {
        std::tuple<std::string, std::string> options = parseOptions(argc, argv);
        std::string ipAddress = std::get<0>(options);
        std::string subnetMask = std::get<1>(options);

        assertUserInput(ipAddress, subnetMask);

        auto ipAddressOctets = splitUserInput(ipAddress, '.');

        Zivid::Application zivid;
        auto cameras = zivid.cameras();
        if(cameras.empty())
        {
            throw std::runtime_error("Failed to connect to camera. No cameras found.");
        }

        int lastIpAddressOctet = std::stoi(ipAddressOctets[3]);
        std::string remainingIpAddressOctets = ipAddressOctets[0] + "." + ipAddressOctets[1] + "." + ipAddressOctets[2];

        // defines the last octet of the ip address of the first Zivid camera. Eg.: x.x.x.2
        int nextIpAddressLastOctet = 2;

        for(auto &camera : cameras)
        {
            if(nextIpAddressLastOctet == lastIpAddressOctet)
            {
                nextIpAddressLastOctet += 1;
            }

            Zivid::NetworkConfiguration newConfig(Zivid::NetworkConfiguration::IPV4(
                Zivid::NetworkConfiguration::IPV4::Mode::manual,
                Zivid::NetworkConfiguration::IPV4::Address(
                    remainingIpAddressOctets + "." + std::to_string(nextIpAddressLastOctet)),
                Zivid::NetworkConfiguration::IPV4::SubnetMask(subnetMask)));

            nextIpAddressLastOctet += 1;

            std::cout << "Applying network configuration to camera " << camera.info().serialNumber() << std::endl;
            camera.applyNetworkConfiguration(newConfig);
            std::cout << "New " << camera.networkConfiguration() << "\n" << std::endl;
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
