/*
Uses Zivid API to change the IP address of the Zivid camera.

*/

#include <Zivid/Zivid.h>

#include <algorithm>
#include <iostream>

namespace
{
    bool confirm(const std::string &message)
    {
        while(true)
        {
            std::cout << message << " [Y/n] ";
            std::string input;
            std::getline(std::cin, input);
            if(input == "y" || input == "Y" || input == "yes" || input == "Yes" || input == "YES")
            {
                return true;
            }
            if(input == "n" || input == "N" || input == "no" || input == "No" || input == "NO")
            {
                return false;
            }
            std::cout << "Invalid input. Please enter 'Y' or 'n'." << std::endl;
        }
    }

} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        auto cameras = zivid.cameras();

        if(cameras.empty())
        {
            throw std::runtime_error("Failed to connect to camera. No cameras found.");
        }

        auto camera = cameras[0];
        auto originalConfig = camera.networkConfiguration();

        std::cout << "Current network configuration of camera " << camera.info().serialNumber() << ":" << std::endl;
        std::cout << originalConfig << std::endl << std::endl;

        auto mode = Zivid::NetworkConfiguration::IPV4::Mode::manual;
        auto address = originalConfig.ipv4().address();
        auto subnetMask = originalConfig.ipv4().subnetMask();

        if(confirm("Do you want to use DHCP?"))
        {
            mode = Zivid::NetworkConfiguration::IPV4::Mode::dhcp;
        }
        else
        {
            std::string inputAddress;
            std::cout << "Enter IPv4 Address [" << originalConfig.ipv4().address() << "]: ";
            std::getline(std::cin, inputAddress);
            if(!inputAddress.empty())
            {
                address = Zivid::NetworkConfiguration::IPV4::Address{ inputAddress };
            }

            std::string inputSubnetMask;
            std::cout << "Enter new Subnet mask [" << originalConfig.ipv4().subnetMask() << "]: ";
            std::getline(std::cin, inputSubnetMask);
            if(!inputSubnetMask.empty())
            {
                subnetMask = Zivid::NetworkConfiguration::IPV4::SubnetMask{ inputSubnetMask };
            }
        }

        Zivid::NetworkConfiguration newConfig(Zivid::NetworkConfiguration::IPV4(mode, address, subnetMask));

        std::cout << "\nNew network configuration:" << std::endl;
        std::cout << newConfig << std::endl;
        if(!confirm(
               "Do you want to apply the new network configuration to camera " + camera.info().serialNumber().toString()
               + "?"))
        {
            return EXIT_SUCCESS;
        }

        std::cout << "Applying network configuration..." << std::endl;
        camera.applyNetworkConfiguration(newConfig);

        std::cout << "Updated network configuration of camera " << camera.info().serialNumber() << ":" << std::endl;
        std::cout << camera.networkConfiguration() << std::endl << std::endl;

        std::cout << "Camera status is '" << camera.state().status() << "'" << std::endl;
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
