#include <Zivid/Zivid.h>

#include <iostream>

namespace
{
    enum class Mode
    {
        read,
        write,
        clear
    };

    std::invalid_argument usageException(const char *const *argv)
    {
        return std::invalid_argument{ std::string{ "Usage: " } + argv[0] + " <read|write <string>|clear>" };
    }

    Mode getMode(int argc, const char *const *argv)
    {
        if(argc >= 2)
        {
            if(std::string{ "read" } == argv[1])
            {
                return Mode::read;
            }
            if(std::string{ "write" } == argv[1])
            {
                return Mode::write;
            }
            if(std::string{ "clear" } == argv[1])
            {
                return Mode::clear;
            }
        }

        throw usageException(argv);
    }

    std::string getWriteData(int argc, const char *const *argv)
    {
        if(argc >= 3)
        {
            return argv[2];
        }
        else
        {
            throw usageException(argv);
        }
    }

    void write(Zivid::Camera &camera, const std::string &string)
    {
        camera.writeUserData(std::vector<uint8_t>{ begin(string), end(string) });
    }

    void clear(Zivid::Camera &camera)
    {
        write(camera, "");
    }

    std::string read(Zivid::Camera &camera)
    {
        const auto data = camera.userData();
        return std::string{ begin(data), end(data) };
    }
} // namespace

int main(int argc, char **argv)
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        const auto maxDataSize = camera.userDataMaxSizeBytes();
        if(maxDataSize == 0)
        {
            throw std::runtime_error{ "This camera does not support user data" };
        }

        switch(getMode(argc, argv))
        {
            case Mode::read:
                std::cout << "Reading user data from camera" << std::endl;
                std::cout << "Done. User data: '" << read(camera) << "'" << std::endl;
                break;
            case Mode::write:
            {
                auto userData = getWriteData(argc, argv);
                std::cout << "Writing '" << userData << "' to the camera" << std::endl;
                write(camera, userData);
                std::cout << "Done" << std::endl;
                break;
            }
            case Mode::clear:
                std::cout << "Clearing user data from camera" << std::endl;
                clear(camera);
                std::cout << "Done" << std::endl;
                break;
        }
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
