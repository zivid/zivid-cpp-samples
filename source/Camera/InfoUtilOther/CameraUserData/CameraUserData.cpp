#include <Zivid/Zivid.h>

#include <clipp.h>

#include <iostream>

namespace
{
    enum class Mode
    {
        read,
        write,
        clear
    };

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

        std::string userData;
        Mode selected = Mode::read;
        auto readMode = (clipp::command("read").set(selected, Mode::read));
        auto clearMode = (clipp::command("clear").set(selected, Mode::clear));
        auto writeMode = (clipp::command("write").set(selected, Mode::write), clipp::values("userData", userData));

        auto cli = ((readMode | clearMode | writeMode));

        if(parse(argc, argv, cli))
        {
            switch(selected)
            {
                case Mode::read:
                    std::cout << "Reading user data from camera" << std::endl;
                    std::cout << "Done. User data: '" << read(camera) << "'" << std::endl;
                    break;
                case Mode::write:
                {
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
        else
        {
            std::cout << clipp::usage_lines(cli, *argv) << std::endl;
        }
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
