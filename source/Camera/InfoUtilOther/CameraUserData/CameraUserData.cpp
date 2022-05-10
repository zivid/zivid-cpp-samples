/*
Store user data on the Zivid camera.
*/

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

    void checkUserDataSupport(const Zivid::Camera &camera)
    {
        const auto maxDataSize = camera.info().userData().maxSizeBytes();
        if(maxDataSize.value() == 0)
        {
            throw std::runtime_error{ "This camera does not support user data" };
        }
    }
} // namespace

int main(int argc, char **argv)
{
    try
    {
        std::string userData;
        Mode mode = Mode::read;
        auto readMode = (clipp::command("read").set(mode, Mode::read));
        auto clearMode = (clipp::command("clear").set(mode, Mode::clear));
        auto writeMode = (clipp::command("write").set(mode, Mode::write), clipp::value("userData", userData));

        auto cli = ((writeMode | readMode | clearMode));

        if(!parse(argc, argv, cli))
        {
            auto fmt = clipp::doc_formatting{}.alternatives_min_split_size(1).surround_labels("\"", "\"");
            std::cout << clipp::usage_lines(cli, "CameraUserData", fmt) << std::endl;
            throw std::runtime_error{ "Invalid usage" };
        }

        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();
        checkUserDataSupport(camera);

        switch(mode)
        {
            case Mode::read:
                std::cout << "Reading user data from camera" << std::endl;
                std::cout << "Done. User data: '" << read(camera) << "'" << std::endl;
                break;
            case Mode::write:
            {
                std::cout << "Writing '" << userData << "' to the camera" << std::endl;
                write(camera, userData);
                std::cout << "Done. Note! Camera must be rebooted to allow another write operation" << std::endl;
                break;
            }
            case Mode::clear:
                std::cout << "Clearing user data from camera" << std::endl;
                clear(camera);
                std::cout << "Done. Note! Camera must be rebooted to allow another clear operation" << std::endl;
                break;
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
