/*
Capture point clouds, with color, with the Zivid file camera.
This sample can be used without access to a physical camera.

The file camera files are found in Zivid Sample Data with ZFC file extension.
See the instructions in README.md to download the Zivid Sample Data.
There are five available file cameras to choose from, one for each camera model.
The default file camera used in this sample is the Zivid 2+ MR60 file camera.
*/

#include <Zivid/Zivid.h>

#include <clipp.h>
#include <iostream>

int main(int argc, char **argv)
{
    try
    {
        bool userInput = false;

        std::string fileCameraPath;
        auto cli =
            (clipp::option("--file-camera").set(userInput, true)
             & clipp::value("<Path to the file camera .zfc file>", fileCameraPath));

        if(!parse(argc, argv, cli))
        {
            auto fmt = clipp::doc_formatting{}.alternatives_min_split_size(1).surround_labels("\"", "\"");
            std::cout << clipp::usage_lines(cli, "Usage: ", fmt) << std::endl;
            throw std::runtime_error{ "Invalid usage" };
        }

        Zivid::Application zivid;

        const auto fileCamera =
            userInput ? fileCameraPath : std::string(ZIVID_SAMPLE_DATA_DIR) + "/FileCameraZivid2PlusMR60.zfc";

        std::cout << "Creating virtual camera using file: " << fileCamera << std::endl;
        auto camera = zivid.createFileCamera(fileCamera);

        std::cout << "Configuring settings" << std::endl;
        Zivid::Settings settings{
            Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} },
            Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
            Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{ 1.5 },
            Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes,
            Zivid::Settings::Processing::Filters::Reflection::Removal::Mode::global,
        };
        Zivid::Settings2D settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} },
                                      Zivid::Settings2D::Processing::Color::Balance::Red{ 1 },
                                      Zivid::Settings2D::Processing::Color::Balance::Green{ 1 },
                                      Zivid::Settings2D::Processing::Color::Balance::Blue{ 1 } };

        settings.color() = Zivid::Settings::Color{ settings2D };

        std::cout << "Capturing frame" << std::endl;
        const auto frame = camera.capture2D3D(settings);

        const auto dataFile = "Frame.zdf";
        std::cout << "Saving frame to file: " << dataFile << std::endl;
        frame.save(dataFile);
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
