// Please make sure that Zivid sample data has been selected during installation of Zivid software.
// Latest version of Zivid software (including samples) can be found at http://zivid.com/software/.

#include <Zivid/Zivid.h>
#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        auto zdfFile = Zivid::Environment::dataPath() + "/MiscObjects.zdf";
        auto resultFile = "result.zdf";

        std::cout << "Initializing camera emulation using file: " << zdfFile << std::endl;
        auto camera = zivid.createFileCamera(zdfFile);

        std::cout << "Capture a frame" << std::endl;
        auto frame = camera.capture();

        std::cout << "Saving frame to file: " << resultFile << std::endl;
        frame.save(resultFile);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
