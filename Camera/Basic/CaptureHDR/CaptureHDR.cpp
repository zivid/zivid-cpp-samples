#include <Zivid/Zivid.h>

#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Recording HDR source images" << std::endl;
        std::vector<Zivid::Frame> frames;
        for(const size_t iris : { 20U, 25U, 30U })
        {
            std::cout << "Capture frame with iris = " << iris << std::endl;
            camera << Zivid::Settings::Iris{ iris };
            frames.emplace_back(camera.capture());
        }

        std::cout << "Creating HDR frame" << std::endl;
        auto hdrFrame = Zivid::HDR::combineFrames(begin(frames), end(frames));

        std::cout << "Saving the frames" << std::endl;
        frames[0].save("20.zdf");
        frames[1].save("25.zdf");
        frames[2].save("30.zdf");
        hdrFrame.save("HDR.zdf");
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
