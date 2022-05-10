/*
Reset in-field correction on a camera.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.
*/

#include <Zivid/Experimental/Calibration/InfieldCorrection.h>
#include <Zivid/Zivid.h>

#include <iomanip>
#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Reset in-field correction on the camera" << std::endl;
        Zivid::Experimental::Calibration::resetCameraCorrection(camera);
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