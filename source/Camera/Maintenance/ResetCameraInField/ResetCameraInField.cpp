/*
Reset infield correction on a camera.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.

For more information about in-field correction, check out this tutorial:
https://support.zivid.com/en/latest/camera/academy/camera/infield-correction.html
*/

#include <Zivid/Calibration/InfieldCorrection.h>
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

        std::cout << "Reset infield correction on the camera" << std::endl;
        Zivid::Calibration::resetCameraCorrection(camera);
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
