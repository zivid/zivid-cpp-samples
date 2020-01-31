/*
This example shows how to read the intrinsic calibration parameters of the
Zivid camera (OpenCV model).
*/

#include <Zivid/Zivid.h>

#include <chrono>
#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to the camera" << std::endl;
        auto camera = zivid.connectCamera();

        auto intrinsics = camera.intrinsics();

        std::string fileNameIntrinsics = "intrinsics.yml";
        std::cout << "Saving camera intrinsics to " << fileNameIntrinsics << std::endl;
        intrinsics.save(fileNameIntrinsics);

        std::cout << intrinsics.description << " - " << intrinsics << std::endl;

        std::cout << camera.intrinsics().cameraMatrix().cx().description
                  << ": CX = " << camera.intrinsics().cameraMatrix().cx().value() << std::endl;
        std::cout << camera.intrinsics().cameraMatrix().cy().description
                  << ": CY = " << camera.intrinsics().cameraMatrix().cy().value() << std::endl;
        std::cout << camera.intrinsics().cameraMatrix().fx().description
                  << ": FX = " << camera.intrinsics().cameraMatrix().fx().value() << std::endl;
        std::cout << camera.intrinsics().cameraMatrix().fy().description
                  << ": FY = " << camera.intrinsics().cameraMatrix().fy().value() << std::endl;

        std::cout << camera.intrinsics().distortion().k1().description
                  << ": K1 = " << camera.intrinsics().distortion().k1().value() << std::endl;
        std::cout << camera.intrinsics().distortion().k2().description
                  << ": K2 = " << camera.intrinsics().distortion().k2().value() << std::endl;
        std::cout << camera.intrinsics().distortion().k3().description
                  << ": K3 = " << camera.intrinsics().distortion().k3().value() << std::endl;
        std::cout << camera.intrinsics().distortion().p1().description
                  << ": P1 = " << camera.intrinsics().distortion().p1().value() << std::endl;
        std::cout << camera.intrinsics().distortion().p2().description
                  << ": P2 = " << camera.intrinsics().distortion().p2().value() << std::endl;
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
