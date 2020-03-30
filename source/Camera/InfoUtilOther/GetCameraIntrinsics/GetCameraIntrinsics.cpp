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

        std::cout << Zivid::CameraIntrinsics::description << " - " << intrinsics << std::endl;

        std::cout << Zivid::CameraIntrinsics::CameraMatrix::CX::description
                  << ": CX = " << camera.intrinsics().cameraMatrix().cx().value() << std::endl;
        std::cout << Zivid::CameraIntrinsics::CameraMatrix::CY::description
                  << ": CY = " << camera.intrinsics().cameraMatrix().cy().value() << std::endl;
        std::cout << Zivid::CameraIntrinsics::CameraMatrix::FX::description
                  << ": FX = " << camera.intrinsics().cameraMatrix().fx().value() << std::endl;
        std::cout << Zivid::CameraIntrinsics::CameraMatrix::FY::description
                  << ": FY = " << camera.intrinsics().cameraMatrix().fy().value() << std::endl;

        std::cout << Zivid::CameraIntrinsics::Distortion::K1::description
                  << ": K1 = " << camera.intrinsics().distortion().k1().value() << std::endl;
        std::cout << Zivid::CameraIntrinsics::Distortion::K2::description
                  << ": K2 = " << camera.intrinsics().distortion().k2().value() << std::endl;
        std::cout << Zivid::CameraIntrinsics::Distortion::K3::description
                  << ": K3 = " << camera.intrinsics().distortion().k3().value() << std::endl;
        std::cout << Zivid::CameraIntrinsics::Distortion::P1::description
                  << ": P1 = " << camera.intrinsics().distortion().p1().value() << std::endl;
        std::cout << Zivid::CameraIntrinsics::Distortion::P2::description
                  << ": P2 = " << camera.intrinsics().distortion().p2().value() << std::endl;
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
