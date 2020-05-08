/*
This example shows how to read the intrinsic calibration parameters of the
Zivid camera (OpenCV model).
*/

#include <Zivid/Experimental/Calibration.h>
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

        auto intrinsics = Zivid::Experimental::Calibration::intrinsics(camera);

        std::string fileNameIntrinsics = "Intrinsics.yml";
        std::cout << "Saving camera intrinsics to " << fileNameIntrinsics << std::endl;
        intrinsics.save(fileNameIntrinsics);

        std::cout << Zivid::CameraIntrinsics::description << " - " << intrinsics << std::endl;

        std::cout << Zivid::CameraIntrinsics::CameraMatrix::CX::description
                  << ": CX = " << intrinsics.cameraMatrix().cx().value() << std::endl;
        std::cout << Zivid::CameraIntrinsics::CameraMatrix::CY::description
                  << ": CY = " << intrinsics.cameraMatrix().cy().value() << std::endl;
        std::cout << Zivid::CameraIntrinsics::CameraMatrix::FX::description
                  << ": FX = " << intrinsics.cameraMatrix().fx().value() << std::endl;
        std::cout << Zivid::CameraIntrinsics::CameraMatrix::FY::description
                  << ": FY = " << intrinsics.cameraMatrix().fy().value() << std::endl;

        std::cout << Zivid::CameraIntrinsics::Distortion::K1::description
                  << ": K1 = " << intrinsics.distortion().k1().value() << std::endl;
        std::cout << Zivid::CameraIntrinsics::Distortion::K2::description
                  << ": K2 = " << intrinsics.distortion().k2().value() << std::endl;
        std::cout << Zivid::CameraIntrinsics::Distortion::K3::description
                  << ": K3 = " << intrinsics.distortion().k3().value() << std::endl;
        std::cout << Zivid::CameraIntrinsics::Distortion::P1::description
                  << ": P1 = " << intrinsics.distortion().p1().value() << std::endl;
        std::cout << Zivid::CameraIntrinsics::Distortion::P2::description
                  << ": P2 = " << intrinsics.distortion().p2().value() << std::endl;
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
