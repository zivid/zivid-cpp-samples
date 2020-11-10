/*
This example shows how to read intrinsic parameters from the Zivid camera (OpenCV model).
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

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Getting camera intrinsics" << std::endl;
        auto intrinsics = Zivid::Experimental::Calibration::intrinsics(camera);

        std::cout << intrinsics << std::endl;

        std::cout << "Separated camera intrinsic parameters with description:" << std::endl;

        std::cout << "    CX: " << std::left << std::setfill(' ') << std::setw(13)
                  << intrinsics.cameraMatrix().cx().value() << Zivid::CameraIntrinsics::CameraMatrix::CX::description
                  << std::endl;
        std::cout << "    CY: " << std::left << std::setfill(' ') << std::setw(13)
                  << intrinsics.cameraMatrix().cy().value() << Zivid::CameraIntrinsics::CameraMatrix::CY::description
                  << std::endl;
        std::cout << "    FX: " << std::left << std::setfill(' ') << std::setw(13)
                  << intrinsics.cameraMatrix().fx().value() << Zivid::CameraIntrinsics::CameraMatrix::FX::description
                  << std::endl;
        std::cout << "    FY: " << std::left << std::setfill(' ') << std::setw(13)
                  << intrinsics.cameraMatrix().fy().value() << Zivid::CameraIntrinsics::CameraMatrix::FY::description
                  << std::endl;

        std::cout << "    K1: " << std::left << std::setfill(' ') << std::setw(13)
                  << intrinsics.distortion().k1().value() << Zivid::CameraIntrinsics::Distortion::K1::description
                  << std::endl;
        std::cout << "    K2: " << std::left << std::setfill(' ') << std::setw(13)
                  << intrinsics.distortion().k2().value() << Zivid::CameraIntrinsics::Distortion::K2::description
                  << std::endl;
        std::cout << "    K3: " << std::left << std::setfill(' ') << std::setw(13)
                  << intrinsics.distortion().k3().value() << Zivid::CameraIntrinsics::Distortion::K3::description
                  << std::endl;
        std::cout << "    P1: " << std::left << std::setfill(' ') << std::setw(13)
                  << intrinsics.distortion().p1().value() << Zivid::CameraIntrinsics::Distortion::P1::description
                  << std::endl;
        std::cout << "    P2: " << std::left << std::setfill(' ') << std::setw(13)
                  << intrinsics.distortion().p2().value() << Zivid::CameraIntrinsics::Distortion::P2::description
                  << std::endl;

        std::string outputFile = "Intrinsics.yml";
        std::cout << "Saving camera intrinsics to file: " << outputFile << std::endl;
        intrinsics.save(outputFile);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }
}
