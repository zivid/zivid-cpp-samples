/*
Detect and decode linear and matrix barcodes from a 2D capture.
*/

#include <Zivid/Experimental/Toolbox/Barcode.h>
#include <Zivid/Zivid.h>

#include <iostream>

using Zivid::Experimental::Toolbox::LinearBarcodeFormat;
using Zivid::Experimental::Toolbox::MatrixBarcodeFormat;

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        const auto barcodeDetector = Zivid::Experimental::Toolbox::BarcodeDetector();

        // Select your specific barcode formats for optimal performance
        const auto linearFormatFilter = LinearBarcodeFormat::code128 | LinearBarcodeFormat::code93
                                        | LinearBarcodeFormat::code39 | LinearBarcodeFormat::ean13
                                        | LinearBarcodeFormat::ean8 | LinearBarcodeFormat::upcA
                                        | LinearBarcodeFormat::upcE;
        const auto matrixFormatFilter = MatrixBarcodeFormat::qrcode | MatrixBarcodeFormat::dataMatrix;

        const auto settings2d = barcodeDetector.suggestSettings(camera);

        std::cout << "Detecting barcodes ..." << std::endl;
        const auto frame2d = camera.capture2D(settings2d);

        const auto linearBarcodeResults = barcodeDetector.readLinearCodes(frame2d, linearFormatFilter);
        const auto matrixBarcodeResults = barcodeDetector.readMatrixCodes(frame2d, matrixFormatFilter);

        if(!linearBarcodeResults.empty())
        {
            std::cout << "Detected " << linearBarcodeResults.size() << " linear barcodes:" << std::endl;
            for(const auto &result : linearBarcodeResults)
            {
                std::cout << "-- Detected barcode " << result.code() << " on format " << toString(result.codeFormat())
                          << " at pixel " << result.centerPosition() << std::endl;
            }
        }

        if(!matrixBarcodeResults.empty())
        {
            std::cout << "Detected " << matrixBarcodeResults.size() << " matrix barcodes:" << std::endl;
            for(const auto &result : matrixBarcodeResults)
            {
                std::cout << "-- Detected barcode " << result.code() << " on format " << toString(result.codeFormat())
                          << " at pixel " << result.centerPosition() << std::endl;
            }
        }

        if(linearBarcodeResults.empty() && matrixBarcodeResults.empty())
        {
            std::cout << "No barcodes detected" << std::endl;
        }
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
