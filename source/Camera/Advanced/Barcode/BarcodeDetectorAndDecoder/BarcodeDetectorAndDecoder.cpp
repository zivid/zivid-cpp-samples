/*
Detect and decode linear and matrix barcodes from a 2D capture.

The sample captures a 2D frame with the settings the barcode detector suggests for the camera.
Linear barcodes are read in two steps: detection finds candidate regions, and decoding reads the
code from each candidate. Matrix barcodes are detected and decoded in one call. For each barcode,
the sample prints its code, format, bounding box and orientation.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.

For more information on how to use the Zivid Barcode Detector, check out the Barcode Reading tutorial:
https://support.zivid.com/en/latest/camera/academy/applications/barcode-detection.html
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
                                        | LinearBarcodeFormat::upcE | LinearBarcodeFormat::itf;
        const auto matrixFormatFilter = MatrixBarcodeFormat::qrcode | MatrixBarcodeFormat::dataMatrix;

        const auto settings2d = barcodeDetector.suggestSettings(camera);

        std::cout << "Capturing 2D frame ..." << std::endl;
        const auto frame2d = camera.capture2D(settings2d);

        std::cout << "Detecting linear barcode candidates ..." << std::endl;
        const auto detectionResults = barcodeDetector.detectLinearCodes(frame2d);

        if(!detectionResults.empty())
        {
            std::cout << "Detected " << detectionResults.size() << " linear barcode candidates:" << std::endl;
            for(size_t i = 0; i < detectionResults.size(); ++i)
            {
                std::cout << "-- Candidate " << (i + 1) << ":" << std::endl;
                std::cout << "   Bounding box: " << detectionResults[i].boundingBox() << std::endl;
            }
        }
        else
        {
            std::cout << "No linear barcode candidates detected" << std::endl;
        }

        const auto decodingResults = barcodeDetector.decodeLinearCodes(detectionResults, linearFormatFilter);

        for(size_t i = 0; i < decodingResults.size(); ++i)
        {
            std::cout << "-- Candidate " << (i + 1) << ":" << std::endl;
            const auto &decodingResult = decodingResults[i];
            if(decodingResult.has_value())
            {
                const auto &decoded = decodingResult.value();
                std::cout << "   Code:         " << decoded.code() << std::endl;
                std::cout << "   Format:       " << toString(decoded.codeFormat()) << std::endl;
                std::cout << "   Bounding box: " << decoded.boundingBox() << std::endl;
                std::cout << "   Orientation:  " << decoded.orientation() << " ("
                          << decoded.orientation().angleInDegrees() << " degrees)" << std::endl;
            }
            else
            {
                std::cout << "   Failed to decode" << std::endl;
            }
        }

        std::cout << "Reading matrix barcodes ..." << std::endl;
        const auto matrixBarcodeResults = barcodeDetector.readMatrixCodes(frame2d, matrixFormatFilter);

        if(!matrixBarcodeResults.empty())
        {
            std::cout << "Detected " << matrixBarcodeResults.size() << " matrix barcodes:" << std::endl;
            for(const auto &result : matrixBarcodeResults)
            {
                std::cout << "-- Code:         " << result.code() << std::endl;
                std::cout << "   Format:       " << toString(result.codeFormat()) << std::endl;
                std::cout << "   Bounding box: " << result.boundingBox() << std::endl;
                std::cout << "   Orientation:  " << result.orientation() << " ("
                          << result.orientation().angleInDegrees() << " degrees)" << std::endl;
            }
        }
        else
        {
            std::cout << "No matrix barcodes detected" << std::endl;
        }
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}
