/*
Continuously capture 2D frames, detect and decode barcodes, and display a live view with the results overlaid.

For each decoded barcode, this sample draws its bounding box, code, and an arrow showing its
decoded orientation, meaning the direction the barcode should be read in.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without notice.

For more information on how to use the Zivid Barcode Detector, check out the Barcode Reading tutorial:
https://support.zivid.com/en/latest/camera/academy/applications/barcode-detection.html
*/

#include <Zivid/Experimental/Toolbox/Barcode.h>
#include <Zivid/Zivid.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace
{
    constexpr auto arrowLengthPx = 80.0;
    constexpr auto arrowHeadAngleDegrees = 30.0;
    constexpr auto arrowHeadLengthRatio = 0.3;

    cv::Scalar arrowColor()
    {
        return { 31, 95, 255 }; // neon orange, BGR
    }

    cv::Scalar boxColor()
    {
        return { 0, 200, 0 }; // green, BGR
    }

    cv::Scalar textColor()
    {
        return { 255, 0, 0 }; // blue, BGR
    }

    // Draws an arrow from start to tip, with a head angle narrower than cv::arrowedLine's fixed 45 degrees.
    void drawArrow(
        cv::Mat &image,
        const cv::Point &start,
        const cv::Point &tip,
        const cv::Scalar &color,
        const int thickness)
    {
        cv::line(image, start, tip, color, thickness, cv::LINE_AA);

        const auto shaftLength = cv::norm(start - tip);
        const auto headLength = shaftLength * arrowHeadLengthRatio;
        const auto reversedAngle =
            std::atan2(static_cast<double>(start.y - tip.y), static_cast<double>(start.x - tip.x));
        const auto headAngleRad = arrowHeadAngleDegrees * CV_PI / 180.0;

        for(const auto sign : { 1.0, -1.0 })
        {
            const cv::Point headPoint{
                cvRound(tip.x + headLength * std::cos(reversedAngle + sign * headAngleRad)),
                cvRound(tip.y + headLength * std::sin(reversedAngle + sign * headAngleRad)),
            };
            cv::line(image, tip, headPoint, color, thickness, cv::LINE_AA);
        }
    }

    template<typename DecodingResult>
    void drawDecodingResult(cv::Mat &image, const DecodingResult &decodingResult)
    {
        const auto box = decodingResult.boundingBox();
        cv::rectangle(
            image, cv::Point{ box.x, box.y }, cv::Point{ box.x + box.width, box.y + box.height }, boxColor(), 2);

        const auto centerPosition = decodingResult.centerPosition();
        const cv::Point center{ cvRound(centerPosition.x), cvRound(centerPosition.y) };

        const auto orientation = decodingResult.orientation();
        const cv::Point tip{
            cvRound(centerPosition.x + orientation.x * arrowLengthPx),
            cvRound(centerPosition.y + orientation.y * arrowLengthPx),
        };
        drawArrow(image, center, tip, arrowColor(), 3);

        const auto label =
            decodingResult.code() + " (" + std::to_string(cvRound(orientation.angleInDegrees())) + " deg)";
        cv::putText(
            image,
            label,
            cv::Point{ box.x, std::max(box.y - 10, 0) },
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            textColor(),
            2,
            cv::LINE_AA);
    }

    void drawOverlay(
        cv::Mat &image,
        const std::vector<Zivid::Experimental::Toolbox::LinearBarcodeDecodingResult> &linearResults,
        const std::vector<Zivid::Experimental::Toolbox::MatrixBarcodeDecodingResult> &matrixResults)
    {
        for(const auto &result : linearResults)
        {
            drawDecodingResult(image, result);
        }
        for(const auto &result : matrixResults)
        {
            drawDecodingResult(image, result);
        }

        const auto countLabel =
            "Linear: " + std::to_string(linearResults.size()) + "  Matrix: " + std::to_string(matrixResults.size());
        cv::putText(image, countLabel, cv::Point{ 10, 30 }, cv::FONT_HERSHEY_SIMPLEX, 0.8, textColor(), 2, cv::LINE_AA);
    }
} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        const auto barcodeDetector = Zivid::Experimental::Toolbox::BarcodeDetector();

        const auto settings2d = barcodeDetector.suggestSettings(camera);

        const auto windowName = "Zivid Live Barcode Reader (press 'q' or Esc to exit)";
        cv::namedWindow(windowName, cv::WINDOW_NORMAL);

        std::cout << "Press q to exit the viewer application" << std::endl;
        while(true)
        {
            const auto frame2d = camera.capture2D(settings2d);
            const auto sourceImage = frame2d.imageBGRA_SRGB();
            auto imageDataPointer = const_cast<void *>(static_cast<const void *>(sourceImage.data()));
            cv::Mat image = cv::Mat(sourceImage.height(), sourceImage.width(), CV_8UC4, imageDataPointer).clone();

            // Reading all formats here; passing only the formats you use is faster and more reliable
            const auto linearResults = barcodeDetector.readLinearCodes(frame2d);
            const auto matrixResults = barcodeDetector.readMatrixCodes(frame2d);

            drawOverlay(image, linearResults, matrixResults);

            cv::imshow(windowName, image);
            const auto key = cv::waitKey(1) & 0xFF;
            if(key == 'q' || key == 27) // 27 == Esc
            {
                break;
            }
            if(cv::getWindowProperty(windowName, cv::WND_PROP_VISIBLE) < 1)
            {
                break;
            }
        }
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
