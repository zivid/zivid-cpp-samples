/*
Capture 2D image with gamma correction.
*/

#include <Zivid/Zivid.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <clipp.h>

#include <iostream>

namespace
{
    double readGamma(int argc, char **argv)
    {
        double gamma{};
        auto cli = clipp::group(clipp::value("gamma", gamma).doc("Gamma correction value"));
        if(!parse(argc, argv, cli))
        {
            auto fmt = clipp::doc_formatting{};
            std::cout << "SYNOPSIS:" << std::endl;
            std::cout << clipp::usage_lines(cli, "GammaCorrection", fmt) << std::endl;
            std::cout << "OPTIONS:" << std::endl;
            std::cout << clipp::documentation(cli) << std::endl;
            throw std::runtime_error{ "Gamma is not provided" };
        }

        return gamma;
    }

    cv::Mat captureBGRAImage(Zivid::Camera &camera, const double gamma)
    {
        std::cout << "Configuring settings" << std::endl;
        const auto settings2D = Zivid::Settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} },
                                                   Zivid::Settings2D::Processing::Color::Gamma{ gamma } };

        std::cout << "Capturing 2D frame" << std::endl;
        const auto frame2D = camera.capture2D(settings2D);
        const auto image = frame2D.imageBGRA();

        auto imageDataPointer = const_cast<void *>(static_cast<const void *>(image.data()));

        cv::Mat bgra = cv::Mat(image.height(), image.width(), CV_8UC4, imageDataPointer).clone();

        return bgra;
    }


    cv::Mat combineImages(const cv::Mat &imageOne, const cv::Mat &imageTwo)
    {
        cv::Mat combinedImage;
        int height = imageOne.rows;
        int width = imageOne.cols;
        cv::hconcat(
            imageOne(cv::Range(0, height), cv::Range(0, width / 2)),
            imageTwo(cv::Range(0, height), cv::Range(width / 2, width)),
            combinedImage);

        return combinedImage;
    }


    void displayBGRA(const cv::Mat &bgra, const std::string &bgraName)
    {
        cv::namedWindow(bgraName, cv::WINDOW_NORMAL);
        cv::imshow(bgraName, bgra);
        std::cout << "Press any key to continue" << std::endl;
        cv::waitKey(0);
    }

} // namespace

int main(int argc, char **argv)
{
    try
    {
        Zivid::Application zivid;
        const double gamma = readGamma(argc, argv);

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Capturing without gamma correction" << std::endl;
        cv::Mat bgraOriginal = captureBGRAImage(camera, 1.0);
        cv::imwrite("Original.jpg", bgraOriginal);
        std::cout << "Capturing with gamma correction: " << gamma << std::endl;
        cv::Mat bgraAdjusted = captureBGRAImage(camera, gamma);
        cv::imwrite("Adjusted.jpg", bgraAdjusted);

        std::cout << "Displaying color image before and after gamma correction: " << gamma << std::endl;
        cv::Mat combinedImage = combineImages(bgraOriginal, bgraAdjusted);
        displayBGRA(combinedImage, "Original on left, adjusted on right");
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
