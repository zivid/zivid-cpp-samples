/*
Capture 2D image with gamma correction.
*/

#include <Zivid/Zivid.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

namespace
{
    double readGamma(int argc, char **argv)
    {
        if(argc < 2)
        {
            throw std::runtime_error("Gamma is not provided");
        }

        return std::stod(argv[1]);
    }

    cv::Mat imageToBGR(const Zivid::Image<Zivid::ColorRGBA> &image)
    {
        // The cast for image.data() is required because the cv::Mat constructor requires non-const void *.
        // It does not actually mutate the data, it only adds an OpenCV header to the matrix. We then protect
        // our own instance with const.
        const cv::Mat rgbaMat(
            image.height(),
            image.width(),
            CV_8UC4, // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
            const_cast<void *>(static_cast<const void *>(image.data())));
        cv::Mat bgr;
        cv::cvtColor(rgbaMat, bgr, cv::COLOR_RGBA2BGR);

        return bgr;
    }

    cv::Mat captureBGRImage(Zivid::Camera &camera, const double gamma)
    {
        std::cout << "Configuring settings" << std::endl;
        const auto settings2D = Zivid::Settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} },
                                                   Zivid::Settings2D::Processing::Color::Gamma{ gamma } };

        std::cout << "Capturing 2D frame" << std::endl;
        const auto frame2D = camera.capture(settings2D);
        const auto image = frame2D.imageRGBA();
        auto bgr = imageToBGR(image);

        return bgr;
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


    void displayBGR(const cv::Mat &bgr, const std::string &bgrName)
    {
        cv::namedWindow(bgrName, cv::WINDOW_NORMAL);
        cv::imshow(bgrName, bgr);
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
        cv::Mat bgrOriginal = captureBGRImage(camera, 1.0);
        cv::imwrite("Original.jpg", bgrOriginal);
        std::cout << "Capturing with gamma correction: " << gamma << std::endl;
        cv::Mat bgrAdjusted = captureBGRImage(camera, gamma);
        cv::imwrite("Adjusted.jpg", bgrAdjusted);

        std::cout << "Displaying color image before and after gamma correction: " << gamma << std::endl;
        cv::Mat combinedImage = combineImages(bgrOriginal, bgrAdjusted);
        displayBGR(combinedImage, "Original on left, adjusted on right");
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
