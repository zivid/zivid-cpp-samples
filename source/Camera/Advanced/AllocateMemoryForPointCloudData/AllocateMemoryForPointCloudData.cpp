/*
Two methods to copy point cloud data from GPU memory to CPU memory, to be consumed by OpenCV.
With the first method, the memory is allocated by user (OpenCV) based on the camera resolution before any capture
occurs. With the second method, the memory is allocated post-capture by Zivid SDK based on the point cloud resolution.

Note: This example uses experimental SDK features, which may be modified, moved, or deleted in the future without
notice.
*/

#include <Zivid/Experimental/SettingsInfo.h>
#include <Zivid/Zivid.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Creating settings" << std::endl;
        auto settings = Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} } };

        std::cout << "Getting camera resolution" << std::endl;
        const auto resolution = Zivid::Experimental::SettingsInfo::resolution(camera.info(), settings);

        std::cout << "Camera resolution:" << std::endl;
        std::cout << "Height: " << resolution.height() << std::endl;
        std::cout << "Width: " << resolution.width() << std::endl;

        // Copy selected data from GPU to system memory (User-allocated)

        std::cout << "Allocating the necessary storage with OpenCV API based on resolution info before any capturing"
                  << std::endl;
        auto bgraUserAllocated = cv::Mat(resolution.height(), resolution.width(), CV_8UC4);

        std::cout << "Capturing frame" << std::endl;
        auto frame = camera.capture(settings);
        auto pointCloud = frame.pointCloud();

        std::cout << "Copying data with Zivid API from the GPU into the memory location allocated by OpenCV"
                  << std::endl;
        pointCloud.copyData(reinterpret_cast<Zivid::ColorBGRA *>(bgraUserAllocated.data));

        std::cout << "Displaying image" << std::endl;
        cv::imshow("BGRA image User Allocated", bgraUserAllocated);
        cv::waitKey(0);

        // Copy selected data from GPU to system memory (Zivid-allocated)

        std::cout << "Capturing frame" << std::endl;
        frame = camera.capture(settings);
        pointCloud = frame.pointCloud();

        std::cout << "Copying colors with Zivid API from GPU to CPU" << std::endl;
        auto colors = pointCloud.copyColorsBGRA();

        std::cout << "Casting the data pointer as a void*, since this is what the OpenCV matrix constructor requires."
                  << std::endl;

        // The cast for colors.data() is required because the cv::Mat constructor requires non-const void *.
        // It does not actually mutate the data, it only adds an OpenCV header to the matrix. We then protect
        // our own instance with const.
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
        auto *dataPtrZividAllocated = const_cast<void *>(static_cast<const void *>(colors.data()));

        std::cout << "Wrapping this block of data in an OpenCV matrix. This is possible since the layout of \n"
                  << "Zivid::ColorBGRA exactly matches the layout of CV_8UC4. No copying occurs in this step."
                  << std::endl;
        const cv::Mat bgraZividAllocated(colors.height(), colors.width(), CV_8UC4, dataPtrZividAllocated);

        std::cout << "Displaying image" << std::endl;
        cv::imshow("BGRA image Zivid Allocated", bgraZividAllocated);
        cv::waitKey(0);
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
