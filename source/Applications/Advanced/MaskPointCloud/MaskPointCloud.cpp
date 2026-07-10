/*
Mask point cloud from a ZDF file using the Mask API and visualize it with Zivid::Visualizer.

This example shows how to:
1. Create a mask using OpenCV rectangle drawing
2. Apply the mask to a point cloud using the Mask API
3. Visualize the results using Zivid::Visualizer instead of PCL

The ZDF file for this sample can be found under the main instructions for Zivid samples.
*/

#include <Zivid/Mask.h>
#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <limits>

namespace
{
    void visualizePointCloudWithTitle(const Zivid::PointCloud &pointCloud, const std::string &title)
    {
        std::cout << "Displaying " << title << std::endl;
        std::cout << "Press q to exit the viewer application" << std::endl;

        Zivid::Visualization::Visualizer visualizer;
        visualizer.setWindowTitle(title);
        visualizer.show(pointCloud);
        visualizer.show();
        visualizer.resetToFit();
        visualizer.run();
    }

    cv::Mat pointCloudToCvZ(const Zivid::PointCloud &pointCloud)
    {
        const auto data = pointCloud.copyPointsZ();
        const int height = static_cast<int>(data.height());
        const int width = static_cast<int>(data.width());

        // Getting min and max values for Z
        float zMax = -std::numeric_limits<float>::max();
        float zMin = std::numeric_limits<float>::max();
        for(int i = 0; i < height; i++)
        {
            for(int j = 0; j < width; j++)
            {
                const auto z = data(i, j).z;
                if(!std::isnan(z))
                {
                    zMax = std::max(zMax, z);
                    zMin = std::min(zMin, z);
                }
            }
        }

        // Filling in OpenCV matrix with the cloud data
        cv::Mat z(height, width, CV_8UC1, cv::Scalar(0)); // NOLINT(hicpp-signed-bitwise)
        for(int i = 0; i < height; i++)
        {
            for(int j = 0; j < width; j++)
            {
                const auto zValue = data(i, j).z;
                if(!std::isnan(zValue) && zMax > zMin)
                {
                    const auto normalizedZ = static_cast<uint8_t>(255.0F * (zValue - zMin) / (zMax - zMin));
                    z.at<uint8_t>(i, j) = normalizedZ;
                }
            }
        }

        // Applying color map
        cv::Mat zColorMap;
        cv::applyColorMap(z, zColorMap, cv::COLORMAP_VIRIDIS);

        // Setting invalid points (nan) to black
        for(int i = 0; i < height; i++)
        {
            for(int j = 0; j < width; j++)
            {
                const auto zValue = data(i, j).z;
                if(std::isnan(zValue))
                {
                    zColorMap.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
                }
            }
        }
        return zColorMap;
    }

    Zivid::Mask createRectangularMask(const Zivid::Resolution &resolution, int pixelsToDisplay)
    {
        // Create a ones-filled mask
        Zivid::Mask mask(resolution);

        // Calculate rectangle bounds
        const int height = static_cast<int>(resolution.height());
        const int width = static_cast<int>(resolution.width());
        const int heightMin = (height - pixelsToDisplay) / 2;
        const int heightMax = (height + pixelsToDisplay) / 2;
        const int widthMin = (width - pixelsToDisplay) / 2;
        const int widthMax = (width + pixelsToDisplay) / 2;

        // Create OpenCV Mat wrapper for the mask data
        cv::Mat maskMat(height, width, CV_8UC1, mask.data());

        // Draw filled rectangle on the mask to unmask the central region
        cv::rectangle(
            maskMat, cv::Point(widthMin, heightMin), cv::Point(widthMax, heightMax), cv::Scalar(0), cv::FILLED);

        return mask;
    }

    void visualizeDepthMap(const Zivid::PointCloud &pointCloud, const std::string &title)
    {
        // Converting to Depth map in OpenCV format
        cv::Mat zColorMap = pointCloudToCvZ(pointCloud);
        // Visualizing Depth map
        cv::namedWindow(title, cv::WINDOW_AUTOSIZE);
        cv::imshow(title, zColorMap);
        std::cout << "Displaying depth map: " << title << std::endl;
        std::cout << "Press any key to continue..." << std::endl;
        cv::waitKey(CI_WAITKEY_TIMEOUT_IN_MS);
    }

} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::string fileName = std::string(ZIVID_SAMPLE_DATA_DIR) + "/Zivid3D.zdf";
        std::cout << "Reading ZDF frame from file: " << fileName << std::endl;
        const auto frame = Zivid::Frame(fileName);

        std::cout << "Getting point cloud from frame" << std::endl;
        auto pointCloud = frame.pointCloud();

        const int pixelsToDisplay = 300;
        std::cout << "Creating rectangular mask of central " << pixelsToDisplay << " x " << pixelsToDisplay
                  << " pixels using new Mask API." << std::endl;

        const Zivid::Resolution resolution(pointCloud.width(), pointCloud.height());
        std::cout << "Point cloud resolution: " << resolution.width() << " x " << resolution.height() << std::endl;

        // Create mask using the new Mask API
        const auto mask = createRectangularMask(resolution, pixelsToDisplay);

        std::cout << "Displaying original point cloud" << std::endl;
        visualizePointCloudWithTitle(pointCloud, "Original Point Cloud");

        std::cout << "Displaying original depth map" << std::endl;
        visualizeDepthMap(pointCloud, "Original Depth Map");

        std::cout << "Applying mask to point cloud using new Mask API" << std::endl;
        auto maskedPointCloud = pointCloud.masked(mask);

        std::cout << "Displaying masked point cloud" << std::endl;
        visualizePointCloudWithTitle(maskedPointCloud, "Masked Point Cloud");

        std::cout << "Displaying masked depth map" << std::endl;
        visualizeDepthMap(maskedPointCloud, "Masked Depth Map");

        // Demonstration: Create a mask from OpenCV Mat (third-party integration)
        std::cout << "Creating OpenCV mask and converting to Zivid::Mask" << std::endl;
        cv::Mat opencvMask =
            cv::Mat::ones(static_cast<int>(resolution.height()), static_cast<int>(resolution.width()), CV_8UC1);

        // Create a circular mask in OpenCV
        const int centerX = static_cast<int>(resolution.width()) / 2;
        const int centerY = static_cast<int>(resolution.height()) / 2;
        const int radius = pixelsToDisplay;
        cv::circle(opencvMask, cv::Point(centerX, centerY), radius, cv::Scalar(0), cv::FILLED);

        // Convert OpenCV mask to Zivid::Mask
        const Zivid::Mask circularMask(resolution, opencvMask.datastart, opencvMask.dataend);

        std::cout << "Created circular mask from OpenCV Mat with " << circularMask.size() << " pixels" << std::endl;

        std::cout << "Applying circular mask (created from OpenCV Mat)" << std::endl;
        auto circularMaskedPointCloud = pointCloud.masked(circularMask);

        std::cout << "Displaying circular masked point cloud" << std::endl;
        visualizePointCloudWithTitle(circularMaskedPointCloud, "Circular Masked Point Cloud (from OpenCV)");

        std::cout << "Displaying circular masked depth map" << std::endl;
        visualizeDepthMap(circularMaskedPointCloud, "Circular Masked Depth Map (from OpenCV)");

        std::cout << "Mask API demonstration completed successfully!" << std::endl;
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
