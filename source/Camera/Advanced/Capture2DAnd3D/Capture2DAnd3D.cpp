/*
Capture 2D and then 3D using various capture strategies, optimizing for both 2D quality and 2D acquisition speed.
*/

#include <Zivid/Zivid.h>

#include <clipp.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <map>
#include <thread>

namespace
{
    cv::Mat mapBGR(const Zivid::Settings::Sampling::Pixel &pixelsToSample, const cv::Mat &fullResolutionBGR)
    {
        std::cout << "Pixels to sample: " << pixelsToSample << std::endl;
        const int subsampleDivider =
            (pixelsToSample.value() == Zivid::Settings::Sampling::Pixel::ValueType::all) ? 1 : 2;
        int offset = (pixelsToSample.value() == Zivid::Settings::Sampling::Pixel::ValueType::blueSubsample2x2) ? 0 : 1;
        cv::Mat
            mappedBGR(fullResolutionBGR.rows / subsampleDivider, fullResolutionBGR.cols / subsampleDivider, CV_8UC3);
        std::cout << "Mapped width: " << mappedBGR.cols << ", height: " << mappedBGR.rows << std::endl;
        for(size_t row = 0; row < static_cast<size_t>(fullResolutionBGR.rows - offset); row += subsampleDivider)
        {
            for(size_t col = 0; col < static_cast<size_t>(fullResolutionBGR.cols - offset); col += subsampleDivider)
            {
                mappedBGR.at<cv::Vec3b>(row / subsampleDivider, col / subsampleDivider) =
                    fullResolutionBGR.at<cv::Vec3b>(row + offset, col + offset);
            }
        }
        return mappedBGR;
    }

    Zivid::Settings::Sampling::Pixel stringToPixelSetting(const std::string &pixelsToSample)
    {
        const auto validValues = Zivid::Settings::Sampling::Pixel::validValues();
        for(const auto &value : validValues)
        {
            const auto pixelSetting = Zivid::Settings::Sampling::Pixel{ value };
            if(pixelSetting.toString() == pixelsToSample)
            {
                return pixelSetting;
            }
        }

        std::stringstream errorMsg;
        errorMsg << "Invalid pixel value. Use one of:";
        for(const auto &value : validValues)
        {
            errorMsg << " " << Zivid::Settings::Sampling::Pixel{ value }.toString();
        }
        throw std::runtime_error(errorMsg.str());
    }

    void displayBGR(const std::vector<cv::Mat> &bgrs, const std::vector<std::string> &bgrNames)
    {
        if(bgrs.empty() || bgrNames.empty() || bgrs.size() != bgrNames.size())
        {
            std::cerr << "Error: Invalid input data." << std::endl;
            return;
        }

        const int titleMargin = 60;
        int totalRows = bgrs[0].rows;
        const int separationMargin = 40;
        int combinedWidth = 0;
        for(const auto &bgr : bgrs)
        {
            totalRows = std::max(totalRows, bgr.rows);
            combinedWidth += bgr.cols + separationMargin;
        }
        totalRows += titleMargin;

        cv::Mat combinedImage(totalRows, combinedWidth, bgrs[0].type());

        int x_offset = 0;
        for(size_t i = 0; i < bgrs.size(); ++i)
        {
            cv::Mat roi(combinedImage, cv::Rect(x_offset, titleMargin, bgrs[i].cols, bgrs[i].rows));
            bgrs[i].copyTo(roi);

            x_offset += bgrs[i].cols + separationMargin;

            cv::putText(
                combinedImage,
                bgrNames[i],
                cv::Point(x_offset - bgrs[i].cols, 20),
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(255, 255, 255),
                1,
                cv::LINE_AA);
        }
        cv::imshow("Combined Image", combinedImage);
        cv::waitKey(0);
    }

    void displayPointCloud(const Zivid::PointCloud &pointCloud, const cv::Mat &mappedBGR)
    {
        const auto xyz = pointCloud.copyData<Zivid::PointXYZ>();

        auto pclPointCloud = pcl::PointCloud<pcl::PointXYZRGB>();
        pclPointCloud.width = xyz.width();
        pclPointCloud.height = xyz.height();
        pclPointCloud.is_dense = false;
        pclPointCloud.points.resize(xyz.size());
        for(size_t i = 0; i < xyz.size(); ++i)
        {
            pclPointCloud.points[i].x = xyz(i).x; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pclPointCloud.points[i].y = xyz(i).y; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pclPointCloud.points[i].z = xyz(i).z; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pclPointCloud.points[i].b =
                mappedBGR.at<cv::Vec3b>(i)[0]; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pclPointCloud.points[i].g =
                mappedBGR.at<cv::Vec3b>(i)[1]; // NOLINT(cppcoreguidelines-pro-type-union-access)
            pclPointCloud.points[i].r =
                mappedBGR.at<cv::Vec3b>(i)[2]; // NOLINT(cppcoreguidelines-pro-type-union-access)
        }

        auto viewer = pcl::visualization::PCLVisualizer("Viewer");

        int viewRgb(0);
        viewer.createViewPort(0.0, 0.0, 1.0, 1.0, viewRgb);
        viewer.addText("Cloud RGB", 0, 0, "Mapped RGB", viewRgb);
        viewer.addPointCloud<pcl::PointXYZRGB>(pclPointCloud.makeShared(), "cloud", viewRgb);

        viewer.setCameraPosition(0, 0, -100, 0, -1, 0);

        std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
        std::cout << "Press q to exit the viewer application" << std::endl;
        while(!viewer.wasStopped())
        {
            viewer.spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
} // namespace

int main(int argc, char **argv)
{
    try
    {
        std::stringstream pixelSettingDoc;
        pixelSettingDoc << "Select which pixels to sample. Valid options:";
        const auto validValues = Zivid::Settings::Sampling::Pixel::validValues();
        for(const auto &value : validValues)
        {
            pixelSettingDoc << " '" << Zivid::Settings::Sampling::Pixel{ value }.toString() << "'";
        }

        auto pixelsToSample = Zivid::Settings::Sampling::Pixel::all;
        auto cli =
            ((clipp::option("-p", "--pixels-to-sample")
              & clipp::value("pixelsToSample") >>
                    [&](const std::string &value) { pixelsToSample = stringToPixelSetting(value); })
             % pixelSettingDoc.str());

        if(!parse(argc, argv, cli))
        {
            auto fmt = clipp::doc_formatting{};
            //.alternatives_min_split_size(1).surround_labels("\"", "\"");
            std::cout << "SYNOPSIS:" << std::endl;
            std::cout << clipp::usage_lines(cli, "Capture2DAnd3D", fmt) << std::endl;
            std::cout << "OPTIONS:" << std::endl;
            std::cout << clipp::documentation(cli) << std::endl;
            throw std::runtime_error{ "Invalid usage" };
        }

        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        std::cout << "Configuring 2D settings" << std::endl;
        const auto settings2D =
            Zivid::Settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } };

        std::cout << "Configuring 3D settings" << std::endl;
        auto settings = Zivid::Settings{
            Zivid::Settings::Experimental::Engine::phase,
            Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} },
            Zivid::Settings::Sampling::Pixel{ pixelsToSample },
            Zivid::Settings::Sampling::Color{ Zivid::Settings::Sampling::Color::disabled },
        };

        const auto cameraModel = camera.info().model().value();
        if(pixelsToSample == Zivid::Settings::Sampling::Pixel::all
           && (cameraModel == Zivid::CameraInfo::Model::ValueType::zivid2PlusM130
               || cameraModel == Zivid::CameraInfo::Model::ValueType::zivid2PlusM60
               || cameraModel == Zivid::CameraInfo::Model::ValueType::zivid2PlusL110))
        {
            // For 2+, we must lower Brightness from the default 2.5 to 2.2, when using `all` mode.
            // This code can be removed by changing the Config.yml option 'Camera/Power/Limit'.
            for(auto &a : settings.acquisitions())
            {
                a.set(Zivid::Settings::Acquisition::Brightness{ 2.2 });
            }
        }

        std::cout << "Capturing 2D frame" << std::endl;
        const auto frame2D = camera.capture(settings2D);
        std::cout << "Getting BGRA image" << std::endl;
        const auto image = frame2D.imageBGRA();
        const cv::Mat fullResolutionBGRA(
            image.height(),
            image.width(),
            CV_8UC4, // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast)
            const_cast<void *>(static_cast<const void *>(image.data())));
        cv::Mat fullResolutionBGR;
        cv::cvtColor(fullResolutionBGRA, fullResolutionBGR, cv::COLOR_BGRA2BGR);
        const auto mappedBGR = (pixelsToSample.value() == Zivid::Settings::Sampling::Pixel::ValueType::all)
                                   ? fullResolutionBGR
                                   : mapBGR(pixelsToSample, fullResolutionBGR);
        if(pixelsToSample.value() == Zivid::Settings::Sampling::Pixel::ValueType::all)
        {
            displayBGR({ fullResolutionBGR }, { "Full resolution 2D" });
        }
        else
        {
            std::ostringstream mappedTitle;
            mappedTitle << pixelsToSample << " RGB from 2D capture";
            displayBGR({ fullResolutionBGR, mappedBGR }, { "Full resolution 2D", mappedTitle.str() });
        }

        std::cout << "Capturing frame" << std::endl;

        const auto frame = camera.capture(settings);
        const auto pointCloud = frame.pointCloud();
        std::cout << "Visualizing point cloud" << std::endl;
        displayPointCloud(pointCloud, mappedBGR);
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
