/*
This example shows how to read intrinsic parameters from the Zivid camera (OpenCV model).
*/

#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <Zivid/Experimental/Calibration.h>
#include <Zivid/Experimental/SettingsInfo.h>
#include <Zivid/Zivid.h>

#include <chrono>
#include <iostream>

namespace
{
    struct CameraIntrinsicsCV
    {
        cv::Mat distortionCoefficients;
        cv::Mat cameraMatrix;
    };

    float getValueZ(const Zivid::PointZ &p)
    {
        return p.z;
    }

    bool isLesserOrNan(const Zivid::PointZ &a, const Zivid::PointZ &b)
    {
        if(std::isnan(getValueZ(a)) && std::isnan(getValueZ(b)))
        {
            return false;
        }
        return getValueZ(a) < getValueZ(b) ? true : std::isnan(getValueZ(a));
    }
    bool isGreaterOrNaN(const Zivid::PointZ &a, const Zivid::PointZ &b)
    {
        if(std::isnan(getValueZ(a)) && std::isnan(getValueZ(b)))
        {
            return false;
        }
        return getValueZ(a) > getValueZ(b) ? true : std::isnan(getValueZ(a));
    }

    std::string get_filename_base(Zivid::Camera &camera, const Zivid::Settings &settings)
    {
        std::stringstream filename_base;
        auto modelName = camera.info().modelName().toString();
        modelName.erase(remove_if(modelName.begin(), modelName.end(), isspace), modelName.end());
        filename_base << modelName << "_" << camera.info().serialNumber() << "_";
        filename_base << "_Gauss-" << settings.processing().filters().smoothing().gaussian().isEnabled();
        if(settings.processing().filters().smoothing().gaussian().isEnabled()
           == Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes)
        {
            filename_base << "-"
                          << std::to_string(settings.processing().filters().smoothing().gaussian().sigma().value());
        }
        filename_base << "_Reflection-" << settings.processing().filters().reflection().removal().isEnabled();
        filename_base << "_CDCorrection-"
                      << settings.processing().filters().experimental().contrastDistortion().correction().isEnabled();
        if(settings.processing().filters().experimental().contrastDistortion().correction().isEnabled()
           == Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Enabled::yes)
        {
            filename_base << "-"
                          << std::to_string(settings.processing()
                                                .filters()
                                                .experimental()
                                                .contrastDistortion()
                                                .correction()
                                                .strength()
                                                .value());
        }
        filename_base << "_CDRemoval-"
                      << settings.processing().filters().experimental().contrastDistortion().removal().isEnabled();
        if(settings.processing().filters().experimental().contrastDistortion().removal().isEnabled()
           == Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Enabled::yes)
        {
            filename_base << "-"
                          << std::to_string(settings.processing()
                                                .filters()
                                                .experimental()
                                                .contrastDistortion()
                                                .removal()
                                                .threshold()
                                                .value());
        }
        return filename_base.str();
    }

    CameraIntrinsicsCV reformatCameraIntrinsics(const Zivid::CameraIntrinsics &cameraIntrinsics)
    {
        cv::Mat distortionCoefficients(1, 5, CV_64FC1, cv::Scalar(0));
        cv::Mat cameraMatrix(3, 3, CV_64FC1, cv::Scalar(0));

        distortionCoefficients.at<double>(0, 0) = cameraIntrinsics.distortion().k1().value();
        distortionCoefficients.at<double>(0, 1) = cameraIntrinsics.distortion().k2().value();
        distortionCoefficients.at<double>(0, 2) = cameraIntrinsics.distortion().p1().value();
        distortionCoefficients.at<double>(0, 3) = cameraIntrinsics.distortion().p2().value();
        distortionCoefficients.at<double>(0, 4) = cameraIntrinsics.distortion().k3().value();

        cameraMatrix.at<double>(0, 0) = cameraIntrinsics.cameraMatrix().fx().value();
        cameraMatrix.at<double>(0, 2) = cameraIntrinsics.cameraMatrix().cx().value();
        cameraMatrix.at<double>(1, 1) = cameraIntrinsics.cameraMatrix().fy().value();
        cameraMatrix.at<double>(1, 2) = cameraIntrinsics.cameraMatrix().cy().value();
        cameraMatrix.at<double>(2, 2) = 1;

        CameraIntrinsicsCV cameraIntrinsicsCV;
        cameraIntrinsicsCV.distortionCoefficients = distortionCoefficients;
        cameraIntrinsicsCV.cameraMatrix = cameraMatrix;

        return cameraIntrinsicsCV;
    }

    cv::Mat get_offsets_as_heatmap(cv::Mat &pixel_offsets)
    {
        double xmin, xmax;
        cv::minMaxIdx(pixel_offsets, &xmin, &xmax);
        // std::cout << "Pixel offset range: " << std::to_string(xmin) << " to " << std::to_string(xmax) << std::endl;
        cv::Mat adjMap;
        //float scale = 255 / (xmax - xmin);
        //pixel_offsets.convertTo(adjMap, CV_8UC1, scale, -xmin * scale);
        // Use fixed scale: xMin = -128, xMax = +128. This allows comparison between heatmaps
        //pixel_offsets.convertTo(adjMap, CV_8UC1, 1, 128);
        // Use fixed scale: xMin = -40, xMax = +40. This allows comparison between heatmaps
        xmin = -20;
        xmax = 20;
        float scale = 255 / (xmax - xmin);
        pixel_offsets.convertTo(adjMap, CV_8UC1, scale, -xmin * scale);
        cv::Mat pixel_offsets_as_heatmap;
        cv::applyColorMap(adjMap, pixel_offsets_as_heatmap, cv::COLORMAP_HSV);
        //cv::applyColorMap(adjMap, pixel_offsets_as_heatmap, cv::COLORMAP_PARULA);
        cv::Mat mat1 = pixel_offsets.clone();
        cv::Mat mat2 = pixel_offsets.clone();
        cv::patchNaNs(mat1, 128);
        cv::patchNaNs(mat2, 200);
        cv::Mat nanmask = (mat1 == 128 & mat2 == 200);
        pixel_offsets_as_heatmap.setTo(cv::Scalar(0, 0, 0), nanmask);
        return pixel_offsets_as_heatmap;
    }

    void visualize_pixel_offsets(cv::Mat &pixel_offsets, std::string &filename_base)
    {
        double xmin, xmax;
        cv::minMaxIdx(pixel_offsets, &xmin, &xmax);
        std::cout << "Pixel offset range (x and y combined): " << std::to_string(xmin) << " to " << std::to_string(xmax)
                  << std::endl;

        std::cout << "Visualize pixel offsets as heatmaps" << std::endl;
        cv::Mat pixel_offset_vector[2];
        cv::split(pixel_offsets, pixel_offset_vector);
        auto x_offset_heatmap = get_offsets_as_heatmap(pixel_offset_vector[1]);
        std::stringstream x_title;
        x_title << "Pixel offset in X for " << filename_base;
        cv::imshow(x_title.str(), x_offset_heatmap);
        std::stringstream x_filename;
        x_filename << filename_base << "_x_offset_heatmap.png";
        cv::imwrite(x_filename.str(), x_offset_heatmap);
        auto y_offset_heatmap = get_offsets_as_heatmap(pixel_offset_vector[0]);
        std::stringstream y_title;
        y_title << "Pixel offset in Y for " << filename_base;
        cv::imshow(y_title.str(), y_offset_heatmap);
        std::stringstream y_filename;
        y_filename << filename_base << "_y_offset_heatmap.png";
        cv::imwrite(y_filename.str(), y_offset_heatmap);
        cv::waitKey(0);
        std::cout << "Press any key to continute..." << std::endl;
    }

    void calculate_pixel_offset_statistics(cv::Mat &pixel_offsets, std::string &filename_base)
    {
        double xmin, xmax;
        cv::minMaxIdx(pixel_offsets, &xmin, &xmax);
        std::cout << "Max negative pixel offset               : " << xmin << std::endl;
        std::cout << "Max positive pixel offset               : " << xmax << std::endl;
        int threshold = 1;
        auto number_of_pixel_offsets_above_threshold = cv::countNonZero(abs(pixel_offsets) > 1);
        std::cout << "Number of absolute pixel offsets above " << std::to_string(threshold) << ": "
                  << std::to_string(number_of_pixel_offsets_above_threshold) << std::endl;
        cv::Mat mean, stddev;
        cv::Mat mat1 = pixel_offsets.clone();
        cv::Mat mat2 = pixel_offsets.clone();
        cv::patchNaNs(mat1, 128);
        cv::patchNaNs(mat2, 200);
        cv::Mat masko = 255 - (mat1 == 128 & mat2 == 200);
        cv::meanStdDev(pixel_offsets, mean, stddev, masko);
        std::cout << "Average pixel offset                    : " << mean << std::endl;
        cv::meanStdDev(abs(pixel_offsets), mean, stddev, masko);
        std::cout << "Average absolute pixel offset           : " << mean << std::endl;
        std::cout << "Std for absolute pixel offsets          : " << stddev << std::endl;
    }

    void capture_estimate_and_project(Zivid::Camera &camera, const Zivid::Settings &settings)
    {
        auto filename_base = get_filename_base(camera, settings);

        std::cout << std::endl << "Processing " << filename_base << std::endl;

        std::cout << "Capturing frame" << std::endl;
        const auto frame = camera.capture(settings);
        frame.save(filename_base + ".zdf");

        const auto resolution = Zivid::Experimental::SettingsInfo::resolution(camera.info(), settings);
        auto pointCloud = frame.pointCloud();

        auto original_rgba = cv::Mat(resolution.height(), resolution.width(), CV_8UC4, cv::Scalar(255, 255, 255, 0));
        pointCloud.copyData(reinterpret_cast<Zivid::ColorRGBA *>(original_rgba.data));

        std::cout << "Getting camera intrinsics" << std::endl;
        auto intrinsics = Zivid::Experimental::Calibration::estimateIntrinsics(frame);
        intrinsics.save(filename_base + ".yml");

        const auto cameraIntrinsticsCV = reformatCameraIntrinsics(intrinsics);
        const auto distortionCoefficients = cameraIntrinsticsCV.distortionCoefficients;
        const auto cameraMatrix = cameraIntrinsticsCV.cameraMatrix;

        std::cout << "Getting 3D points" << std::endl;
        const auto ordered_xyz = cv::Mat(resolution.height(), resolution.width(), CV_32FC3);
        pointCloud.copyData(reinterpret_cast<Zivid::PointXYZ *>(ordered_xyz.data));
        const auto rvec = cv::Mat::zeros(cv::Size(1, 3), CV_32FC1);
        const auto tvec = cv::Mat::zeros(cv::Size(1, 3), CV_32FC1);

        std::cout << "Projecting points from 3D to 2D" << std::endl;
        const auto unordered_xyz = ordered_xyz.reshape(0, resolution.height() * resolution.width());
        auto npoints = unordered_xyz.checkVector(3);
        auto depth = unordered_xyz.depth();
        if(npoints < 0)
        {
            std::cout << "No points in unordered_xyz" << std::endl;
        }
        auto unordered_image_points = cv::Mat(cv::Size(resolution.height(), resolution.width()), CV_32FC2);
        cv::projectPoints(unordered_xyz, rvec, tvec, cameraMatrix, distortionCoefficients, unordered_image_points);
        const auto ordered_image_points = unordered_image_points.reshape(0, resolution.height());

        std::cout << "Recreating 2D image and calculating pixel offsets" << std::endl;
        auto projected_rgb = cv::Mat(resolution.height(), resolution.width(), CV_8UC3, cv::Scalar(255, 255, 255));
        int oversampling_factor = 3;
        auto oversampled_rgb = cv::Mat(resolution.height() * oversampling_factor,
                                       resolution.width() * oversampling_factor,
                                       CV_8UC3,
                                       cv::Scalar(255, 255, 255));
        auto pixel_offsets = cv::Mat(resolution.height(), resolution.width(), CV_32FC2);
        for(int row = 0; row < resolution.height(); row++)
        {
            for(int col = 0; col < resolution.width(); col++)
            {
                auto estimated_pixel_row = ordered_image_points.at<cv::Point2f>(row, col).y;
                auto estimated_pixel_col = ordered_image_points.at<cv::Point2f>(row, col).x;
                pixel_offsets.at<cv::Vec2f>(row, col) = cv::Vec2f(row - estimated_pixel_row, col - estimated_pixel_col);
                if(estimated_pixel_row > resolution.height() || estimated_pixel_row < 0)
                {
                    // if(estimated_pixel_row > (resolution.height() + 1) || estimated_pixel_row < -1)
                    //{
                    //    std::cout << "Invalid pixel index: " << std::to_string(estimated_pixel_row) << " > "
                    //              << std::to_string(resolution.height()) << std::endl;
                    //}
                }
                else if(estimated_pixel_col > resolution.width() || estimated_pixel_col < 0)
                {
                    // if(estimated_pixel_col > (resolution.width() + 1) || estimated_pixel_col < -1)
                    //{
                    //    std::cout << "Invalid pixel index: " << std::to_string(estimated_pixel_col) << " > "
                    //              << std::to_string(resolution.width()) << std::endl;
                    //}
                }
                else if(!std::isnan(estimated_pixel_row) || !std::isnan(estimated_pixel_col))
                {
                    projected_rgb.at<cv::Vec3b>(int(estimated_pixel_row), int(estimated_pixel_col))[0] =
                        original_rgba.at<cv::Vec4b>(row, col)[0];
                    projected_rgb.at<cv::Vec3b>(int(estimated_pixel_row), int(estimated_pixel_col))[1] =
                        original_rgba.at<cv::Vec4b>(row, col)[1];
                    projected_rgb.at<cv::Vec3b>(int(estimated_pixel_row), int(estimated_pixel_col))[2] =
                        original_rgba.at<cv::Vec4b>(row, col)[2];
                    // True pixel is set to black.If projection is correct, then this pixel is overwritten.
                    // If not, then we'll see some black in that area
                    oversampled_rgb.at<cv::Vec3b>(int(row * oversampling_factor), int(col * oversampling_factor)) =
                        cv::Vec3b(0, 0, 0);
                    oversampled_rgb.at<cv::Vec3b>(int(estimated_pixel_row * oversampling_factor),
                                                  int(estimated_pixel_col * oversampling_factor))[0] =
                        original_rgba.at<cv::Vec4b>(row, col)[0];
                    oversampled_rgb.at<cv::Vec3b>(int(estimated_pixel_row * oversampling_factor),
                                                  int(estimated_pixel_col * oversampling_factor))[1] =
                        original_rgba.at<cv::Vec4b>(row, col)[1];
                    oversampled_rgb.at<cv::Vec3b>(int(estimated_pixel_row * oversampling_factor),
                                                  int(estimated_pixel_col * oversampling_factor))[2] =
                        original_rgba.at<cv::Vec4b>(row, col)[2];
                }
            }
        }

        //std::stringstream org_title;
        //org_title << "Original RGB for " << filename_base;
        // cv::imshow(org_title.str(), original_rgba);

        cv::Mat pixel_offset_vector[2];
        cv::split(pixel_offsets, pixel_offset_vector);
        std::cout << "Statistics for row (y)" << std::endl;
        calculate_pixel_offset_statistics(pixel_offset_vector[0], filename_base);
        std::cout << "Statistics for col (x)" << std::endl;
        calculate_pixel_offset_statistics(pixel_offset_vector[1], filename_base);
        
        visualize_pixel_offsets(pixel_offsets, filename_base);
    }

} // namespace

int main()
{
    try
    {
        Zivid::Application zivid;

        std::cout << "Connecting to camera" << std::endl;
        auto camera = zivid.connectCamera();

        //// the filecamera file is in zivid sample data. see instructions in readme.md
        // const auto fileCamera = std::string(ZIVID_SAMPLE_DATA_DIR) + "/FileCameraZividOne.zfc";

        // std::cout << "creating virtual camera using file: " << fileCamera << std::endl;
        // auto camera = zivid.createFileCamera(fileCamera);

        std::cout << "Configuring settings" << std::endl;
        std::vector<Zivid::Settings> settings_list = std::vector<Zivid::Settings>{
            Zivid::Settings{
                Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{
                    Zivid::Settings::Acquisition::Brightness{ 1.0 },
                    Zivid::Settings::Acquisition::ExposureTime{ std::chrono::microseconds{ 20000 } } } },
                Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::no,
                Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::no,
                Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Enabled::no,
                Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Enabled::no,
            },
            Zivid::Settings{
                Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{
                    Zivid::Settings::Acquisition::Brightness{ 1.0 },
                    Zivid::Settings::Acquisition::ExposureTime{ std::chrono::microseconds{ 20000 } } } },
                Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
                Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{ 1.5 },
                Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::no,
                Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Enabled::no,
                Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Enabled::no,
            },
            Zivid::Settings{
                Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{
                    Zivid::Settings::Acquisition::Brightness{ 1.0 },
                    Zivid::Settings::Acquisition::ExposureTime{ std::chrono::microseconds{ 20000 } } } },
                Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
                Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{ 5 },
                Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::no,
                Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Enabled::no,
                Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Enabled::no,
            },
            Zivid::Settings{
                Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{
                    Zivid::Settings::Acquisition::Brightness{ 1.0 },
                    Zivid::Settings::Acquisition::ExposureTime{ std::chrono::microseconds{ 20000 } } } },
                Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::no,
                Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes,
                Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Enabled::no,
                Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Enabled::no,
            },
            Zivid::Settings{
                Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{
                    Zivid::Settings::Acquisition::Brightness{ 1.0 },
                    Zivid::Settings::Acquisition::ExposureTime{ std::chrono::microseconds{ 20000 } } } },
                Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::no,
                Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::no,
                Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Enabled::yes,
                Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Strength{ 0.4 },
                Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Enabled::no,
            },
            Zivid::Settings{
                Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{
                    Zivid::Settings::Acquisition::Brightness{ 1.0 },
                    Zivid::Settings::Acquisition::ExposureTime{ std::chrono::microseconds{ 20000 } } } },
                Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::no,
                Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::no,
                Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Enabled::yes,
                Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Strength{ 0.8 },
                Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Enabled::no,
            },
            Zivid::Settings{
                Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{
                    Zivid::Settings::Acquisition::Brightness{ 1.0 },
                    Zivid::Settings::Acquisition::ExposureTime{ std::chrono::microseconds{ 20000 } } } },
                Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::no,
                Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::no,
                Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Enabled::yes,
                Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Strength{ 0.4 },
                Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Enabled::yes,
                Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Threshold{ 0.8 },
            },
        };
        for(auto settings : settings_list)
        {
            capture_estimate_and_project(camera, settings);
        }
    }
    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }
}
