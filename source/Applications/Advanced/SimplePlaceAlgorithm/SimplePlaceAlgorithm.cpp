#include <Eigen/Core>
#include <iostream>
#include <open3d/Open3D.h>
//#include <pcl/common/common.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iomanip>

//typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXd;
typedef std::array<MatrixXd, 3> Matrix3Xd;
typedef std::array<MatrixXd, 5> Matrix5Xd;

namespace
{
    using HighResClock = std::chrono::high_resolution_clock;
    using Duration = std::chrono::nanoseconds;

    template<typename T>
    std::string valueToStringWithPrecision(const T& value, const size_t precision)
    {
        std::ostringstream ss;
        ss << std::setprecision(precision) << std::fixed << value;
        return ss.str();
    }

    std::string formatDuration(const Duration& duration)
    {
        return valueToStringWithPrecision(
            std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(duration).count(), 3)
            + " ms";
    }

    Matrix3Xd crop_xyz_2d(const std::map<std::string, int>& bin, const Matrix3Xd& xyz)
    {
        int row_min = bin.at("row_min");
        int col_min = bin.at("col_min");
        int rows = bin.at("row_max") - bin.at("row_min");
        int cols = bin.at("col_max") - bin.at("col_min");
        return {
            xyz[0].block(row_min, col_min, rows, cols),
            xyz[1].block(row_min, col_min, rows, cols),
            xyz[2].block(row_min, col_min, rows, cols),
        };
    }

    cv::Mat crop_bgr_2d(const std::map<std::string, int>& bin, const cv::Mat& bgr)
    {
        cv::Rect roi_rect(bin.at("col_min"), bin.at("row_min"), bin.at("col_max") - bin.at("col_min"), bin.at("row_max") - bin.at("row_min"));
        return bgr(roi_rect);
    }


    double nanmean(const MatrixXd& matrix, const int start_row, const int start_col, const int rows, const int cols)
    {
        double accumulator = 0;
        int counter = 0;
        for (int row = start_row; row < start_row + rows; row++)
        {
            for (int col = start_col; col < start_col + cols; col++)
            {
                if (!isnan(matrix(row, col)))
                {
                    accumulator += matrix(row, col);
                    counter++;
                }
            }
        }
        return accumulator / counter;
    }

    struct BinInfo {
        const Eigen::Vector2i bin_extent_pixel;
        const Eigen::Vector2d bin_extent_mm;
        const Eigen::Vector2d mm_per_pixel;
    };

    struct ObjectInfo {
        const Eigen::Vector2i object_extent_pixel;
        const Eigen::Vector2i pixel_stride;
        const double extent_divider_for_search;
    };

    BinInfo getBinInfo(const Matrix3Xd& xyz_bin)
    {
        const Eigen::Vector2d min_values(
            xyz_bin[0].minCoeff<Eigen::NaNPropagationOptions::PropagateNumbers>(),
            xyz_bin[1].minCoeff<Eigen::NaNPropagationOptions::PropagateNumbers>()
        );
        const Eigen::Vector2d max_values(
            xyz_bin[0].maxCoeff<Eigen::NaNPropagationOptions::PropagateNumbers>(),
            xyz_bin[1].maxCoeff<Eigen::NaNPropagationOptions::PropagateNumbers>()
        );
        std::cout << "Min values: " << min_values << std::endl;
        std::cout << "Max values: " << max_values << std::endl;
        const Eigen::Vector2i bin_extent_pixel(xyz_bin[0].cols(), xyz_bin[0].rows());
        const Eigen::Vector2d bin_extent_mm = max_values - min_values;
        const Eigen::Vector2d mm_per_pixel(bin_extent_mm[1] / xyz_bin[0].rows(), bin_extent_mm[0] / xyz_bin[0].cols());
        std::cout << "mm per pixel: " << mm_per_pixel << std::endl;
        return { bin_extent_pixel, bin_extent_mm, mm_per_pixel};
    }

    ObjectInfo getObjectInfo(const BinInfo& binInfo, const Eigen::Vector2d& object_extent, const double extent_divider_for_search)
    {
        const Eigen::Vector2d object_extent_pixel_d = object_extent.array() / binInfo.mm_per_pixel.array();
        const Eigen::Vector2i object_extent_pixel((int)object_extent_pixel_d[0], (int)object_extent_pixel_d[1]);
        const Eigen::Vector2d pixel_stride_d = (object_extent / extent_divider_for_search).array() / binInfo.mm_per_pixel.array();
        const Eigen::Vector2i pixel_stride((int)pixel_stride_d[0], (int)pixel_stride_d[1]);
        std::cout << "pixel stride: " << pixel_stride << std::endl;
        return { object_extent_pixel, pixel_stride, extent_divider_for_search };
    }


    Matrix5Xd find_placement_options(const Matrix3Xd& xyz_bin, const ObjectInfo& object_info)
    {
        const auto pixel_stride = object_info.pixel_stride;
        const auto object_extent_pixel = object_info.object_extent_pixel;
        const auto extent_divider_for_search = object_info.extent_divider_for_search;

        const int total_rows = xyz_bin[0].rows();
        const int total_cols = xyz_bin[0].cols();

        std::vector<int> start_rows, start_cols;
        for (int i = 0; i < total_rows - object_extent_pixel[0]; i += pixel_stride[0])
        {
            start_rows.push_back(i);
        }
        for (int i = 0; i < total_cols - object_extent_pixel[1]; i += pixel_stride[1])
        {
            start_cols.push_back(i);
        }

        const int add_other_edge = 1;

        Matrix5Xd placement_options{
            MatrixXd(start_rows.size() + add_other_edge, start_cols.size() + add_other_edge),
            MatrixXd(start_rows.size() + add_other_edge, start_cols.size() + add_other_edge),
            MatrixXd(start_rows.size() + add_other_edge, start_cols.size() + add_other_edge),
            MatrixXd(start_rows.size() + add_other_edge, start_cols.size() + add_other_edge),
            MatrixXd(start_rows.size() + add_other_edge, start_cols.size() + add_other_edge)
        };
        for (int placement_row = 0; placement_row < start_rows.size(); ++placement_row)
        {
            for (int placement_col = 0; placement_col < start_cols.size(); ++placement_col)
            {
                const int pixel_row_start = start_rows[placement_row];
                const int pixel_col_start = start_cols[placement_col];
                Matrix3Xd block{
                    xyz_bin[0].block(pixel_row_start,
                                               pixel_col_start,
                                               pixel_stride[0] * extent_divider_for_search,
                                               pixel_stride[1] * extent_divider_for_search),
                    xyz_bin[1].block(pixel_row_start,
                                               pixel_col_start,
                                               pixel_stride[0] * extent_divider_for_search,
                                               pixel_stride[1] * extent_divider_for_search),
                    xyz_bin[2].block(pixel_row_start,
                                               pixel_col_start,
                                               pixel_stride[0] * extent_divider_for_search,
                                               pixel_stride[1] * extent_divider_for_search) };
                placement_options[0](placement_row, placement_col) = nanmean(block[0], 0, 0, block[0].rows(), block[0].cols());
                placement_options[1](placement_row, placement_col) = nanmean(block[1], 0, 0, block[1].rows(), block[1].cols());
                placement_options[2](placement_row, placement_col) = block[2].block(0, 0, block[2].rows(), 1).minCoeff<Eigen::NaNPropagationOptions::PropagateNumbers>();
                placement_options[3](placement_row, placement_col) = pixel_row_start + (int)(object_extent_pixel[0] / 2);
                placement_options[4](placement_row, placement_col) = pixel_col_start + (int)(object_extent_pixel[1] / 2);
            }
        }
        if (add_other_edge > 0)
        {
            for (int placement_row = 0; placement_row < start_rows.size(); ++placement_row)
            {
                const int pixel_row_start = start_rows[placement_row];
                const int pixel_col_start = total_cols - object_extent_pixel[1];
                Matrix3Xd block{
                    xyz_bin[0].block(pixel_row_start,
                                               pixel_col_start,
                                               pixel_stride[0] * extent_divider_for_search,
                                               pixel_stride[1] * extent_divider_for_search),
                    xyz_bin[1].block(pixel_row_start,
                                               pixel_col_start,
                                               pixel_stride[0] * extent_divider_for_search,
                                               pixel_stride[1] * extent_divider_for_search),
                    xyz_bin[2].block(pixel_row_start,
                                               pixel_col_start,
                                               pixel_stride[0] * extent_divider_for_search,
                                               pixel_stride[1] * extent_divider_for_search)
                };
                placement_options[0](placement_row, start_cols.size()) = nanmean(block[0], 0, 0, block[0].rows(), block[0].cols());
                placement_options[1](placement_row, start_cols.size()) = nanmean(block[1], 0, 0, block[1].rows(), block[1].cols());
                placement_options[2](placement_row, start_cols.size()) = block[2].block(0, 0, block[2].rows(), 1).minCoeff<Eigen::NaNPropagationOptions::PropagateNumbers>();
                placement_options[3](placement_row, start_cols.size()) = pixel_row_start + (int)(object_extent_pixel[0] / 2);
                placement_options[4](placement_row, start_cols.size()) = pixel_col_start + (int)(object_extent_pixel[1] / 2);
            }

            for (int placement_col = 0; placement_col < start_cols.size(); ++placement_col)
            {
                const int pixel_row_start = total_rows - object_extent_pixel[0];
                const int pixel_col_start = start_cols[placement_col];
                Matrix3Xd block{
                    xyz_bin[0].block(pixel_row_start,
                                               pixel_col_start,
                                               pixel_stride[0] * extent_divider_for_search,
                                               pixel_stride[1] * extent_divider_for_search),
                    xyz_bin[1].block(pixel_row_start,
                                               pixel_col_start,
                                               pixel_stride[0] * extent_divider_for_search,
                                               pixel_stride[1] * extent_divider_for_search),
                    xyz_bin[2].block(pixel_row_start,
                                               pixel_col_start,
                                               pixel_stride[0] * extent_divider_for_search,
                                               pixel_stride[1] * extent_divider_for_search)
                };
                placement_options[0](start_rows.size(), placement_col) = nanmean(block[0], 0, 0, block[0].rows(), block[0].cols());
                placement_options[1](start_rows.size(), placement_col) = nanmean(block[1], 0, 0, block[1].rows(), block[1].cols());
                placement_options[2](start_rows.size(), placement_col) = block[2].block(0, 0, block[2].rows(), 1).minCoeff<Eigen::NaNPropagationOptions::PropagateNumbers>();
                placement_options[3](start_rows.size(), placement_col) = pixel_row_start + (int)(object_extent_pixel[0] / 2);
                placement_options[4](start_rows.size(), placement_col) = pixel_col_start + (int)(object_extent_pixel[1] / 2);
            }

            const int pixel_row_start = total_rows - object_extent_pixel[0];
            const int pixel_col_start = total_cols - object_extent_pixel[1];
            Matrix3Xd block{
                xyz_bin[0].block(pixel_row_start,
                                            pixel_col_start,
                                            pixel_stride[0] * extent_divider_for_search,
                                            pixel_stride[1] * extent_divider_for_search),
                xyz_bin[1].block(pixel_row_start,
                                            pixel_col_start,
                                            pixel_stride[0] * extent_divider_for_search,
                                            pixel_stride[1] * extent_divider_for_search),
                xyz_bin[2].block(pixel_row_start,
                                            pixel_col_start,
                                            pixel_stride[0] * extent_divider_for_search,
                                            pixel_stride[1] * extent_divider_for_search)
            };
            placement_options[0](start_rows.size(), start_cols.size()) = nanmean(block[0], 0, 0, block[0].rows(), block[0].cols());
            placement_options[1](start_rows.size(), start_cols.size()) = nanmean(block[1], 0, 0, block[1].rows(), block[1].cols());
            placement_options[2](start_rows.size(), start_cols.size()) = block[2].block(0, 0, block[2].rows(), 1).minCoeff<Eigen::NaNPropagationOptions::PropagateNumbers>();
            placement_options[3](start_rows.size(), start_cols.size()) = total_rows - (int)(object_extent_pixel[0] / 2);
            placement_options[4](start_rows.size(), start_cols.size()) = total_cols - (int)(object_extent_pixel[1] / 2);
        }
        //std::cout << "Available positions x: " << placement_options[0] << std::endl;
        //std::cout << "Available positions y: " << placement_options[1] << std::endl;
        //std::cout << "Available positions z: " << placement_options[2] << std::endl;

        return placement_options;
    }

    struct PlacementData
    {
        const int pixel_row;
        const int pixel_col;
        const Eigen::Vector3d xyz;
    };

    PlacementData choose_placement_point(const Matrix5Xd& placement_options)
    {
        double max_min_z = placement_options[2].maxCoeff<Eigen::NaNPropagationOptions::PropagateNumbers>();
        Matrix3Xd placement;
        Eigen::Vector3d placement_point;
        int placement_row = 0;
        int placement_col = 0;
        for (int row = 0; row < placement_options[0].rows(); ++row)
        {
            for (int col = 0; col < placement_options[0].cols(); ++col)
            {
                if (placement_options[2](row, col) == max_min_z)
                {
                    for (int i = 0; i < 3; i++)
                    {
                        placement_point(i) = placement_options[i](row, col);
                        placement_row = row;
                        placement_col = col;
                    }
                    break;
                }
            }
        }
        return PlacementData{ placement_row, placement_col, placement_point };
    }

    void annotatePlacePoint(cv::Mat& annotated_image, const std::vector<double>& placement_data, const ObjectInfo& object_info)
    {
        const cv::Point2i placement_point((int)placement_data[4], (int)placement_data[3]);
        std::ostringstream oss;
        oss << "z:";
        oss << std::fixed << std::setprecision(1) << placement_data[2];
        oss << ", (" << placement_data[3] << ", " << placement_data[4] << ")";
        std::string text = oss.str();
        std::cout << "Placement: " << placement_point << ", x: " << placement_data[0] << " y: " << placement_data[1] << " z: " << placement_data[2] << std::endl;
        const cv::Scalar color(0, 255, 255);
        //cv::putText(
        //    annotated_image,
        //    text,
        //    placement_point,
        //    cv::FONT_HERSHEY_PLAIN,
        //    1,
        //    color
        //);
        const cv::Point top_left(
            placement_point.x - (object_info.object_extent_pixel[1] / object_info.extent_divider_for_search),
            placement_point.y - (object_info.object_extent_pixel[0] / object_info.extent_divider_for_search)
        );
        const cv::Point bottom_right(
            placement_point.x + (object_info.object_extent_pixel[1] / object_info.extent_divider_for_search),
            placement_point.y + (object_info.object_extent_pixel[0] / object_info.extent_divider_for_search)
        );
        cv::rectangle(annotated_image, top_left, bottom_right, color, 5);
        cv::circle(annotated_image, placement_point, 5, color, cv::FILLED);
    }

    cv::Mat annotate_placements(const cv::Mat& bgr, const Matrix5Xd& placement_options, const ObjectInfo& object_info)
    {
        cv::Mat annotated_image = bgr.clone();
        for (int row = 0; row < placement_options[0].rows(); ++row)
        {
            for (int col = 0; col < placement_options[0].cols(); ++col)
            {
                annotatePlacePoint(
                    annotated_image,
                    std::vector<double>{
                    placement_options[0](row, col),
                        placement_options[1](row, col),
                        placement_options[2](row, col),
                        placement_options[3](row, col),
                        placement_options[4](row, col),
                },
                    object_info
                        );
            }
        }
        return annotated_image;
    }

    void print_time(const std::string& text, const Duration& duration)
    {
        std::cout << std::left << std::setfill(' ') << std::setw(50) << text << " - Execution time: " << formatDuration(duration) << std::endl;
    }

    template<typename Function, typename... Args>
    auto timeFunction(const std::string& text, Function&& func, Args&&... args) {
        auto startTime = std::chrono::high_resolution_clock::now();

        // Call the function with the provided arguments
        auto result = std::invoke(std::forward<Function>(func), std::forward<Args>(args)...);

        auto endTime = std::chrono::high_resolution_clock::now();

        print_time(text, (endTime - startTime));

        return result;
    }

    template<typename Function, typename... Args>
    void timeVoidFunction(const std::string& text, Function&& func, Args&&... args) {
        auto startTime = std::chrono::high_resolution_clock::now();

        // Call the function with the provided arguments
        std::invoke(std::forward<Function>(func), std::forward<Args>(args)...);

        auto endTime = std::chrono::high_resolution_clock::now();

        print_time(text, (endTime - startTime));
    }

    bool update_progress(double progress)
    {
        std::cout << "\rLoading point cloud: " << progress;
        for (int i = 0; i < 10; i++)
        {
            std::cout << " ";
        }
        std::cout.flush();
        if (progress == 100.0)
        {
            std::cout << std::endl;
        }
        return true;
    }

    uint8_t convertColorToUint8(const double color)
    {
        return (uint8_t)(255 * color);
    }

    std::tuple< Matrix3Xd, cv::Mat > placeDataInMatrices(open3d::geometry::PointCloud& pointCloud, const int height, const int width)
    {
        Matrix3Xd xyz{ MatrixXd(height, width), MatrixXd(height, width), MatrixXd(height, width) };
        cv::Mat full_bgr(height, width, CV_8UC3, cv::Scalar(100, 100, 0));

        for (int row = 0; row < height; ++row)
        {
            for (int col = 0; col < width; ++col)
            {
                const auto i = col + row * width;
                auto point = pointCloud.points_[i];
                xyz[0](row, col) = point[0];
                xyz[1](row, col) = point[1];
                xyz[2](row, col) = point[2];
                auto color = pointCloud.colors_[i];
                full_bgr.at<cv::Vec3b>(row, col) = cv::Vec3b(
                    convertColorToUint8(color[2]),
                    convertColorToUint8(color[1]),
                    convertColorToUint8(color[0])
                );
            }
        }
        return std::make_tuple(xyz, full_bgr);
    }
} // namespace

int main()
{
    // INPUT
    std::string pcd_path = "C:/Zivid/Temp/Fizyr/checkerboard_in_bin.pcd";
    std::cout << "Loading point cloud from " << pcd_path << std::endl;
    // Load the point cloud using Open3D
    //auto pointCloud = open3d::io::CreatePointCloudFromFile(pcd_path);
    auto pointCloud = open3d::geometry::PointCloud();
    const open3d::io::ReadPointCloudOption pclOption(update_progress);
    open3d::io::ReadPointCloudFromPCD(pcd_path, pointCloud, pclOption);
    const int width = 1224;
    const int height = 1024;
    if (!pointCloud.HasColors())
    {
        std::cout << "Has no colors" << std::endl;
    }
    if (!pointCloud.HasPoints())
    {
        std::cout << "Has no points" << std::endl;
    }
    if (pointCloud.HasPoints() && pointCloud.HasColors())
    {
        std::cout << "PointCloud size: " << pointCloud.points_.size() << std::endl;
    }
    if (pointCloud.points_.size() != (width * height))
    {
        std::cout << "Expected " << (width * height) << " points, got " << pointCloud.points_.size() << std::endl;
    }

    const std::map<std::string, int> bin{ { "row_min", 236 }, { "col_min", 229 }, { "row_max", 840 }, { "col_max", 1166 } };
    //const Eigen::Vector2d object_extent_mm(40, 40);
    //const int extent_divider_for_search = 4;
    const Eigen::Vector2d object_extent_mm(200, 300);
    const int extent_divider_for_search = 2;

    // PREPARE DATA
    Matrix3Xd xyz;
    cv::Mat full_bgr;
    std::tie(xyz, full_bgr) = placeDataInMatrices(pointCloud, height, width);

    // ALGORITHM
    const auto xyz_cropped = timeFunction("* TIME TO crop xyz", crop_xyz_2d, bin, xyz);
    const auto cropped_bgr = timeFunction("* TIME TO crop bgr", crop_bgr_2d, bin, full_bgr);

    std::cout << "object_extent_mm = " << object_extent_mm << std::endl;

    const auto bin_info = timeFunction("* TIME TO getBinInfo", getBinInfo, xyz_cropped);
    const auto object_info = timeFunction("* TIME TO getObjectInfo", getObjectInfo, bin_info, object_extent_mm, extent_divider_for_search);

    const Matrix5Xd placement_options = timeFunction("* TIME TO find_placement_options", find_placement_options, xyz_cropped, object_info);
    const PlacementData placement_point = timeFunction("choose_placement_point", choose_placement_point, placement_options);
    std::cout << "Selected place point, translation: " << placement_point.xyz << std::endl;

    // VISUALIZATION
    //cv::imshow("BGR image full", full_bgr);
    //cv::imshow("BGR image cropped", cropped_bgr);
    const auto image_with_all_placements = timeFunction("* TIME TO annotate all placements", annotate_placements, cropped_bgr, placement_options, object_info);

    const auto startTimeImageShowAll = std::chrono::high_resolution_clock::now();
    cv::imshow("Annotated - all points", image_with_all_placements);
    const auto endTimeImageShowAll = std::chrono::high_resolution_clock::now();
    print_time("* TIME TO show image with all annotations", (endTimeImageShowAll - startTimeImageShowAll));

    const auto startTimeCloneImage = std::chrono::high_resolution_clock::now();
    auto selected_point_bgr = cropped_bgr.clone();
    const auto endTimeClone = std::chrono::high_resolution_clock::now();
    print_time("* TIME TO clone image", (endTimeClone - startTimeCloneImage));

    timeVoidFunction("* TIME TO annotate selected point", annotatePlacePoint,
        selected_point_bgr,
        std::vector<double>{
        placement_options[0](placement_point.pixel_row, placement_point.pixel_col),
            placement_options[1](placement_point.pixel_row, placement_point.pixel_col),
            placement_options[2](placement_point.pixel_row, placement_point.pixel_col),
            placement_options[3](placement_point.pixel_row, placement_point.pixel_col),
            placement_options[4](placement_point.pixel_row, placement_point.pixel_col),
    },
        object_info
            );

    const auto startTimeImageShowSelected = std::chrono::high_resolution_clock::now();
    cv::imshow("Annotated - selected point", selected_point_bgr);
    const auto endTimeImageShowSelected = std::chrono::high_resolution_clock::now();
    print_time("* TIME TO show image with selected annotations", (endTimeImageShowSelected - startTimeImageShowSelected));

    cv::waitKey(0);

    return 0;
}
