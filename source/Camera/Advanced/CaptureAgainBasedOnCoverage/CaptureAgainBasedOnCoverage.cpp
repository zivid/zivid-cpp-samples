/*
Capture point clouds, with color, with the Zivid file camera.
This sample can be used without access to a physical camera.

The file camera files are found in Zivid Sample Data with ZFC file extension.
See the instructions in README.md to download the Zivid Sample Data.
There are five available file cameras to choose from, one for each camera model.
The default file camera used in this sample is the Zivid 2 M70 file camera.
*/

#include <opencv2/opencv.hpp>
#include <Zivid/Zivid.h>

#include <clipp.h>
#include <vector>
#include <execution>
#include <iostream>
#include <atomic>

namespace {

    using SteadyClock = std::chrono::steady_clock;

    cv::Mat pointCloudToCvMat(const Zivid::PointCloud& pointCloud) {
        auto pointsZ = cv::Mat(cv::Size(pointCloud.height(), pointCloud.width()), CV_32FC1);
        pointCloud.copyData(reinterpret_cast<Zivid::PointZ*>(pointsZ.data));
        return pointsZ;
    }

    cv::Mat pointCloudZToCvNaNMask(const Zivid::Array2D<Zivid::PointZ> pointsZ) {
        auto nanMask = cv::Mat(pointsZ.height(), pointsZ.width(), CV_32FC1);

        for (size_t i = 0; i < pointsZ.height(); i++)
        {
            for (size_t j = 0; j < pointsZ.width(); j++)
            {
                nanMask.at<float>(i, j) = std::isnan(pointsZ(i, j).z) ? std::numeric_limits<float>::quiet_NaN() : pointsZ(i, j).z;
            }
        }

        return nanMask;
    }

    cv::Mat pointsZToCvNaNMask(Zivid::Array2D<Zivid::PointZ> pointsZ) {
        auto nanMask = cv::Mat(pointsZ.height(), pointsZ.width(), CV_8UC1, cv::Scalar(0));

        for (size_t i = 0; i < pointsZ.height(); i++) {
            for (size_t j = 0; j < pointsZ.width(); j++) {
                if (std::isnan(pointsZ(i, j).z)) {
                    nanMask.at<uchar>(i, j) = 255;
                }
            }
        }
        return nanMask;
    }

    int countNaNs(const cv::Mat& image) {
        auto patchedImage = cv::Mat(image);
        cv::patchNaNs(patchedImage, 0.0);
        return (patchedImage.size().height * patchedImage.size().width) - cv::countNonZero(patchedImage);
    }

    cv::Mat generateNaNMask(const cv::Mat& depthImage) {
        if (depthImage.empty() || depthImage.type() != CV_32F) {
            std::cerr << "Invalid depth image. Ensure it is a non-empty CV_32F matrix." << std::endl;
            return cv::Mat();
        }
        // Step 1: Create a NaN mask
        return (depthImage != depthImage); // NaNs are the only values where x != x
    }

    cv::Mat computeValidRegionMaskParallel(const Zivid::Array2D<Zivid::PointZ>& pointsZ) {
        using SteadyClock = std::chrono::steady_clock;
        const auto before = SteadyClock::now();

        int rows = pointsZ.height();
        int cols = pointsZ.width();
        std::vector<uint8_t> validRegion(rows * cols, 255);
        // cv::Mat validRegion(rows, cols, CV_8UC1, cv::Scalar(255));

        std::vector<int> firstValidInRow(rows, -1), lastValidInRow(rows, -1);
        std::vector<int> firstValidInCol(cols, -1), lastValidInCol(cols, -1);

		//std::cout << "**Parallel: Find left & right bounds per row**" << std::endl;
        std::vector<int> rowIndices(rows);
        std::iota(rowIndices.begin(), rowIndices.end(), 0);

        std::for_each(std::execution::par, rowIndices.begin(), rowIndices.end(),
            [&](int y) {
                for (int x = 0; x < cols; ++x) {
                    if (!std::isnan(pointsZ(y, x).z)) {
                        if (firstValidInRow[y] == -1) firstValidInRow[y] = x;
                        lastValidInRow[y] = x;
                    }
                }
            });

		//std::cout << "**Parallel: Find top & bottom bounds per column**" << std::endl;
        std::vector<int> colIndices(cols);
        std::iota(colIndices.begin(), colIndices.end(), 0);

        std::for_each(std::execution::par, colIndices.begin(), colIndices.end(),
            [&](int x) {
                for (int y = 0; y < rows; ++y) {
                    if (!std::isnan(pointsZ(y, x).z)) {
                        if (firstValidInCol[x] == -1) firstValidInCol[x] = y;
                        lastValidInCol[x] = y;
                    }
                }
            });

		//std::cout << "**Parallel: Identify edge NaNs**" << std::endl;
        std::vector<int> indices(rows * cols);
        std::iota(indices.begin(), indices.end(), 0);  // Create a linear index array

        std::for_each(std::execution::par_unseq, indices.begin(), indices.end(),
            [&](int index) {
                int y = index / cols;
                int x = index % cols;

                if (std::isnan(pointsZ(y, x).z)) {
                    bool isEdge = (x == 0 || x == cols - 1 || y == 0 || y == rows - 1) ||
                        (firstValidInRow[y] != -1 && (x < firstValidInRow[y] || x > lastValidInRow[y])) ||
                        (firstValidInCol[x] != -1 && (y < firstValidInCol[x] || y > lastValidInCol[x]));

                    if (isEdge) {
                        validRegion[y * cols + x] = 0;
                        // validRegion.at<uint8_t>(y, x) = 0; // Mark as edge NaN
                    }
                }
            });
        const auto after = SteadyClock::now();
        std::cout << "Finding edge mask took (parallel): "
            << std::chrono::duration_cast<std::chrono::milliseconds>(after - before).count()
            << " ms" << std::endl;

        return cv::Mat(rows, cols, CV_8UC1, validRegion.data()).clone();
    }

    cv::Mat computeValidRegionMask(const Zivid::Array2D<Zivid::PointZ> pointsZ) {
        const auto before = SteadyClock::now();
        int rows = pointsZ.height();
        int cols = pointsZ.width();
        auto validRegion = cv::Mat(pointsZ.height(), pointsZ.width(), CV_8UC1, cv::Scalar(255));
        //cv::Mat nanMask = cv::Mat::zeros(rows, cols, CV_8U); // Initialize mask (0 = not edge NaN)

        std::vector<int> firstValidInRow(rows, -1), lastValidInRow(rows, -1);
        std::vector<int> firstValidInCol(cols, -1), lastValidInCol(cols, -1);

        // Find left & right bounds per row
        for (int y = 0; y < rows; ++y) {
            for (int x = 0; x < cols; ++x) {
                if (!std::isnan(pointsZ(y, x).z)) {
                    if (firstValidInRow[y] == -1) firstValidInRow[y] = x;
                    lastValidInRow[y] = x;
                }
            }
        }

        // Find top & bottom bounds per column
        for (int x = 0; x < cols; ++x) {
            for (int y = 0; y < rows; ++y) {
                if (!std::isnan(pointsZ(y, x).z)) {
                    if (firstValidInCol[x] == -1) firstValidInCol[x] = y;
                    lastValidInCol[x] = y;
                }
            }
        }

        // Identify edge NaNs
        for (int y = 0; y < rows; ++y) {
            for (int x = 0; x < cols; ++x) {
                if (std::isnan(pointsZ(y, x).z)) {
                    bool isEdge = (x == 0 || x == cols - 1 || y == 0 || y == rows - 1) ||
                        (firstValidInRow[y] != -1 && (x < firstValidInRow[y] || x > lastValidInRow[y])) ||
                        (firstValidInCol[x] != -1 && (y < firstValidInCol[x] || y > lastValidInCol[x]));

                    if (isEdge) {
                        validRegion.at<uint8_t>(y, x) = 0; // Mark as edge NaN
                    }
                }
            }
        }
        const auto after = SteadyClock::now();
        std::cout << "Finding edge mask took: " << std::chrono::duration_cast<std::chrono::milliseconds>(after - before).count() << " ms" << std::endl;
        return validRegion;
    }

    cv::Mat validRegionMaskWithErosion(const cv::Mat& nanMask, int erosionSize) {
        // Step 2: Erode NaN regions inward
        cv::Mat erodedMask;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erosionSize, erosionSize));
        cv::erode(nanMask, erodedMask, kernel);
        // Step 3: Define the valid region (everything outside the eroded NaN mask)
        return (erodedMask == 0);
    }

    double coverageWithEdgeMask(Zivid::Array2D<Zivid::PointZ> pointsZ) {
        const auto before = SteadyClock::now();
        const auto nanMask = pointsZToCvNaNMask(pointsZ);
        const auto validRegionMask = computeValidRegionMask(pointsZ);
        const double validRegionPointCount = cv::countNonZero(validRegionMask);
        cv::Mat validNaNs;
        cv::bitwise_and(nanMask, validRegionMask, validNaNs);
        const int validRegionNanCount = cv::countNonZero(validNaNs);
        const auto after = SteadyClock::now();
        const double totalNumberOfNaNs = cv::countNonZero(nanMask);
        const double coverage = (validRegionPointCount - static_cast<double>(validRegionNanCount)) / validRegionPointCount;
        std::cout << "Total number of points: " << pointsZ.height() * pointsZ.width() << std::endl;
        std::cout << "Total number of NaNs: " << totalNumberOfNaNs << std::endl;
        std::cout << "Total number of points in valid region: " << validRegionPointCount << std::endl;
        std::cout << "Total number of NaNs in valid region: " << validRegionNanCount << std::endl;
        std::cout << "Coverage with edge mask: (" << (validRegionPointCount - validRegionNanCount) << " / " << validRegionPointCount << ") = " << std::fixed << std::setprecision(1) << coverage * 100 << "%" << std::endl;
        std::cout << "Coverage calculation took: " << std::chrono::duration_cast<std::chrono::milliseconds>(after - before).count() << " ms" << std::endl;
        cv::imwrite("validRegionEdgeMask.png", validRegionMask);

        return (validRegionPointCount - validRegionNanCount) / validRegionPointCount;
    }

    double coverageWithEdgeMaskParallel(Zivid::Array2D<Zivid::PointZ> pointsZ) {
        const auto before = SteadyClock::now();
        const auto nanMask = pointsZToCvNaNMask(pointsZ);
        const auto validRegionMask = computeValidRegionMaskParallel(pointsZ);
        const double validRegionPointCount = cv::countNonZero(validRegionMask);
        cv::Mat validNaNs;
        cv::bitwise_and(nanMask, validRegionMask, validNaNs);
        const int validRegionNanCount = cv::countNonZero(validNaNs);
        const auto after = SteadyClock::now();
        const double totalNumberOfNaNs = cv::countNonZero(nanMask);
        const double coverage = (validRegionPointCount - static_cast<double>(validRegionNanCount)) / validRegionPointCount;
        std::cout << "Total number of points: " << pointsZ.height() * pointsZ.width() << std::endl;
        std::cout << "Total number of NaNs: " << totalNumberOfNaNs << std::endl;
        std::cout << "Total number of points in valid region: " << validRegionPointCount << std::endl;
        std::cout << "Total number of NaNs in valid region: " << validRegionNanCount << std::endl;
        std::cout << "Coverage with edge mask: (" << (validRegionPointCount - validRegionNanCount) << " / " << validRegionPointCount << ") = " << std::fixed << std::setprecision(1) << coverage * 100 << "%" << std::endl;
        std::cout << "Coverage calculation took: " << std::chrono::duration_cast<std::chrono::milliseconds>(after - before).count() << " ms" << std::endl;
        cv::imwrite("validRegionEdgeMaskParallel.png", validRegionMask);

        return (validRegionPointCount - validRegionNanCount) / validRegionPointCount;
    }

    double coverageWithErosion(Zivid::Array2D<Zivid::PointZ> pointsZ, const int erosionSize) {
        const auto before = SteadyClock::now();
        const auto nanMask = pointsZToCvNaNMask(pointsZ);
        const auto validRegionMask = validRegionMaskWithErosion(nanMask, erosionSize);
        const double validRegionPointCount = cv::countNonZero(validRegionMask);
        cv::Mat validNaNs;
        cv::bitwise_and(nanMask, validRegionMask, validNaNs);
        const int validRegionNanCount = cv::countNonZero(validNaNs);
        const auto after = SteadyClock::now();
        const double totalNumberOfNaNs = cv::countNonZero(nanMask);
        const double coverage = (validRegionPointCount - static_cast<double>(validRegionNanCount)) / validRegionPointCount;
        std::cout << "Total number of points: " << pointsZ.height() * pointsZ.width() << std::endl;
        std::cout << "Total number of NaNs: " << totalNumberOfNaNs << std::endl;
        std::cout << "Total number of points in valid region: " << validRegionPointCount << std::endl;
        std::cout << "Total number of NaNs in valid region: " << validRegionNanCount << std::endl;
        std::cout << "Coverage with erosion: (" << (validRegionPointCount - validRegionNanCount) << " / " << validRegionPointCount << ") = " << std::fixed << std::setprecision(1) << coverage * 100 << "%" << std::endl;
        std::cout << "Coverage calculation took: " << std::chrono::duration_cast<std::chrono::milliseconds>(after - before).count() << " ms" << std::endl;
        cv::imwrite("validRegionErosion.png", validRegionMask);

        return (validRegionPointCount - validRegionNanCount) / validRegionPointCount;
    }

    double coverageWithCrop(Zivid::Array2D<Zivid::PointZ> pointsZ) {
        const double crop = 0.05;
        const auto before = SteadyClock::now();
        const int croppedWidth = pointsZ.width() * (1 - 2 * crop);
        const int croppedHeight = pointsZ.height() * (1 - 2 * crop);
        const int totalNumberOfPoints = croppedWidth * croppedHeight;
        int numberOfNanPoints = 0;
        for (size_t i = static_cast<size_t>(pointsZ.height() * crop); i < croppedHeight; i++)
        {
            for (size_t j = static_cast<size_t>(pointsZ.width() * crop); j < croppedWidth; j++)
            {
                if (pointsZ(i, j).isNaN())
                {
                    numberOfNanPoints++;
                }
            }
        }
        const auto after = SteadyClock::now();
        const double coverage = (static_cast<double>(totalNumberOfPoints) - numberOfNanPoints) / totalNumberOfPoints;
        std::cout << "Coverage with crop: (" << (totalNumberOfPoints - numberOfNanPoints) << " / " << totalNumberOfPoints << ") = " << std::fixed << std::setprecision(1) << coverage * 100 << "%" << std::endl;
        std::cout << "Coverage calculation took: " << std::chrono::duration_cast<std::chrono::milliseconds>(after - before).count() << " ms" << std::endl;
        return (totalNumberOfPoints - numberOfNanPoints) / totalNumberOfPoints;
    }

    double coverageWithEdgeMaskOptimized(Zivid::Array2D<Zivid::PointZ> pointsZ) {
        using SteadyClock = std::chrono::steady_clock;
        const auto before = SteadyClock::now();

        int rows = pointsZ.height();
        int cols = pointsZ.width();
        // cv::Mat validRegion(rows, cols, CV_8UC1, cv::Scalar(255));

        std::vector<int> firstValidInRow(rows, -1), lastValidInRow(rows, -1);
        std::vector<int> firstValidInCol(cols, -1), lastValidInCol(cols, -1);

        //std::cout << "**Parallel: Find left & right bounds per row**" << std::endl;
        std::vector<int> rowIndices(rows);
        std::iota(rowIndices.begin(), rowIndices.end(), 0);

        std::for_each(std::execution::par, rowIndices.begin(), rowIndices.end(),
            [&](int y) {
                for (int x = 0; x < cols; ++x) {
                    if (!std::isnan(pointsZ(y, x).z)) {
                        if (firstValidInRow[y] == -1) firstValidInRow[y] = x;
                        lastValidInRow[y] = x;
                    }
                }
            });

        //std::cout << "**Parallel: Find top & bottom bounds per column**" << std::endl;
        std::vector<int> colIndices(cols);
        std::iota(colIndices.begin(), colIndices.end(), 0);

        std::for_each(std::execution::par, colIndices.begin(), colIndices.end(),
            [&](int x) {
                for (int y = 0; y < rows; ++y) {
                    if (!std::isnan(pointsZ(y, x).z)) {
                        if (firstValidInCol[x] == -1) firstValidInCol[x] = y;
                        lastValidInCol[x] = y;
                    }
                }
            });

        //std::cout << "**Parallel: Identify edge NaNs**" << std::endl;
        std::vector<uint8_t> nonEdgeNaNs(rows * cols, 0);
        std::vector<uint8_t> nonEdgeValues(rows * cols, 0);
        std::vector<int> indices(rows * cols);
        std::iota(indices.begin(), indices.end(), 0);  // Create a linear index array

        std::for_each(std::execution::par_unseq, indices.begin(), indices.end(),
            [&](int index) {
                int y = index / cols;
                int x = index % cols;

                bool isEdge = (x == 0 || x == cols - 1 || y == 0 || y == rows - 1) ||
                    (firstValidInRow[y] != -1 && (x < firstValidInRow[y] || x > lastValidInRow[y])) ||
                    (firstValidInCol[x] != -1 && (y < firstValidInCol[x] || y > lastValidInCol[x]));

                if (!isEdge) {
					nonEdgeValues[y * cols + x] = 255;
                    if (std::isnan(pointsZ(y, x).z)) {
                        nonEdgeNaNs[y * cols + x] = 255;
                        // validRegion.at<uint8_t>(y, x) = 0; // Mark as edge NaN
                    }
                }
            });
        int nonEdgeNaNCount = 0;
        int nonEdgeCount = 0;
		for (int y = 0; y < rows; ++y) {
			for (int x = 0; x < cols; ++x) {
				if (nonEdgeValues[y * cols + x] == 255) {
					nonEdgeCount++;
				}
				if (nonEdgeNaNs[y * cols + x] == 255) {
					nonEdgeNaNCount++;
				}
			}
		}
        const auto after = SteadyClock::now();

        std::cout << "Total number of points in valid region: " << nonEdgeCount << std::endl;
        std::cout << "Total number of NaNs in valid region: " << nonEdgeNaNCount << std::endl;
		const double coverage = static_cast<double>(nonEdgeCount - nonEdgeNaNCount) / nonEdgeCount;
        std::cout << "Coverage with edge mask optimized: (" << (nonEdgeCount - nonEdgeNaNCount) << " / " << nonEdgeCount << ") = " << std::fixed << std::setprecision(1) << coverage * 100 << "%" << std::endl;
        std::cout << "Coverage calculation took: " << std::chrono::duration_cast<std::chrono::milliseconds>(after - before).count() << " ms" << std::endl;

        return coverage;
    }
}

int main(int argc, char** argv)
{
    try
    {
        bool userInput = false;

        std::string fileCameraPath;
        auto cli =
            (clipp::option("--file-camera").set(userInput, true)
                & clipp::value("<Path to the file camera .zfc file>", fileCameraPath));

        if (!parse(argc, argv, cli))
        {
            auto fmt = clipp::doc_formatting{}.alternatives_min_split_size(1).surround_labels("\"", "\"");
            std::cout << clipp::usage_lines(cli, "Usage: ", fmt) << std::endl;
            throw std::runtime_error{ "Invalid usage" };
        }

        Zivid::Application zivid;

        const auto fileCamera =
            userInput ? fileCameraPath : std::string(ZIVID_SAMPLE_DATA_DIR) + "/FileCameraZivid2PlusMR60.zfc";

        std::cout << "Creating virtual camera using file: " << fileCamera << std::endl;
        auto camera = zivid.createFileCamera(fileCamera);

        std::cout << "Configuring settings" << std::endl;
        Zivid::Settings settings{
            Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} },
            Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
            Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{ 1.5 },
            Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes,
            Zivid::Settings::Processing::Filters::Reflection::Removal::Mode::global,
        };
        Zivid::Settings2D settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} },
                                      Zivid::Settings2D::Processing::Color::Balance::Red{ 1 },
                                      Zivid::Settings2D::Processing::Color::Balance::Green{ 1 },
                                      Zivid::Settings2D::Processing::Color::Balance::Blue{ 1 } };

        settings.color() = Zivid::Settings::Color{ settings2D };

        std::cout << "Capturing frame" << std::endl;
        auto frame = camera.capture2D3D(settings);

        const auto pointCloud = frame.pointCloud();
        const auto pointsZ = pointCloud.copyPointsZ();

        const auto pointsZCV = pointCloudToCvMat(pointCloud);
        std::cout << "Nan count in pointsZCV: " << countNaNs(pointsZCV) << std::endl;

        const auto pointsZwithNaN = pointCloudZToCvNaNMask(pointsZ);
        std::cout << "Nan count in pointsZCVwithNaN: " << countNaNs(pointsZwithNaN) << std::endl;

        const double minimumCoverage = 0.9;
        const int erosionSize = 5;
        std::cout << std::endl;
        std::cout << "Coverage calculation with crop:" << std::endl;
        std::cout << "************************************" << std::endl;
        coverageWithCrop(pointsZ);
        std::cout << std::endl;
		std::cout << "Coverage calculation with erosion:" << std::endl;
		std::cout << "************************************" << std::endl;
        coverageWithErosion(pointsZ, erosionSize);
        std::cout << std::endl;
        std::cout << "Coverage calculation with edge mask:" << std::endl;
        std::cout << "************************************" << std::endl;
        coverageWithEdgeMask(pointsZ);
        std::cout << std::endl;
        std::cout << "Coverage calculation with edge mask (parallel):" << std::endl;
        std::cout << "************************************" << std::endl;
        coverageWithEdgeMaskParallel(pointsZ);
		std::cout << std::endl;
		std::cout << "Coverage calculation with edge mask (optimized):" << std::endl;
		std::cout << "************************************" << std::endl;
		coverageWithEdgeMaskOptimized(pointsZ);
        /*< minimumCoverage) {
            settings.acquisitions().emplaceBack(Zivid::Settings::Acquisition{});
            settings.acquisitions().emplaceBack(Zivid::Settings::Acquisition{});
            frame = camera.capture2D3D(settings);
        }*/

        //const auto dataFile = "Frame.zdf";
        //std::cout << "Saving frame to file: " << dataFile << std::endl;
        //frame.save(dataFile);

        // Generate and save the nanMask as a grayscale image
        const auto nanMask = pointsZToCvNaNMask(pointsZ);
        const std::string nanMaskFile = "nanMask.png";
        std::cout << "Saving nanMask to file: " << nanMaskFile << std::endl;
        cv::imwrite(nanMaskFile, nanMask);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
