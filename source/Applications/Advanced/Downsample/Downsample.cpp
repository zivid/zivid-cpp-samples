/*
This example shows how to downsample point cloud from ZDF file.
*/

#include <Zivid/Visualization/Visualizer.h>
#include <Zivid/Zivid.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>

namespace
{
    float nanToZero(float x)
    {
        if(std::isnan(x))
        {
            return 0;
        }
        return x;
    }

    Eigen::MatrixXf lineSum(Eigen::MatrixXf matrixf, int downsamplingFactor)
    {
        Eigen::Map<Eigen::MatrixXf> flattenedMatrixMap(matrixf.data(),
                                                       downsamplingFactor,
                                                       matrixf.rows() * matrixf.cols() / downsamplingFactor);
        std::function<float(float)> nanToZeroFunctor = nanToZero;

        Eigen::MatrixXf flattenedMatrixNansRemoved = flattenedMatrixMap.unaryExpr(nanToZeroFunctor);

        Eigen::MatrixXf flattenedMatrixColwiseSum = flattenedMatrixNansRemoved.colwise().sum();

        Eigen::Map<Eigen::MatrixXf> reshapedMatrixMap(flattenedMatrixColwiseSum.data(),
                                                      matrixf.rows() / downsamplingFactor,
                                                      matrixf.cols());

        return reshapedMatrixMap;
    }

    Eigen::MatrixXf gridSum(const Eigen::MatrixXf &matrixf, int downsamplingFactor)
    {
        return lineSum(lineSum(matrixf, downsamplingFactor).transpose(), downsamplingFactor).transpose();
    }

    Eigen::MatrixXi downsampleAndRound(const Eigen::MatrixXi &matrixi, int downsamplingFactor)
    {
        Eigen::MatrixXf matrixf = matrixi.template cast<float>();

        std::function<float(float)> roundFunctor = [](float f) { return std::round(f); };

        return ((gridSum(matrixf, downsamplingFactor) / (downsamplingFactor * downsamplingFactor))
                    .unaryExpr(roundFunctor))
            .template cast<int>();
    }

    pcl::PointCloud<pcl::PointXYZRGB> downsample(const Zivid::PointCloud &pointCloud, int downsamplingFactor)
    {
        /*
        Function for downsampling a Zivid point cloud. The downsampling factor represents the denominator
        of a fraction that represents the size of the downsampled point cloud relative to the original
        point cloud, e.g. 2 - one-half,  3 - one-third, 4 one-quarter, etc.
        */

        if((pointCloud.height() % downsamplingFactor) != 0U || (pointCloud.width() % downsamplingFactor) != 0U)
        {
            throw std::invalid_argument("Downsampling factor (" + std::to_string(downsamplingFactor)
                                        + ") has to be a factor of the width (" + std::to_string(pointCloud.width())
                                        + ") and height (" + std::to_string(pointCloud.height())
                                        + ") of the input point cloud.");
        }

        std::cout << "Creating point cloud structure" << std::endl;
        Eigen::MatrixXf x(pointCloud.height(), pointCloud.width());
        Eigen::MatrixXf y(pointCloud.height(), pointCloud.width());
        Eigen::MatrixXf z(pointCloud.height(), pointCloud.width());
        Eigen::MatrixXi r(pointCloud.height(), pointCloud.width());
        Eigen::MatrixXi g(pointCloud.height(), pointCloud.width());
        Eigen::MatrixXi b(pointCloud.height(), pointCloud.width());
        Eigen::MatrixXf snr(pointCloud.height(), pointCloud.width());

        auto xyz = pointCloud.copyPointsXYZ();
        auto rgba = pointCloud.copyColorsRGBA();
        for(size_t i = 0; i < pointCloud.height(); i++)
        {
            for(size_t j = 0; j < pointCloud.width(); j++)
            {
                x(i, j) = xyz(i, j).x;
                y(i, j) = xyz(i, j).y;
                z(i, j) = xyz(i, j).z;
                r(i, j) = rgba(i, j).r;
                g(i, j) = rgba(i, j).g;
                b(i, j) = rgba(i, j).b;
            }
        }

        // Copying SNR values directly from GPU to Eigen matrix
        pointCloud.copyData(
            reinterpret_cast<Zivid::SNR *>(snr.data())); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)

        Eigen::MatrixXi redDownsampled = downsampleAndRound(r, downsamplingFactor);
        Eigen::MatrixXi greenDownsampled = downsampleAndRound(g, downsamplingFactor);
        Eigen::MatrixXi blueDownsampled = downsampleAndRound(b, downsamplingFactor);

        std::function<float(float)> isNotNanFunctor = [](float f) { return !std::isnan(f); };
        Eigen::MatrixXf snrNulled = (z.unaryExpr(isNotNanFunctor)).cwiseProduct(snr);

        auto snrWeight = gridSum(snrNulled, downsamplingFactor);

        Eigen::MatrixXf xContrasted = x.cwiseProduct(snrNulled);
        Eigen::MatrixXf yContrasted = y.cwiseProduct(snrNulled);
        Eigen::MatrixXf zContrasted = z.cwiseProduct(snrNulled);
        Eigen::MatrixXf snrContrasted = snr.cwiseProduct(snrNulled);

        Eigen::MatrixXf xDownsampled = gridSum(xContrasted, downsamplingFactor).cwiseQuotient(snrWeight);
        Eigen::MatrixXf yDownsampled = gridSum(yContrasted, downsamplingFactor).cwiseQuotient(snrWeight);
        Eigen::MatrixXf zDownsampled = gridSum(zContrasted, downsamplingFactor).cwiseQuotient(snrWeight);
        Eigen::MatrixXf contrastDownsampled = gridSum(snrContrasted, downsamplingFactor).cwiseQuotient(snrWeight);

        pcl::PointCloud<pcl::PointXYZRGB> pointCloudDownsampled(xDownsampled.rows(), xDownsampled.cols());
        pointCloudDownsampled.is_dense = false;
        pointCloudDownsampled.points.resize(xDownsampled.rows() * xDownsampled.cols());

        for(int i = 0; i < xDownsampled.rows(); i++)
        {
            for(int j = 0; j < xDownsampled.cols(); j++)
            {
                pointCloudDownsampled(i, j).r = redDownsampled(i, j);
                pointCloudDownsampled(i, j).g = greenDownsampled(i, j);
                pointCloudDownsampled(i, j).b = blueDownsampled(i, j);
                pointCloudDownsampled(i, j).x = xDownsampled(i, j);
                pointCloudDownsampled(i, j).y = yDownsampled(i, j);
                pointCloudDownsampled(i, j).z = zDownsampled(i, j);
            }
        }

        return pointCloudDownsampled;
    }

    void visualizePCLPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudPtr)
    {
        std::cout << "Running PCL visualizer. Blocking until window closes" << std::endl;
        pcl::visualization::CloudViewer viewer("PCL Cloud Viewer");
        viewer.showCloud(cloudPtr);
        std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
        std::cout << "Press q to exit the viewer application" << std::endl;
        while(!viewer.wasStopped())
        {
        }
    }

    void visualizePointCloud(const Zivid::PointCloud &pointCloud)
    {
        std::cout << "Setting up visualization" << std::endl;
        Zivid::Visualization::Visualizer visualizer;
        std::cout << "Visualizing point cloud" << std::endl;
        visualizer.showMaximized();
        visualizer.show(pointCloud);
        visualizer.resetToFit();

        std::cout << "Running visualizer. Blocking until window closes" << std::endl;
        visualizer.run();
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

        const auto pointCloud = frame.pointCloud();
        std::cout << "Loaded " << pointCloud.size() << " data points" << std::endl;

        auto downsamplingFactor = 4;
        auto pointCloudDownsampled = downsample(pointCloud, downsamplingFactor);

        std::cout << "Visualizing point cloud before downsampling" << std::endl;
        visualizePointCloud(pointCloud);

        std::cout << "Visualizing point cloud after downsampling" << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
        *cloudPtr = pointCloudDownsampled;
        visualizePCLPointCloud(cloudPtr);
    }

    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        if(std::cin.get() == '\n')
        {
            return EXIT_FAILURE;
        }
    }
}
