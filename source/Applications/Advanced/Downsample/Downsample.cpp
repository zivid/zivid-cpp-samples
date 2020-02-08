/*
Import a ZDF point cloud and downsample it.
*/

#include <Zivid/CloudVisualizer.h>
#include <Zivid/Zivid.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>

Zivid::PointCloud downsample(const Zivid::PointCloud &, int);
Eigen::MatrixXi downsampleAndRound(const Eigen::MatrixXi &, int);
Eigen::MatrixXf gridSum(const Eigen::MatrixXf &, int);
Eigen::MatrixXf lineSum(Eigen::MatrixXf, int);
float nanToZero(float);
void visualizePointCloud(const Zivid::PointCloud &, Zivid::Application &);

int main()
{
    try
    {
        Zivid::Application zivid;

        std::string filename = "Zivid3D.zdf";
        std::cout << "Reading " << filename << " point cloud" << std::endl;
        Zivid::Frame frame(filename);

        auto pointCloud = frame.getPointCloud();

        auto downsamplingFactor = 4;

        auto pointCloudDownsampled = downsample(pointCloud, downsamplingFactor);

        visualizePointCloud(pointCloud, zivid);
        visualizePointCloud(pointCloudDownsampled, zivid);
    }

    catch(const std::exception &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        return EXIT_FAILURE;
    }
}

Zivid::PointCloud downsample(const Zivid::PointCloud &pointCloud, int downsamplingFactor)
{
    /*
	Function for downsampling a Zivid point cloud. The downsampling factor represents the denominator
	of a fraction that represents the size of the downsampled point cloud relative to the original
	point cloud, e.g. 2 - one-half,  3 - one-third, 4 one-quarter, etc.
	*/

    if((pointCloud.height() % downsamplingFactor) || (pointCloud.width() % downsamplingFactor))
    {
        throw std::invalid_argument("Downsampling factor (" + std::to_string(downsamplingFactor)
                                    + ") has to a factor of the width (" + std::to_string(pointCloud.width())
                                    + ") and height (" + std::to_string(pointCloud.height())
                                    + ") of the input point cloud.");
    }

    Eigen::MatrixXf x(pointCloud.height(), pointCloud.width());
    Eigen::MatrixXf y(pointCloud.height(), pointCloud.width());
    Eigen::MatrixXf z(pointCloud.height(), pointCloud.width());
    Eigen::MatrixXi r(pointCloud.height(), pointCloud.width());
    Eigen::MatrixXi g(pointCloud.height(), pointCloud.width());
    Eigen::MatrixXi b(pointCloud.height(), pointCloud.width());
    Eigen::MatrixXf contrast(pointCloud.height(), pointCloud.width());

    for(size_t i = 0; i < pointCloud.height(); i++)
    {
        for(size_t j = 0; j < pointCloud.width(); j++)
        {
            x(i, j) = pointCloud(i, j).x;
            y(i, j) = pointCloud(i, j).y;
            z(i, j) = pointCloud(i, j).z;
            r(i, j) = pointCloud(i, j).red();
            g(i, j) = pointCloud(i, j).green();
            b(i, j) = pointCloud(i, j).blue();
            contrast(i, j) = pointCloud(i, j).contrast;
        }
    }

    Eigen::MatrixXi redDownsampled = downsampleAndRound(r, downsamplingFactor);
    Eigen::MatrixXi greenDownsampled = downsampleAndRound(g, downsamplingFactor);
    Eigen::MatrixXi blueDownsampled = downsampleAndRound(b, downsamplingFactor);

    std::function<float(float)> is_not_nan_functor = [](float f) { return !std::isnan(f); };
    std::function<float(float)> nan_to_zero_functor = nanToZero;

    Eigen::MatrixXf contrastNulled =
        (z.unaryExpr(is_not_nan_functor)).cwiseProduct(contrast.unaryExpr(nan_to_zero_functor));

    auto contrastWeight = gridSum(contrastNulled, downsamplingFactor);

    Eigen::MatrixXf xContrasted = x.cwiseProduct(contrastNulled);
    Eigen::MatrixXf yContrasted = y.cwiseProduct(contrastNulled);
    Eigen::MatrixXf zContrasted = z.cwiseProduct(contrastNulled);
    Eigen::MatrixXf contrastContrasted = contrast.cwiseProduct(contrastNulled);

    Eigen::MatrixXf xDownsampled = gridSum(xContrasted, downsamplingFactor).cwiseQuotient(contrastWeight);
    Eigen::MatrixXf yDownsampled = gridSum(yContrasted, downsamplingFactor).cwiseQuotient(contrastWeight);
    Eigen::MatrixXf zDownsampled = gridSum(zContrasted, downsamplingFactor).cwiseQuotient(contrastWeight);
    Eigen::MatrixXf contrastDownsampled = gridSum(contrastContrasted, downsamplingFactor).cwiseQuotient(contrastWeight);

    Zivid::PointCloud pointCloudDownsampled(redDownsampled.rows(), redDownsampled.cols());

    for(int i = 0; i < redDownsampled.rows(); i++)
    {
        for(int j = 0; j < redDownsampled.cols(); j++)
        {
            pointCloudDownsampled(i, j).setRgb(redDownsampled(i, j), greenDownsampled(i, j), blueDownsampled(i, j));
            pointCloudDownsampled(i, j).setContrast(contrastDownsampled(i, j));
            pointCloudDownsampled(i, j).x = xDownsampled(i, j);
            pointCloudDownsampled(i, j).y = yDownsampled(i, j);
            pointCloudDownsampled(i, j).z = zDownsampled(i, j);
        }
    }

    return pointCloudDownsampled;
}

Eigen::MatrixXi downsampleAndRound(const Eigen::MatrixXi &matrixi, int downsamplingFactor)
{
    Eigen::MatrixXf matrixf = matrixi.template cast<float>();

    std::function<float(float)> round_functor = [](float f) { return std::round(f); };

    return ((gridSum(matrixf, downsamplingFactor) / (downsamplingFactor * downsamplingFactor)).unaryExpr(round_functor))
        .template cast<int>();
}

Eigen::MatrixXf gridSum(const Eigen::MatrixXf &r, int downsamplingFactor)
{
    return lineSum(lineSum(r, downsamplingFactor).transpose(), downsamplingFactor).transpose();
}

Eigen::MatrixXf lineSum(Eigen::MatrixXf matrix, int downsamplingFactor)
{
    Eigen::Map<Eigen::MatrixXf> flattenedMatrixMap(matrix.data(),
                                                   downsamplingFactor,
                                                   matrix.rows() * matrix.cols() / downsamplingFactor);
    std::function<float(float)> nan_to_zero_functor = nanToZero;

    Eigen::MatrixXf flattenedMatrixNansRemoved = flattenedMatrixMap.unaryExpr(nan_to_zero_functor);

    Eigen::MatrixXf flattenedMatrixColwiseSum = flattenedMatrixNansRemoved.colwise().sum();

    Eigen::Map<Eigen::MatrixXf> reshapedMatrixMap(flattenedMatrixColwiseSum.data(),
                                                  matrix.rows() / downsamplingFactor,
                                                  matrix.cols());

    return reshapedMatrixMap;
}

float nanToZero(float x)
{
    if(std::isnan(x))
    {
        return 0;
    }
    else
    {
        return x;
    }
}

void visualizePointCloud(const Zivid::PointCloud &pointCloud, Zivid::Application &zivid)
{
    std::cout << "Setting up visualization" << std::endl;
    Zivid::CloudVisualizer vis;
    zivid.setDefaultComputeDevice(vis.computeDevice());

    std::cout << "Displaying the point cloud" << std::endl;
    vis.showMaximized();
    vis.show(pointCloud);
    vis.resetToFit();

    std::cout << "Running the visualizer. Blocking until the window closes" << std::endl;
    vis.run();
}
