/*
Import a ZDF point cloud and downsample it.
*/

#include <Zivid/CloudVisualizer.h>
#include <Zivid/Zivid.h>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <iostream>
Zivid::PointCloud downsample(const Zivid::PointCloud&, int);
Eigen::MatrixXi downsample_and_round(const Eigen::MatrixXi&, int);
Eigen::MatrixXf grid_sum(const Eigen::MatrixXf&, int);
Eigen::MatrixXf line_sum(const Eigen::MatrixXf&, int);
float nan_to_zero(float);
void visualize_point_cloud(const Zivid::PointCloud&, Zivid::Application&);

int main()
{
	try
	{
		Zivid::Application zivid;
		
		std::string filename = "Zivid3D.zdf";
		std::cout << "Reading " << filename << " point cloud" << std::endl;
		Zivid::Frame frame(filename);

		auto pointCloud = frame.getPointCloud();

		auto downsamplingFactor = 1;

		auto pointCloudDownsampled = downsample(pointCloud, downsamplingFactor);

		visualize_point_cloud(pointCloud,zivid);
		visualize_point_cloud(pointCloudDownsampled,zivid);
	}

	catch (const std::exception &e)
	{
		std::cerr << "Error: " << Zivid::toString(e) << std::endl;
		return EXIT_FAILURE;
	}
}

Zivid::PointCloud downsample(const Zivid::PointCloud& pointCloud, int downsamplingFactor)
{
	/*
	Function for downsampling a Zivid point cloud. The downsampling factor represents the denominator
	of a fraction that represents the size of the downsampled point cloud relative to the original
	point cloud, e.g. 2 - one-half,  3 - one-third, 4 one-quarter, etc.
	*/

	if ((pointCloud.height() % downsamplingFactor) || (pointCloud.width() % downsamplingFactor))
	{
		throw std::invalid_argument(
			"Downsampling factor (" + std::to_string(downsamplingFactor) +
			") has to a factor of the width (" +
			std::to_string(pointCloud.width()) + ") and height (" +
			std::to_string(pointCloud.height()) + ") of the input point cloud.");
	}

	Eigen::MatrixXf x(pointCloud.height(), pointCloud.width());
	Eigen::MatrixXf y(pointCloud.height(), pointCloud.width());
	Eigen::MatrixXf z(pointCloud.height(), pointCloud.width());
	Eigen::MatrixXi r(pointCloud.height(), pointCloud.width());
	Eigen::MatrixXi g(pointCloud.height(), pointCloud.width());
	Eigen::MatrixXi b(pointCloud.height(), pointCloud.width());
	Eigen::MatrixXf contrast(pointCloud.height(), pointCloud.width());

	for (int i = 0; i < pointCloud.height(); i++)
	{
		for (int j = 0; j < pointCloud.width(); j++)
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

	Eigen::MatrixXi redDownsampled = downsample_and_round(r, downsamplingFactor);
	Eigen::MatrixXi greenDownsampled = downsample_and_round(g, downsamplingFactor);
	Eigen::MatrixXi blueDownsampled = downsample_and_round(b, downsamplingFactor);

	std::function<float(float)> is_not_nan_functor = [](float f) {return !std::isnan(f); };
	std::function<float(float)> nan_to_zero_functor = nan_to_zero;

	Eigen::MatrixXf contrastNulled = (z.unaryExpr(is_not_nan_functor)).cwiseProduct(contrast.unaryExpr(nan_to_zero_functor));

	auto contrastWeight = grid_sum(contrastNulled, downsamplingFactor);

	Eigen::MatrixXf xContrasted = x.cwiseProduct(contrastNulled);
	Eigen::MatrixXf yContrasted = y.cwiseProduct(contrastNulled);
	Eigen::MatrixXf zContrasted = z.cwiseProduct(contrastNulled);
	Eigen::MatrixXf contrastContrasted = contrast.cwiseProduct(contrastNulled);

	Eigen::MatrixXf xDownsampled = grid_sum(xContrasted, downsamplingFactor).cwiseQuotient(contrastWeight);
	Eigen::MatrixXf yDownsampled = grid_sum(yContrasted, downsamplingFactor).cwiseQuotient(contrastWeight);
	Eigen::MatrixXf zDownsampled = grid_sum(zContrasted, downsamplingFactor).cwiseQuotient(contrastWeight);
	Eigen::MatrixXf contrastDownsampled = grid_sum(contrastContrasted, downsamplingFactor).cwiseQuotient(contrastWeight);

	Zivid::PointCloud pointCloudDownsampled(redDownsampled.rows(), redDownsampled.cols());

	for (int i = 0; i < redDownsampled.rows(); i++)
	{
		for (int j = 0; j < redDownsampled.cols(); j++)
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

Eigen::MatrixXi downsample_and_round(const Eigen::MatrixXi& matrixi, int downsamplingFactor)
{
	Eigen::MatrixXf matrixf = matrixi.template cast<float>();

	std::function<float(float)> round_functor = [](float f) {return std::round(f); };

	return ((grid_sum(matrixf, downsamplingFactor) / (downsamplingFactor*downsamplingFactor)).unaryExpr(round_functor)).template cast<int>();
}

Eigen::MatrixXf grid_sum(const Eigen::MatrixXf& r, int downsamplingFactor)
{
	return line_sum(line_sum(r, downsamplingFactor), downsamplingFactor);
}

Eigen::MatrixXf line_sum(const Eigen::MatrixXf& matrix, int downsamplingFactor)
{
	Eigen::MatrixXf reshapedMatrix = matrix.reshaped(downsamplingFactor, (int)matrix.rows() * (int)matrix.cols() / downsamplingFactor);

	std::function<float(float)> nan_to_zero_functor = nan_to_zero;

	reshapedMatrix = reshapedMatrix.unaryExpr(nan_to_zero_functor);

	return reshapedMatrix.colwise().sum().reshaped((int)matrix.rows() / downsamplingFactor, (int)matrix.cols()).transpose();
}

float nan_to_zero(float x)
{
	if (std::isnan(x))
	{
		return 0;
	}
	else
	{
		return x;
	}
}

void visualize_point_cloud(const Zivid::PointCloud& pointCloud, Zivid::Application& zivid)
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

