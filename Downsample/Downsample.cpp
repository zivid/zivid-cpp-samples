/*
Import a ZDF point cloud and downsample it.
*/

#include <Zivid/CloudVisualizer.h>
#include <Zivid/Zivid.h>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <iostream>
Zivid::PointCloud downsample(const Zivid::PointCloud&, int);
Eigen::MatrixXi downsampleandround(const Eigen::MatrixXi&, int);
Eigen::MatrixXf gridsum(const Eigen::MatrixXf&, int);
Eigen::MatrixXf sumline(const Eigen::MatrixXf&, int);
float nantozero(float);
void visualizepointcloud(const Zivid::PointCloud&, Zivid::Application&);

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

		visualizepointcloud(pointCloud,zivid);
		visualizepointcloud(pointCloudDownsampled,zivid);
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
	Function for downsampling a Zivid point cloud. The downsampling factor represents the denominator of a fraction that represents the size of the downsampled point cloud relative to the original point cloud, e.g. 2 - one-half,  3 - one-third, 4 one-quarter, etc.
	*/

	if ((pointCloud.height() % downsamplingFactor) || (pointCloud.width() % downsamplingFactor))
	{
		throw std::invalid_argument("Downsampling factor (" + std::to_string(downsamplingFactor) + ") has to be divisible by the width (" + std::to_string(pointCloud.width()) + ") and height (" + std::to_string(pointCloud.height()) + ") of the input point cloud.");
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

	Eigen::MatrixXi redDownsampled = downsampleandround(r, downsamplingFactor);
	Eigen::MatrixXi greenDownsampled = downsampleandround(g, downsamplingFactor);
	Eigen::MatrixXi blueDownsampled = downsampleandround(b, downsamplingFactor);

	std::function<float(float)> is_not_nan_functor = [](float f) {return !std::isnan(f); };
	std::function<float(float)> nan_to_zero_functor = nantozero;

	Eigen::MatrixXf contrastNulled = (z.unaryExpr(is_not_nan_functor)).cwiseProduct(contrast.unaryExpr(nan_to_zero_functor));

	auto contrastWeight = gridsum(contrastNulled, downsamplingFactor);

	Eigen::MatrixXf xContrasted = x.cwiseProduct(contrastNulled);
	Eigen::MatrixXf yContrasted = y.cwiseProduct(contrastNulled);
	Eigen::MatrixXf zContrasted = z.cwiseProduct(contrastNulled);
	Eigen::MatrixXf contrastContrasted = contrast.cwiseProduct(contrastNulled);

	Eigen::MatrixXf xDownsampled = gridsum(xContrasted, downsamplingFactor).cwiseQuotient(contrastWeight);
	Eigen::MatrixXf yDownsampled = gridsum(yContrasted, downsamplingFactor).cwiseQuotient(contrastWeight);
	Eigen::MatrixXf zDownsampled = gridsum(zContrasted, downsamplingFactor).cwiseQuotient(contrastWeight);
	Eigen::MatrixXf contrastDownsampled = gridsum(contrastContrasted, downsamplingFactor).cwiseQuotient(contrastWeight);

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

Eigen::MatrixXi downsampleandround(const Eigen::MatrixXi& matrixi, int downsamplingFactor)
{
	Eigen::MatrixXf matrixf = matrixi.template cast<float>();

	std::function<float(float)> round_wrap = [](float f) {return std::round(f); };

	return ((gridsum(matrixf, downsamplingFactor) / (downsamplingFactor*downsamplingFactor)).unaryExpr(round_wrap)).template cast<int>();
}

Eigen::MatrixXf gridsum(const Eigen::MatrixXf& r, int downsamplingFactor)
{
	return sumline(sumline(r, downsamplingFactor), downsamplingFactor);
}

Eigen::MatrixXf sumline(const Eigen::MatrixXf& matrix, int downsamplingFactor)
{
	Eigen::MatrixXf reshapedMatrix = matrix.reshaped(downsamplingFactor, (int)matrix.rows() * (int)matrix.cols() / downsamplingFactor);

	std::function<float(float)> nan_to_zero_functor = nantozero;

	reshapedMatrix = reshapedMatrix.unaryExpr(nan_to_zero_functor);

	return reshapedMatrix.colwise().sum().reshaped((int)matrix.rows() / downsamplingFactor, (int)matrix.cols()).transpose();
}

float nantozero(float x)
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

void visualizepointcloud(const Zivid::PointCloud& pointCloud, Zivid::Application& zivid)
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

