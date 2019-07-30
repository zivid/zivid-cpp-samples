/*
Import a ZDF point cloud and downsample it.
*/

#include <Zivid/CloudVisualizer.h>
#include <Zivid/Zivid.h>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <iostream>
Eigen::MatrixXf sumline(Eigen::MatrixXf&, int, int);
Eigen::MatrixXf gridsum(Eigen::MatrixXf&, int);
float myround(float); 
float isnanmask(float);
float nantozero(float);
Zivid::PointCloud downsample(Zivid::PointCloud&, int);

int main()
{
	try
	{
		Zivid::Application zivid;

		std::string Filename = "Zivid3D.zdf";
		std::cout << "Reading " << Filename << " point cloud" << std::endl;
		Zivid::Frame frame = Zivid::Frame(Filename);

		auto pointCloud = frame.getPointCloud();
		auto downSamplingFactor = 4;

		Zivid::PointCloud pointCloudDownsampled(pointCloud.height()/downSamplingFactor, pointCloud.width()/downSamplingFactor);

		pointCloudDownsampled = downsample(pointCloud, downSamplingFactor);

		std::cout << "Setting up visualization" << std::endl;
		{
			Zivid::CloudVisualizer vis;
			zivid.setDefaultComputeDevice(vis.computeDevice());

			std::cout << "Displaying the original point cloud" << std::endl;
			vis.showMaximized();
			vis.show(pointCloud);
			vis.resetToFit();

			std::cout << "Running the visualizer. Blocking until the window closes" << std::endl;
			vis.run();
		}
		{
			Zivid::CloudVisualizer vis;
			zivid.setDefaultComputeDevice(vis.computeDevice());

			std::cout << "Displaying the downsampled point cloud" << std::endl;
			vis.showMaximized();
			vis.show(pointCloudDownsampled);
			vis.resetToFit();

			std::cout << "Running the visualizer. Blocking until the window closes" << std::endl;
			vis.run();
		}
	}

	catch (const std::exception &e)
	{
		std::cerr << "Error: " << Zivid::toString(e) << std::endl;
		return EXIT_FAILURE;
	}
}

Eigen::MatrixXf sumline(Eigen::MatrixXf& matrix, int downSamplingFactor)
{
	Eigen::MatrixXf reshapedMatrix = matrix.reshaped(downSamplingFactor, (int)matrix.rows() * (int)matrix.cols() / downSamplingFactor);
	
	std::function<float(float)> nan_to_zero_wrap = nantozero;

	reshapedMatrix = reshapedMatrix.unaryExpr(nan_to_zero_wrap);

	return reshapedMatrix.colwise().sum().reshaped((int)matrix.rows() / downSamplingFactor, (int)matrix.cols()).transpose();
}

Eigen::MatrixXf gridsum(Eigen::MatrixXf& r, int downSamplingFactor)
{
	return sumline(sumline(r, downSamplingFactor), downSamplingFactor);
}

float myround(float x)
{
	return std::round(x);
}

float isnanmask(float x)
{
	if (std::isnan(x))
	{
		return 0;
	}
	else
	{
		return 1;
	}
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

Zivid::PointCloud downsample(Zivid::PointCloud& pointCloud, int downSamplingFactor)
{
	/*
	Function for downsampling a Zivid point cloud. The downsampling factor represents the denominator of a fraction that represents the size of the downsampled point cloud relative to the original point cloud, e.g. 2 - one-half,  3 - one-third, 4 one-quarter, etc.
	*/

	if ((downSamplingFactor % 2 != 0) || (pointCloud.height() % downSamplingFactor) || (pointCloud.width() % downSamplingFactor))
	{
		throw std::invalid_argument("Downsampling factor has to have one of the following values: 2, 3, 4, 5, 6.");
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

	Eigen::MatrixXf rf = r.template cast<float>();
	Eigen::MatrixXf gf = g.template cast<float>();
	Eigen::MatrixXf bf = b.template cast<float>();

	std::function<float(float)> round_wrap = myround;

	Eigen::MatrixXi redDownsampled = ((gridsum(rf, downSamplingFactor) / (downSamplingFactor*downSamplingFactor)).unaryExpr(round_wrap)).template cast<int>();
	Eigen::MatrixXi greenDownsampled = ((gridsum(gf, downSamplingFactor) / (downSamplingFactor*downSamplingFactor)).unaryExpr(round_wrap)).template cast<int>();
	Eigen::MatrixXi blueDownsampled = ((gridsum(bf, downSamplingFactor) / (downSamplingFactor*downSamplingFactor)).unaryExpr(round_wrap)).template cast<int>();

	std::function<float(float)> is_nan_mask_wrap = isnanmask;
	std::function<float(float)> nan_to_zero_wrap = nantozero;

	Eigen::MatrixXf contrastNulled = (z.unaryExpr(is_nan_mask_wrap)).cwiseProduct(contrast.unaryExpr(nan_to_zero_wrap));

	// Contrast downsampling is WIP

	/*auto contrastWeight = gridsum(contrastNulled, downSamplingFactor);
	Eigen::MatrixXf contrastContrasted = contrast.cwiseProduct(contrastNulled);
	Eigen::MatrixXf contrastDownsampled = gridsum(contrastContrasted, downSamplingFactor).cwiseQuotient(gridsum(contrastNulled, downSamplingFactor));
	std::cout << contrast({ 0,1,2,3,4,5,6,7 }, { 0,1,2,3,4,5,6,7 }) << std::endl;
	std::cout << contrastNulled({ 0,1,2,3,4,5,6,7 }, { 0,1,2,3,4,5,6,7 }) << std::endl;
	std::cout << contrastContrasted({ 0,1,2,3,4,5,6,7 }, { 0,1,2,3,4,5,6,7 }) << std::endl;
	std::cout << contrastWeight({ 0,1,2,3,4,5,6,7 }, { 0,1,2,3,4,5,6,7 }) << std::endl;
	std::cout << contrastDownsampled({ 0,1, }, { 0,1, }) << std::endl;*/

	Eigen::MatrixXf xContrasted = x.cwiseProduct(contrastNulled);
	Eigen::MatrixXf yContrasted = y.cwiseProduct(contrastNulled);
	Eigen::MatrixXf zContrasted = z.cwiseProduct(contrastNulled);

	Eigen::MatrixXf xDownsampled = gridsum(xContrasted, downSamplingFactor).cwiseQuotient(gridsum(contrastNulled, downSamplingFactor));
	Eigen::MatrixXf yDownsampled = gridsum(yContrasted, downSamplingFactor).cwiseQuotient(gridsum(contrastNulled, downSamplingFactor));
	Eigen::MatrixXf zDownsampled = gridsum(zContrasted, downSamplingFactor).cwiseQuotient(gridsum(contrastNulled, downSamplingFactor));

	Zivid::PointCloud pointCloudDownsampled(redDownsampled.rows(), redDownsampled.cols());

	for (int i = 0; i < redDownsampled.rows(); i++)
	{
		for (int j = 0; j < redDownsampled.cols(); j++)
		{
			pointCloudDownsampled(i, j).setRgb(redDownsampled(i, j), greenDownsampled(i, j), blueDownsampled(i, j));
			//pointCloudDownsampled(i, j).setContrast(contrastDownsampled(i,j));
			pointCloudDownsampled(i, j).x = xDownsampled(i, j);
			pointCloudDownsampled(i, j).y = yDownsampled(i, j);
			pointCloudDownsampled(i, j).z = zDownsampled(i, j);
		}
	}

	return pointCloudDownsampled;
}