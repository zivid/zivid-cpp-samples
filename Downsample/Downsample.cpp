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
Zivid::PointCloud downsample(Zivid::PointCloud&, int);
float isnanmask(float);
float myround(float);

int main()
{
	try
	{
		Zivid::Application zivid;

		std::cout << "Setting up visualization" << std::endl;
		//Zivid::CloudVisualizer vis;
		//zivid.setDefaultComputeDevice(vis.computeDevice());

		std::string Filename = "Zivid3D.zdf";
		std::cout << "Reading " << Filename << " point cloud" << std::endl;
		Zivid::Frame frame = Zivid::Frame(Filename);

		auto pointCloud = frame.getPointCloud();
		auto dsf = 4;

		Zivid::PointCloud pointCloudDownsampled(pointCloud.height()/dsf, pointCloud.width()/dsf);

		pointCloudDownsampled = downsample(pointCloud, dsf);

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

Eigen::MatrixXf sumline(Eigen::MatrixXf& matrix, int dsf)
{
	Eigen::MatrixXf reshapedMatrix = matrix.reshaped(dsf, (int)matrix.rows() * (int)matrix.cols() / dsf);

	//I don't know a better way of doing this
	//reshapedMatrix = reshapedMatrix.unaryExpr([](float v) { return std::isfinite(v) ? v : 0.0; });

	for (int i = 0; i < reshapedMatrix.size(); ++i)
	{
		if (std::isnan(reshapedMatrix(i)))
		{
			reshapedMatrix(i) = 0;
		}
	}

	return reshapedMatrix.colwise().sum().reshaped((int)matrix.rows() / dsf, (int)matrix.cols()).transpose();
}

Eigen::MatrixXf gridsum(Eigen::MatrixXf& r, int dsf)
{
	return sumline(sumline(r, dsf), dsf);
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

Zivid::PointCloud downsample(Zivid::PointCloud& pointCloud, int dsf)
{
	/*
	Function for downsampling a Zivid point cloud. The input argument 'dsf' is the downsampling factor that represents the denominator of a fraction that represents the size of the downsampled point cloud relative to the original point cloud, e.g. 2 - one-half,  3 - one-third, 4 one-quarter, etc.
	*/

	Eigen::MatrixXf x(pointCloud.height(), pointCloud.width());
	Eigen::MatrixXf y(pointCloud.height(), pointCloud.width());
	Eigen::MatrixXf z(pointCloud.height(), pointCloud.width());
	Eigen::MatrixXi r(pointCloud.height(), pointCloud.width());
	Eigen::MatrixXi g(pointCloud.height(), pointCloud.width());
	Eigen::MatrixXi b(pointCloud.height(), pointCloud.width());
	Eigen::MatrixXf contrast(pointCloud.height(), pointCloud.width());
	Eigen::MatrixXf zeros(pointCloud.height(), pointCloud.width());
	Eigen::MatrixXf ones(pointCloud.height(), pointCloud.width());
	Eigen::MatrixXf nanm(pointCloud.height(), pointCloud.width());

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
			zeros(i, j) = 0;
			ones(i, j) = 1;
			nanm(i, j) = std::nan("1");
		}
	}

	Eigen::MatrixXf rf = r.template cast<float>();
	Eigen::MatrixXf gf = g.template cast<float>();
	Eigen::MatrixXf bf = b.template cast<float>();

	//Eigen::MatrixXf rfA = gridsum(rf, dsf) / (dsf*dsf);
	//Eigen::MatrixXf bfA = gridsum(gf, dsf) / (dsf*dsf);
	//Eigen::MatrixXf gfA = gridsum(bf, dsf) / (dsf*dsf);

	std::function<float(float)> round_wrap = myround;

	Eigen::MatrixXf rfAA = (gridsum(rf, dsf) / (dsf*dsf)).unaryExpr(round_wrap);
	Eigen::MatrixXf bfAA = (gridsum(gf, dsf) / (dsf*dsf)).unaryExpr(round_wrap);
	Eigen::MatrixXf gfAA = (gridsum(bf, dsf) / (dsf*dsf)).unaryExpr(round_wrap);

	Eigen::MatrixXi redDownsampled = (rfAA).template cast<int>();
	Eigen::MatrixXi greenDownsampled = (bfAA).template cast<int>();
	Eigen::MatrixXi blueDownsampled = (gfAA).template cast<int>();

	std::function<float(float)> is_nan_mask_wrap = isnanmask;
	Eigen::MatrixXf nanMask = z.unaryExpr(is_nan_mask_wrap);

	contrast = nanMask.cwiseProduct(contrast);
	auto contrastWeight = gridsum(contrast, dsf);

	Eigen::MatrixXf xc = x.cwiseProduct(contrast);
	Eigen::MatrixXf yc = y.cwiseProduct(contrast);
	Eigen::MatrixXf zc = z.cwiseProduct(contrast);

	Eigen::MatrixXf X_new = gridsum(xc, dsf).cwiseQuotient(contrastWeight);
	Eigen::MatrixXf Y_new = gridsum(yc, dsf).cwiseQuotient(contrastWeight);
	Eigen::MatrixXf Z_new = gridsum(zc, dsf).cwiseQuotient(contrastWeight);

	Zivid::PointCloud pointCloudDownsampled(redDownsampled.rows(), redDownsampled.cols());

	for (int i = 0; i < redDownsampled.rows(); i++)
	{
		for (int j = 0; j < redDownsampled.cols(); j++)
		{
			pointCloudDownsampled(i, j).setRgb(redDownsampled(i, j), greenDownsampled(i, j), blueDownsampled(i, j));
			pointCloudDownsampled(i, j).setContrast(1);
			pointCloudDownsampled(i, j).x = X_new(i, j);
			pointCloudDownsampled(i, j).y = Y_new(i, j);
			pointCloudDownsampled(i, j).z = Z_new(i, j);
		}
	}

	return pointCloudDownsampled;
}