/*

*/

#include <Zivid/CloudVisualizer.h>
#include <Zivid/Zivid.h>
#include <Eigen/Dense>

#include <iostream>

Eigen::MatrixXf sumline(Eigen::MatrixXf&, int, int);
Eigen::MatrixXf gridsum( Eigen::MatrixXf&, int);

int main()
{
	try
	{
		

		Eigen::MatrixXf a(2,2);
		a(0, 0) = NAN;
		a(0, 1) = 1;
		a(1, 1) = 2;
		a(1, 0) = 3;
		
		Eigen::MatrixXf b2(2, 2);
		b2(0, 0) = 234;
		b2(0, 1) = NAN;
		b2(1, 1) = 2;
		b2(1, 0) = 3;

		auto c = a + b2;
		std::cout << c << std::endl;

		Zivid::Application zivid;

		std::cout << "Setting up visualization" << std::endl;
		Zivid::CloudVisualizer vis;
		zivid.setDefaultComputeDevice(vis.computeDevice());

		std::string Filename = "Zivid3D.zdf";
		std::cout << "Reading " << Filename << " point cloud" << std::endl;
		Zivid::Frame frame = Zivid::Frame(Filename);

		auto pointCloud = frame.getPointCloud();
		/*auto point = pointCloud(2,2);
		auto x = point.x;
		auto y = point.y;
		auto z = point.z;
		auto r = point.red();
		auto g = point.green();
		auto b = point.blue();
		auto contrast = point.contrast;*/

		
		auto rows = pointCloud.height();
		auto columns = pointCloud.width();

		Eigen::MatrixXf x(rows, columns);
		Eigen::MatrixXf y(rows, columns);
		Eigen::MatrixXf z(rows, columns);
		Eigen::MatrixXi r(rows, columns);
		Eigen::MatrixXi g(rows, columns);
		Eigen::MatrixXi b(rows, columns);
		Eigen::MatrixXf contrast(rows, columns);

		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < columns; j++)
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
		std::cout << "contrast" << std::endl;
		std::cout << contrast({ 0,1,2,3 }, { 0,1,2,3,4,5 }) << std::endl;

		auto dsf = 4;

		Eigen::MatrixXf rf = r.template cast<float>();
		Eigen::MatrixXf gf = g.template cast<float>();
		Eigen::MatrixXf bf = b.template cast<float>();

		//auto rff = gridsum(rf, dsf);
		//auto gff = gridsum(gf, dsf);
		//auto bff = gridsum(bf, dsf);

		//Eigen::MatrixXf rff = gridsum(rf, dsf) / (dsf*dsf);
		//Eigen::MatrixXf gff = gridsum(gf, dsf) / (dsf*dsf);
		//Eigen::MatrixXf bff = gridsum(bf, dsf) / (dsf*dsf);
		
		//the problem is that going to int is always floor! not good. need to round to closest.
		Eigen::MatrixXi redDownsampled = (gridsum(rf, dsf) / (dsf*dsf)).template cast<int>();
		Eigen::MatrixXi greenDownsampled = (gridsum(gf, dsf) / (dsf*dsf)).template cast<int>();
		Eigen::MatrixXi blueDownsampled = (gridsum(bf, dsf) / (dsf*dsf)).template cast<int>();

		std::cout << "red"<< std::endl;
		std::cout << redDownsampled({ 0,1,2,3 }, { 0,1,2,3,4,5 }) << std::endl;
		std::cout << "green" << std::endl;
		std::cout << greenDownsampled({ 0,1,2,3 }, { 0,1,2,3,4,5 }) << std::endl;
		std::cout << "blue" << std::endl;
		std::cout << blueDownsampled({ 0,1,2,3 }, { 0,1,2,3,4,5 }) << std::endl;

		for (int i = 0; i < contrast.size(); ++i)
		{
			if (std::isnan(z(i)))
			{
				contrast(i) = 0;
			}
		}
		auto contrastWeight = gridsum(contrast, dsf);

		Eigen::MatrixXf xc = x.cwiseProduct(contrast);
		Eigen::MatrixXf yc = y.cwiseProduct(contrast);
		Eigen::MatrixXf zc = z.cwiseProduct(contrast);

		Eigen::MatrixXf X_new = gridsum(xc, dsf).cwiseQuotient(contrastWeight);
		Eigen::MatrixXf Y_new = gridsum(yc, dsf).cwiseQuotient(contrastWeight);
		Eigen::MatrixXf Z_new = gridsum(zc, dsf).cwiseQuotient(contrastWeight);
		
		std::cout << "x new" << std::endl;
		std::cout << X_new({ 0,1,2,3 }, { 0,1,2,3,4,5 }) << std::endl;
		std::cout << "y new" << std::endl;
		std::cout << Y_new({ 0,1,2,3 }, { 0,1,2,3,4,5 }) << std::endl;
		std::cout << "z new" << std::endl;
		std::cout << Z_new({ 0,1,2,3 }, { 0,1,2,3,4,5 }) << std::endl;
		
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

		auto frameNew = Zivid::Frame();

		/*std::cout << "Displaying the frame" << std::endl;
		vis.showMaximized();
		vis.show(frame);
		vis.resetToFit();

		std::cout << "Running the visualizer. Blocking until the window closes" << std::endl;
		vis.run();*/
	}
	catch (const std::exception &e)
	{
		std::cerr << "Error: " << Zivid::toString(e) << std::endl;
		return EXIT_FAILURE;
	}
}

int downsample(int a)
{
	
	return 1;

}
Eigen::MatrixXf sumline(Eigen::MatrixXf& matrix, int dsf)
{
	//auto rs = (int)matrix.rows() * (int)matrix.cols() / dsf;
	//Eigen::MatrixXf temp = matrix.reshaped(dsf, (int)matrix.rows() * (int)matrix.cols() / dsf);
	//auto s3 = (int)matrix.cols();
	//auto s2 = (int)matrix.rows() / dsf;
	//Eigen::MatrixXf s1 = matrix.reshaped(dsf, (int)matrix.rows() * (int)matrix.cols() / dsf).colwise().sum();
	//ovaj sum ovde ne ignorise nanove
	Eigen::MatrixXf o = matrix.reshaped(dsf, (int)matrix.rows() * (int)matrix.cols() / dsf);

	for (int i = 0; i < o.size(); ++i)
	{
		if (std::isnan(o(i)))
		{
			o(i) = 0;
		}
	}

	return o.colwise().sum().reshaped((int)matrix.rows() / dsf, (int)matrix.cols()).transpose();
	//Eigen::MatrixXf g2 = s0.transpose();
	//return matrixOut;
}

Eigen::MatrixXf gridsum(Eigen::MatrixXf& r, int dsf)
{
	/*auto rs = 1200 * 1920 / dsf;
	Eigen::MatrixXf temp = r.reshaped(dsf, rs);
	auto s3 = 1920;
	auto s2 = 1200 / dsf;
	Eigen::MatrixXf s1 = temp.colwise().sum();
	Eigen::MatrixXf s0 = s1.reshaped(s2, s3);
	Eigen::MatrixXf g2 = s0.transpose();

	rs = 1920 * 1200 / dsf / dsf;
	Eigen::MatrixXf s44 = g2.reshaped(dsf, rs);
	auto s33 = 1200 / dsf;
	auto s22 = 1920 / dsf;
	Eigen::MatrixXf s11 = s44.colwise().sum();
	Eigen::MatrixXf s00 = s11.reshaped(s22, s33);	
	Eigen::MatrixXf ff = s00.transpose();*/


	//Eigen::MatrixXf sgf = sumline(r,dsf);
	//Eigen::MatrixXf sgf1 = sgf.transpose();
	return sumline(sumline(r, dsf), dsf);
	//Eigen::MatrixXf sgf2 = sgf22.transpose();

	/*Eigen::MatrixXf sum1 = sumline(r, dsf).transpose();
	Eigen::MatrixXf sum2 = sumline(sum1, dsf).transpose();
	return sum2;*/
}

/*template<typename Derived>
inline bool is_nan(const Eigen::MatrixBase<Derived>& x)
{
	return ((x.array() == x.array())).all();
}*/