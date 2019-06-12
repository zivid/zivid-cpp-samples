/*
This example shows how to acquire HDR images from the Zivid camera in a loop
(while actively changing some HDR settings).
*/

#include <Zivid/CloudVisualizer.h>
#include <Zivid/Zivid.h>

#include <iostream>

int main()
{
	try
	{
		Zivid::Application zivid;

		std::cout << "Setting up visualization" << std::endl;
		Zivid::CloudVisualizer vis;
		zivid.setDefaultComputeDevice(vis.computeDevice());

		std::cout << "Connecting to the camera" << std::endl;
		auto camera = zivid.connectCamera();

		std::cout << "Recording HDR source images" << std::endl;
		std::vector<Zivid::Frame> frames;

		camera << Zivid::Settings::Brightness{ 1 }
			<< Zivid::Settings::Gain{ 1 }
			<< Zivid::Settings::Bidirectional{ false }
			<< Zivid::Settings::Filters::Contrast::Enabled::yes
			<< Zivid::Settings::Filters::Contrast::Threshold{ 5 }
			<< Zivid::Settings::Filters::Gaussian::Enabled::yes
			<< Zivid::Settings::Filters::Gaussian::Sigma{ 1.5 }
			<< Zivid::Settings::Filters::Outlier::Enabled::yes
			<< Zivid::Settings::Filters::Outlier::Threshold{ 5 }
			<< Zivid::Settings::Filters::Reflection::Enabled::yes
			<< Zivid::Settings::Filters::Saturated::Enabled::yes
			<< Zivid::Settings::BlueBalance{ 1.081 }
		<< Zivid::Settings::RedBalance{ 1.709 };

		size_t iris[3] = { 10U, 20U, 30U };
		int exposureTime[3] = { 10000, 20000, 30000 };

		for (size_t i = 0; i < 3; ++i)
		{
			camera << Zivid::Settings::Iris{ iris[i] }
			<< Zivid::Settings::ExposureTime{ std::chrono::microseconds{ exposureTime[i] } };

			frames.emplace_back(camera.capture());
			std::cout << "Frame " << i << " " << camera.settings() << std::endl;
		}

		std::cout << "Creating the HDR frame" << std::endl;
		auto hdrFrame = Zivid::HDR::combineFrames(begin(frames), end(frames));

		std::cout << "Saving the frames" << std::endl;
		frames[0].save("20.zdf");
		frames[1].save("25.zdf");
		frames[2].save("30.zdf");
		hdrFrame.save("HDR.zdf");

		std::cout << "Displaying the HDR frame" << std::endl;
		vis.showMaximized();
		vis.show(hdrFrame);
		vis.resetToFit();

		std::cout << "Running the visualizer. Blocking until the window closes" << std::endl;
		vis.run();
	}
	catch (const std::exception &e)
	{
		std::cerr << "Error: " << Zivid::toString(e) << std::endl;
		return EXIT_FAILURE;
	}
}
