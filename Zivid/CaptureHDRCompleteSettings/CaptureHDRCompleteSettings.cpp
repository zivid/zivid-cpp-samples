/*
This example shows how to acquire an HDR image from the Zivid camera with fully
configured settings for each frame. In general, taking an HDR image is a lot
simpler than this as the default settings work for most scenes. The purpose of
this example is to demonstrate how to configure all the settings.
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

		camera << Zivid::Settings::Iris{ 10U }
			<< Zivid::Settings::ExposureTime{ std::chrono::microseconds{ 10000 } }
			<< Zivid::Settings::Brightness{ 1 }
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

		frames.emplace_back(camera.capture());
		std::cout << "Frame 1 " << camera.settings() << std::endl;

		camera << Zivid::Settings::Iris{ 20U }
			<< Zivid::Settings::ExposureTime{ std::chrono::microseconds{ 20000 } }
			<< Zivid::Settings::Brightness{ 0.5 }
			<< Zivid::Settings::Gain{ 2 }
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
		frames.emplace_back(camera.capture());
		std::cout << "Frame 2 " << camera.settings() << std::endl;

		camera << Zivid::Settings::Iris{ 30U }
			<< Zivid::Settings::ExposureTime{ std::chrono::microseconds{ 33000 } }
			<< Zivid::Settings::Brightness{ 1 }
			<< Zivid::Settings::Gain{ 1 }
			<< Zivid::Settings::Bidirectional{ true }
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
		frames.emplace_back(camera.capture());
		std::cout << "Frame 3 " << camera.settings() << std::endl;

		std::cout << "Creating the HDR frame" << std::endl;
		auto hdrFrame = Zivid::HDR::combineFrames(begin(frames), end(frames));

		std::cout << "Saving the frames" << std::endl;
		frames[0].save("10.zdf");
		frames[1].save("20.zdf");
		frames[2].save("30.zdf");
		hdrFrame.save("HDR.zdf");

		std::cout << "Displaying the frames" << std::endl;
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
