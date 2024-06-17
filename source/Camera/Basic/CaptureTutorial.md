# Capture Tutorial

Note\! This tutorial has been generated for use on Github. For original
tutorial see:
[CaptureTutorial](https://support.zivid.com/latest/academy/camera/capture-tutorial.html)



---

*Contents:*
[**Introduction**](#Introduction) |
[**Initialize**](#Initialize) |
[**Connect**](#Connect) |
[**Configure**](#Configure) |
[**Capture**](#Capture) |
[**Save**](#Save) |
[**Multithreading**](#Multithreading) |
[**Conclusion**](#Conclusion)

---



## Introduction

This tutorial describes how to use the Zivid SDK to capture point clouds
and 2D images.

For MATLAB see [Zivid Capture Tutorial for
MATLAB](https://github.com/zivid/zivid-matlab-samples/blob/master/source/Camera/Basic/CaptureTutorial.md).

-----

Tip:

> If you prefer watching a video, our webinar [Making 3D captures easy -
> A tour of Zivid Studio and Zivid
> SDK](https://www.zivid.com/webinars-page?wchannelid=ffpqbqc7sg&wmediaid=ce68dbjldk)
> covers the same content as the Capture Tutorial.

**Prerequisites**

  - Install [Zivid
    Software](https://support.zivid.com/latest//getting-started/software-installation.html).
  - For Python: install
    [zivid-python](https://github.com/zivid/zivid-python#installation)

## Initialize

Calling any of the APIs in the Zivid SDK requires initializing the Zivid
application and keeping it alive while the program runs.

-----

Note:

`Zivid::Application` must be kept alive while operating the Zivid
Camera. This is essentially the Zivid driver.

-----

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L13))

``` sourceCode cpp
Zivid::Application zivid;
```

## Connect

Now we can connect to the camera.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L16))

``` sourceCode cpp
auto camera = zivid.connectCamera();
```

### Specific Camera

Sometimes multiple cameras are connected to the same computer, but it
might be necessary to work with a specific camera in the code. This can
be done by providing the serial number of the wanted camera.

``` sourceCode cpp
auto camera = zivid.connectCamera(Zivid::CameraInfo::SerialNumber{ "2020C0DE" });
```

-----

Note:

> The serial number of your camera is shown in the Zivid Studio.

-----

You may also list all cameras connected to the computer, and view their
serial numbers through

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/InfoUtilOther/CameraInfo/CameraInfo.cpp#L16-L22))

``` sourceCode cpp
auto cameras = zivid.cameras();
std::cout << "Found " << cameras.size() << " cameras" << std::endl;
for(auto &camera : cameras)
{
	std::cout << "Camera Info: " << camera.info() << std::endl;
	std::cout << "Camera State: " << camera.state() << std::endl;
}
```

### File Camera

The file camera option allows you to experiment with the SDK without
access to a physical camera. The file cameras can be found in [Sample
Data](https://support.zivid.com/latest/api-reference/samples/sample-data.html)
where there are multiple file cameras to choose from. Each file camera
demonstrates a use case within one of the main applications of the
respective camera model. The example below shows how to create a file
camera using the Zivid 2 M70 file camera from [Sample
Data](https://support.zivid.com/latest/api-reference/samples/sample-data.html).

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureFromFileCamera/CaptureFromFileCamera.cpp#L36-L37))

``` sourceCode cpp
const auto fileCamera =
	userInput ? fileCameraPath : std::string(ZIVID_SAMPLE_DATA_DIR) + "/FileCameraZivid2M70.zfc";
```

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureFromFileCamera/CaptureFromFileCamera.cpp#L40))

``` sourceCode cpp
auto camera = zivid.createFileCamera(fileCamera);
```

The acquisition settings should be initialized like shown below, but you
are free to alter the processing settings.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureFromFileCamera/CaptureFromFileCamera.cpp#L43-L50))

``` sourceCode cpp
const auto settings = Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} },
									Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
									Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{ 1.5 },
									Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes,
									Zivid::Settings::Processing::Filters::Reflection::Removal::Mode::global,
									Zivid::Settings::Processing::Color::Balance::Red{ 1 },
									Zivid::Settings::Processing::Color::Balance::Green{ 1 },
									Zivid::Settings::Processing::Color::Balance::Blue{ 1 } };
```

You can read more about the file camera option in [File
Camera](https://support.zivid.com/latest/academy/camera/file-camera.html).

## Configure

As with all cameras there are settings that can be configured.

### Presets

The recommendation is to use
[Presets](https://support.zivid.com/latest/reference-articles/presets-settings.html)
available in Zivid Studio and as .yml files (see `load_yml_label`
below). Alternatively, you can use our Capture Assistant, or you may
configure the settings manually.

### Capture Assistant

It can be difficult to know what settings to configure. Luckily we have
the Capture Assistant. This is available in the Zivid SDK to help
configure camera settings.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureAssistant/CaptureAssistant.cpp#L19-L25))

``` sourceCode cpp
const auto suggestSettingsParameters = Zivid::CaptureAssistant::SuggestSettingsParameters{
	Zivid::CaptureAssistant::SuggestSettingsParameters::AmbientLightFrequency::none,
	Zivid::CaptureAssistant::SuggestSettingsParameters::MaxCaptureTime{ std::chrono::milliseconds{ 1200 } }
};
std::cout << "Running Capture Assistant with parameters:\n" << suggestSettingsParameters << std::endl;
auto settings = Zivid::CaptureAssistant::suggestSettings(camera, suggestSettingsParameters);
```

There are only two parameters to configure with Capture Assistant:

1.  **Maximum Capture Time** in number of milliseconds.
    1.  Minimum capture time is 200 ms. This allows only one
        acquisition.
    2.  The algorithm will combine multiple acquisitions if the budget
        allows.
    3.  The algorithm will attempt to cover as much of the dynamic range
        in the scene as possible.
    4.  A maximum capture time of more than 1 second will get good
        coverage in most scenarios.
2.  **Ambient light compensation**
    1.  May restrict capture assistant to exposure periods that are
        multiples of the ambient light period.
    2.  60Hz is found in Japan, Americas, Taiwan, South Korea and
        Philippines.
    3.  50Hz is common in the rest of the world.

### Manual configuration

Another option is to configure settings manually. For more information
about what each settings does, please see [Camera
Settings](https://support.zivid.com/latest/reference-articles/camera-settings.html).
Then, the next step it's [Capturing High Quality Point
Clouds](https://support.zivid.com/latest/academy/camera/capturing-high-quality-point-clouds.html)

#### Single Acquisition

We can create settings for a single acquisition capture.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L19))

``` sourceCode cpp
const auto settings = Zivid::Settings(Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} });
```

#### Multi Acquisition HDR

We may also create settings to be used in a multi-acquisition HDR
capture.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureHDR/CaptureHDR.cpp#L21-L29))

``` sourceCode cpp
Zivid::Settings settings;
for(const auto aperture : { 11.31, 5.66, 2.83 })
{
	std::cout << "Adding acquisition with aperture = " << aperture << std::endl;
	const auto acquisitionSettings = Zivid::Settings::Acquisition{
		Zivid::Settings::Acquisition::Aperture{ aperture },
	};
	settings.acquisitions().emplaceBack(acquisitionSettings);
}
```

Fully configured settings are demonstrated below.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureHDRCompleteSettings/CaptureHDRCompleteSettings.cpp#L68-L132))

``` sourceCode cpp
std::cout << "Configuring settings for capture:" << std::endl;
Zivid::Settings settings{
	Zivid::Settings::Engine::phase,
	Zivid::Settings::Sampling::Color::rgb,
	Zivid::Settings::Sampling::Pixel::blueSubsample2x2,
	Zivid::Settings::RegionOfInterest::Box::Enabled::yes,
	Zivid::Settings::RegionOfInterest::Box::PointO{ 1000, 1000, 1000 },
	Zivid::Settings::RegionOfInterest::Box::PointA{ 1000, -1000, 1000 },
	Zivid::Settings::RegionOfInterest::Box::PointB{ -1000, 1000, 1000 },
	Zivid::Settings::RegionOfInterest::Box::Extents{ -1000, 1000 },
	Zivid::Settings::RegionOfInterest::Depth::Enabled::yes,
	Zivid::Settings::RegionOfInterest::Depth::Range{ 200, 2000 },
	Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
	Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{ 1.5 },
	Zivid::Settings::Processing::Filters::Noise::Removal::Enabled::yes,
	Zivid::Settings::Processing::Filters::Noise::Removal::Threshold{ 7.0 },
	Zivid::Settings::Processing::Filters::Noise::Suppression::Enabled::yes,
	Zivid::Settings::Processing::Filters::Noise::Repair::Enabled::yes,
	Zivid::Settings::Processing::Filters::Outlier::Removal::Enabled::yes,
	Zivid::Settings::Processing::Filters::Outlier::Removal::Threshold{ 5.0 },
	Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes,
	Zivid::Settings::Processing::Filters::Reflection::Removal::Mode::global,
	Zivid::Settings::Processing::Filters::Cluster::Removal::Enabled::yes,
	Zivid::Settings::Processing::Filters::Cluster::Removal::MaxNeighborDistance{ 10 },
	Zivid::Settings::Processing::Filters::Cluster::Removal::MinArea{ 100 },
	Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Enabled::yes,
	Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Strength{ 0.4 },
	Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Enabled::no,
	Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Threshold{ 0.5 },
	Zivid::Settings::Processing::Filters::Hole::Repair::Enabled::yes,
	Zivid::Settings::Processing::Filters::Hole::Repair::HoleSize{ 0.2 },
	Zivid::Settings::Processing::Filters::Hole::Repair::Strictness{ 1 },
	Zivid::Settings::Processing::Resampling::Mode::upsample2x2,
	Zivid::Settings::Processing::Color::Balance::Red{ 1.0 },
	Zivid::Settings::Processing::Color::Balance::Green{ 1.0 },
	Zivid::Settings::Processing::Color::Balance::Blue{ 1.0 },
	Zivid::Settings::Processing::Color::Gamma{ 1.0 },
	Zivid::Settings::Processing::Color::Experimental::Mode::automatic
};
std::cout << settings << std::endl;
std::cout << "Configuring base acquisition with settings same for all HDR acquisition:" << std::endl;
const auto baseAcquisition = Zivid::Settings::Acquisition{};
std::cout << baseAcquisition << std::endl;

std::cout << "Configuring acquisition settings different for all HDR acquisitions" << std::endl;
auto exposureValues = getExposureValues(camera);
const std::vector<double> aperture = std::get<0>(exposureValues);
const std::vector<double> gain = std::get<1>(exposureValues);
const std::vector<std::chrono::microseconds> exposureTime = std::get<2>(exposureValues);
const std::vector<double> brightness = std::get<3>(exposureValues);
for(size_t i = 0; i < aperture.size(); ++i)
{
	std::cout << "Acquisition " << i + 1 << ":" << std::endl;
	std::cout << "  Exposure Time: " << exposureTime.at(i).count() << std::endl;
	std::cout << "  Aperture: " << aperture.at(i) << std::endl;
	std::cout << "  Gain: " << gain.at(i) << std::endl;
	std::cout << "  Brightness: " << brightness.at(i) << std::endl;
	const auto acquisitionSettings = baseAcquisition.copyWith(
		Zivid::Settings::Acquisition::Aperture{ aperture.at(i) },
		Zivid::Settings::Acquisition::Gain{ gain.at(i) },
		Zivid::Settings::Acquisition::ExposureTime{ exposureTime.at(i) },
		Zivid::Settings::Acquisition::Brightness{ brightness.at(i) });
	settings.acquisitions().emplaceBack(acquisitionSettings);
}
```

#### 2D Settings

It is possible to only capture a 2D image. This is faster than a 3D
capture. 2D settings are configured as follows.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture2D/Capture2D.cpp#L21-L29))

``` sourceCode cpp
const auto settings2D =
	Zivid::Settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{
						Zivid::Settings2D::Acquisition::ExposureTime{ std::chrono::microseconds{ 30000 } },
						Zivid::Settings2D::Acquisition::Aperture{ 11.31 },
						Zivid::Settings2D::Acquisition::Brightness{ 1.80 },
						Zivid::Settings2D::Acquisition::Gain{ 2.0 } } },
					Zivid::Settings2D::Processing::Color::Balance::Red{ 1 },
					Zivid::Settings2D::Processing::Color::Balance::Green{ 1 },
					Zivid::Settings2D::Processing::Color::Balance::Blue{ 1 } };
```

### Load

Zivid Studio can store the current settings to .yml files. These can be
read and applied in the API. You may find it easier to modify the
settings in these (human-readable) yaml-files in your preferred editor.
Check out
[Presets](https://support.zivid.com/latest/reference-articles/presets-settings.html)
for recommended .yml files tuned for your application.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureHDRCompleteSettings/CaptureHDRCompleteSettings.cpp#L144-L149))

``` sourceCode cpp
const auto settingsFile = "Settings.yml";
std::cout << "Loading settings from file: " << settingsFile << std::endl;
const auto settingsFromFile = Zivid::Settings(settingsFile);
```

### Save

You can also save settings to .yml file.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureHDRCompleteSettings/CaptureHDRCompleteSettings.cpp#L144-L146))

``` sourceCode cpp
const auto settingsFile = "Settings.yml";
std::cout << "Saving settings to file: " << settingsFile << std::endl;
settings.save(settingsFile);
```

-----

Caution\!:

> Zivid settings files must use .yml file extension ( not .yaml).

## Capture

Now we can capture a 3D image. Whether there is a single acquisition or
multiple acquisitions (HDR) is given by the number of `acquisitions` in
`settings`.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L22))

``` sourceCode cpp
const auto frame = camera.capture(settings);
```

The `Zivid::Frame` contains the point cloud and color image (stored on
compute device memory) and the capture and camera information.

### Load

Once saved, the frame can be loaded from a ZDF file.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Basic/FileFormats/ReadIterateZDF/ReadIterateZDF.cpp#L17-L19))

``` sourceCode cpp
const auto dataFile = std::string(ZIVID_SAMPLE_DATA_DIR) + "/Zivid3D.zdf";
std::cout << "Reading ZDF frame from file: " << dataFile << std::endl;
const auto frame = Zivid::Frame(dataFile);
```

Saving to a ZDF file is addressed later in the tutorial.

### Capture 2D

If we only want to capture a 2D image, which is faster than 3D, we can
do so via the 2D API.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture2D/Capture2D.cpp#L32))

``` sourceCode cpp
const auto frame2D = camera.capture(settings2D);
```

## Save

We can now save our results.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L24-L26))

``` sourceCode cpp
const auto dataFile = "Frame.zdf";
frame.save(dataFile);
```

-----

Tip:

> You can open and view `Frame.zdf` file in [Zivid
> Studio](https://support.zivid.com/latest//getting-started/studio-guide.html).

### Export

The API detects which format to use. See [Point
Cloud](https://support.zivid.com/latest//reference-articles/point-cloud-structure-and-output-formats.html)
for a list of supported formats. For example, we can export the point
cloud to .ply format.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L28-L30))

``` sourceCode cpp
const auto dataFilePLY = "PointCloud.ply";
frame.save(dataFilePLY);
```

### Save 2D

We can get 2D color image from a 3D capture.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/ROIBoxViaArucoMarker/ROIBoxViaArucoMarker.cpp#L191))

``` sourceCode cpp
const auto image = pointCloud.copyImageRGBA();
```

2D captures also produce 2D color images.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture2D/Capture2D.cpp#L35))

``` sourceCode cpp
const auto image = frame2D.imageRGBA();
```

Then, we can save the 2D image.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture2D/Capture2D.cpp#L44-L46))

``` sourceCode cpp
const auto imageFile = "Image.png";
std::cout << "Saving 2D color image to file: " << imageFile << std::endl;
image.save(imageFile);
```

## Multithreading

Operations on camera objects are thread-safe, but other operations like
listing cameras and connecting to cameras should be executed in
sequence. Find out more in
[CaptureTutorial](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Basic/CaptureTutorial.md).

## Conclusion

This tutorial shows how to use the Zivid SDK to connect to, configure,
capture, and save from the Zivid camera.
