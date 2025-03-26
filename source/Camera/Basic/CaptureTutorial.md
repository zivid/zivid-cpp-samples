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
[**Capture**](#Capture-2D3D) |
[**Save**](#Save) |
[**Multithreading**](#Multithreading) |
[**Conclusion**](#Conclusion)

---



## Introduction

This tutorial describes how to use the Zivid SDK to capture point clouds
and 2D images.

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
	userInput ? fileCameraPath : std::string(ZIVID_SAMPLE_DATA_DIR) + "/FileCameraZivid2PlusMR60.zfc";
```

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureFromFileCamera/CaptureFromFileCamera.cpp#L40))

``` sourceCode cpp
auto camera = zivid.createFileCamera(fileCamera);
```

The acquisition settings should be initialized like shown below, but you
are free to alter the processing settings.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureFromFileCamera/CaptureFromFileCamera.cpp#L43-L55))

``` sourceCode cpp
Zivid::Settings settings{
	Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} },
	Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
	Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{ 1.5 },
	Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes,
	Zivid::Settings::Processing::Filters::Reflection::Removal::Mode::global,
};
Zivid::Settings2D settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} },
							Zivid::Settings2D::Processing::Color::Balance::Red{ 1 },
							Zivid::Settings2D::Processing::Color::Balance::Green{ 1 },
							Zivid::Settings2D::Processing::Color::Balance::Blue{ 1 } };
settings.color() = Zivid::Settings::Color{ settings2D };
```

You can read more about the file camera option in [File
Camera](https://support.zivid.com/latest/academy/camera/file-camera.html).

## Configure

As with all cameras there are settings that can be configured.

### Presets

The recommendation is to use
[Presets](https://support.zivid.com/latest/reference-articles/presets-settings.html)
available in Zivid Studio and as .yml files (see below). Presets are
designed to work well for most cases right away, making them a great
starting point. If needed, you can easily fine-tune the settings for
better results. You can edit the YAML files in any text editor or code
the settings manually.

### Load

You can export camera settings to .yml files from Zivid Studio. These
can be loaded and applied in the API.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureHDRCompleteSettings/CaptureHDRCompleteSettings.cpp#L218-L223))

``` sourceCode cpp
const auto settingsFile = "Settings.yml";
std::cout << "Loading settings from file: " << settingsFile << std::endl;
const auto settingsFromFile = Zivid::Settings(settingsFile);
```

### Save

You can also save settings to .yml file.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureHDRCompleteSettings/CaptureHDRCompleteSettings.cpp#L218-L220))

``` sourceCode cpp
const auto settingsFile = "Settings.yml";
std::cout << "Saving settings to file: " << settingsFile << std::endl;
settings.save(settingsFile);
```

### Manual configuration

Another option is to configure settings manually. For more information
about what each settings does, please see [Camera
Settings](https://support.zivid.com/latest/reference-articles/camera-settings.html).
Then, the next step it's [Capturing High Quality Point
Clouds](https://support.zivid.com/latest/academy/camera/capturing-high-quality-point-clouds.html)

#### Single 2D and 3D Acquisition - Default settings

We can create settings for a single acquisition capture.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L19-L22))

``` sourceCode cpp
const auto settings =
	Zivid::Settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} },
					Zivid::Settings::Color{ Zivid::Settings2D{
						Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } } } };
```

#### Multi Acquisition HDR

We may also create settings to be used in a multi-acquisition HDR
capture.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Advanced/CaptureHDRPrintNormals/CaptureHDRPrintNormals.cpp#L39-L47))

``` sourceCode cpp
Zivid::Settings settings;
for(const auto aperture : { 5.66, 4.00, 2.59 })
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
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureHDRCompleteSettings/CaptureHDRCompleteSettings.cpp#L112-L206))

``` sourceCode cpp
std::cout << "Configuring settings for capture:" << std::endl;
Zivid::Settings2D settings2D{
	Zivid::Settings2D::Sampling::Color::rgb,
	Zivid::Settings2D::Sampling::Pixel::all,
Zivid::Settings2D::Processing::Color::Balance::Blue{ 1.0 },
Zivid::Settings2D::Processing::Color::Balance::Green{ 1.0 },
Zivid::Settings2D::Processing::Color::Balance::Red{ 1.0 },
Zivid::Settings2D::Processing::Color::Gamma{ 1.0 },

Zivid::Settings2D::Processing::Color::Experimental::Mode::automatic,
```

> };
> 
>   - Zivid::Settings settings{  
>     Zivid::Settings::Color{ settings2D },
>     
>     Zivid::Settings::Engine::phase,
>     
>     Zivid::Settings::RegionOfInterest::Box::Enabled::yes,
>     Zivid::Settings::RegionOfInterest::Box::PointO{ 1000, 1000, 1000
>     }, Zivid::Settings::RegionOfInterest::Box::PointA{ 1000, -1000,
>     1000 }, Zivid::Settings::RegionOfInterest::Box::PointB{ -1000,
>     1000, 1000 }, Zivid::Settings::RegionOfInterest::Box::Extents{
>     -1000, 1000 },
>     
>     Zivid::Settings::RegionOfInterest::Depth::Enabled::yes,
>     Zivid::Settings::RegionOfInterest::Depth::Range{ 200, 2000 },
>     
>     Zivid::Settings::Processing::Filters::Cluster::Removal::Enabled::yes,
>     Zivid::Settings::Processing::Filters::Cluster::Removal::MaxNeighborDistance{
>     10 },
>     Zivid::Settings::Processing::Filters::Cluster::Removal::MinArea{
>     100 },
>     
>     Zivid::Settings::Processing::Filters::Hole::Repair::Enabled::yes,
>     Zivid::Settings::Processing::Filters::Hole::Repair::HoleSize{ 0.2
>     }, Zivid::Settings::Processing::Filters::Hole::Repair::Strictness{
>     1 },
>     
>     Zivid::Settings::Processing::Filters::Noise::Removal::Enabled::yes,
>     Zivid::Settings::Processing::Filters::Noise::Removal::Threshold{
>     7.0 },
>     
>     Zivid::Settings::Processing::Filters::Noise::Suppression::Enabled::yes,
>     Zivid::Settings::Processing::Filters::Noise::Repair::Enabled::yes,
>     
>     Zivid::Settings::Processing::Filters::Outlier::Removal::Enabled::yes,
>     Zivid::Settings::Processing::Filters::Outlier::Removal::Threshold{
>     5.0 },
>     
>     Zivid::Settings::Processing::Filters::Reflection::Removal::Enabled::yes,
>     Zivid::Settings::Processing::Filters::Reflection::Removal::Mode::global,
>     
>     Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Enabled::yes,
>     Zivid::Settings::Processing::Filters::Smoothing::Gaussian::Sigma{
>     1.5 },
>     
>     Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Enabled::yes,
>     Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Correction::Strength{
>     0.4 },
>     
>     Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Enabled::no,
>     Zivid::Settings::Processing::Filters::Experimental::ContrastDistortion::Removal::Threshold{
>     0.5 },
>     
>     Zivid::Settings::Processing::Resampling::Mode::upsample2x2,
>     
>     Zivid::Settings::Diagnostics::Enabled::no,
> 
> };
> 
> setSamplingPixel(settings, camera); std::cout \<\< settings \<\<
> std::endl; std::cout \<\< "Configuring base acquisition with settings
> same for all HDR acquisition:" \<\< std::endl; const auto
> baseAcquisition = Zivid::Settings::Acquisition{}; std::cout \<\<
> baseAcquisition \<\< std::endl; const auto baseAquisition2D =
> Zivid::Settings2D::Acquisition{};
> 
> std::cout \<\< "Configuring acquisition settings different for all HDR
> acquisitions" \<\< std::endl; auto exposureValues =
> getExposureValues(camera); const std::vector\<double\> aperture =
> std::get\<0\>(exposureValues); const std::vector\<double\> gain =
> std::get\<1\>(exposureValues); const
> std::vector\<std::chrono::microseconds\> exposureTime =
> std::get\<2\>(exposureValues); const std::vector\<double\> brightness
> = std::get\<3\>(exposureValues); for(size\_t i = 0; i \<
> aperture.size(); ++i) { std::cout \<\< "Acquisition " \<\< i + 1 \<\<
> ":" \<\< std::endl; std::cout \<\< " Exposure Time: " \<\<
> exposureTime.at(i).count() \<\< std::endl; std::cout \<\< " Aperture:
> " \<\< aperture.at(i) \<\< std::endl; std::cout \<\< " Gain: " \<\<
> gain.at(i) \<\< std::endl; std::cout \<\< " Brightness: " \<\<
> brightness.at(i) \<\< std::endl; const auto acquisitionSettings =
> baseAcquisition.copyWith( Zivid::Settings::Acquisition::Aperture{
> aperture.at(i) }, Zivid::Settings::Acquisition::Gain{ gain.at(i) },
> Zivid::Settings::Acquisition::ExposureTime{ exposureTime.at(i) },
> Zivid::Settings::Acquisition::Brightness{ brightness.at(i) });
> settings.acquisitions().emplaceBack(acquisitionSettings); } const auto
> acquisitionSettings2D = baseAquisition2D.copyWith(
> Zivid::Settings2D::Acquisition::Aperture{ 2.83 },
> Zivid::Settings2D::Acquisition::ExposureTime{ microseconds{ 10000 } },
> Zivid::Settings2D::Acquisition::Brightness{ 1.8 },
> Zivid::Settings2D::Acquisition::Gain{ 1.0 });
> settings.color().value().acquisitions().emplaceBack(acquisitionSettings2D);

## Capture 2D3D

Now we can capture a 2D and 3D image (point cloud with color). Whether
there is a single acquisition or multiple acquisitions (HDR) is given by
the number of `acquisitions` in `settings`.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L25))

``` sourceCode cpp
const auto frame = camera.capture2D3D(settings);
```

The `Zivid::Frame` contains the point cloud, the color image, the
capture, and the camera information (all of which are stored on the
compute device memory).

### Capture 3D

If we only want to capture 3D, the points cloud without color, we can do
so via the `capture3D` API.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureWithSettingsFromYML/CaptureWithSettingsFromYML.cpp#L72))

``` sourceCode cpp
const auto frame3D = camera.capture3D(settings);
```

### Capture 2D

If we only want to capture a 2D image, which is faster than 3D, we can
do so via the `capture2D` API.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureWithSettingsFromYML/CaptureWithSettingsFromYML.cpp#L52))

``` sourceCode cpp
const auto frame2D = camera.capture2D(settings);
```

## Save

We can now save our results.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L32-L34))

``` sourceCode cpp
const auto dataFile = "Frame.zdf";
frame.save(dataFile);
```

-----

Tip:

> You can open and view `Frame.zdf` file in [Zivid
> Studio](https://support.zivid.com/latest//getting-started/studio-guide.html).

### Export

In the next code example, the point cloud is exported to the .ply
format. For other exporting options, see [Point
Cloud](https://support.zivid.com/latest//reference-articles/point-cloud-structure-and-output-formats.html)
for a list of supported formats.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L36-L38))

``` sourceCode cpp
const auto dataFilePLY = "PointCloud.ply";
frame.save(dataFilePLY);
```

### Load

Once saved, the frame can be loaded from a ZDF file.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Basic/FileFormats/ReadIterateZDF/ReadIterateZDF.cpp#L17-L19))

``` sourceCode cpp
const auto dataFile = std::string(ZIVID_SAMPLE_DATA_DIR) + "/Zivid3D.zdf";
std::cout << "Reading ZDF frame from file: " << dataFile << std::endl;
const auto frame = Zivid::Frame(dataFile);
```

### Save 2D

We can get the 2D color image from `Frame2D`, which is part of the
`Frame` object, obtained from `capture2D3D()`.

([go to source]())

``` sourceCode cpp
const auto image2D = frame.frame2D().value().imageBGRA();
```

We can get 2D color image directly from the point cloud. This image will
have the same resolution as the point cloud.

([go to source]())

``` sourceCode cpp
const auto pointCloud = frame.pointCloud();
const auto image2DInPointCloudResolution = pointCloud.copyImageRGBA();
```

2D captures also produce 2D color images in linear RGB and sRGB color
space.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L27))

``` sourceCode cpp
const auto imageRGBA = frame.frame2D().value().imageRGBA();
.. tab-item:: sRGB
```

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureWithSettingsFromYML/CaptureWithSettingsFromYML.cpp#L54))

``` sourceCode cpp
const auto imageSRGB = frame2D.imageSRGB();
```

Then, we can save the 2D image in linear RGB or sRGB color space.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L28-L30))

``` sourceCode cpp
const auto imageFile = "ImageRGB.png";
std::cout << "Saving 2D color image (linear RGB color space) to file: " << imageFile << std::endl;
imageRGBA.save(imageFile);
.. tab-item:: sRGB
```

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureWithSettingsFromYML/CaptureWithSettingsFromYML.cpp#L55-L57))

``` sourceCode cpp
const auto imageFile = "ImageSRGB.png";
std::cout << "Saving 2D color image (sRGB color space) to file: " << imageFile << std::endl;
imageSRGB.save(imageFile);
```

## Multithreading

Operations on camera objects are thread-safe, but other operations like
listing cameras and connecting to cameras should be executed in
sequence. Find out more in
[CaptureTutorial](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Basic/CaptureTutorial.md).

## Conclusion

This tutorial shows how to use the Zivid SDK to connect to, configure,
capture, and save from the Zivid camera.
