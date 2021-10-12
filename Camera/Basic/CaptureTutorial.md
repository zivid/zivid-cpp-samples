# Capture Tutorial

- [Introduction](#introduction)
- [Initialize](#initialize)
- [Connect](#connect)
  - [Connect - Specific Camera](#connect---specific-camera)
  - [Connect - File Camera](#connect---file-camera)
- [Configure](#configure)
  - [Capture Assistant](#capture-assistant)
  - [Manual configuration](#manual-configuration)
  - [From File](#from-file)
- [Capture](#capture)
  - [Capture2D](#capture2d)
- [Save](#save)
  - [Save 2D](#save-2d)
- [Conclusion](#conclusion)


## Introduction

This tutorial describes how to use the Zivid SDK to capture point clouds
and 2D images.

For MATLAB see [Zivid Capture Tutorial for
MATLAB](https://github.com/zivid/zivid-matlab-samples/blob/master/source/Camera/Basic/CaptureTutorial.md).

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

[go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master/source/source/Camera/Basic/Capture/Capture.cpp#L14)

``` sourceCode cpp
Zivid::Application zivid;
```

## Connect

Now we can connect to the camera.

[go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master/source/source/Camera/Basic/Capture/Capture.cpp#L17)

``` sourceCode cpp
auto camera = zivid.connectCamera();
```

### Connect - Specific Camera

Sometimes multiple cameras are connected to the same computer, but it
might be necessary to work with a specific camera in the code. This can
be done by providing the serial number of the wanted camera.

``` sourceCode cpp
auto camera = zivid.connectCamera(Zivid::CameraInfo::SerialNumber{ "2020C0DE" });
```

-----

Note:

The serial number of your camera is shown in the Zivid Studio.

-----

You may also list all cameras connected to the computer, and view their
serial numbers through

[go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/InfoUtilOther/PrintVersionInfo/PrintVersionInfo.cpp#L14)

``` sourceCode cpp
auto cameras = zivid.cameras();
std::cout << "Found " << cameras.size() << " cameras" << std::endl;
for(auto &camera : cameras)
{
	std::cout << "Camera Info: " << camera.info() << std::endl;
}
```

### Connect - File Camera

You may want to experiment with the SDK, without access to a physical
camera. Minor changes are required to keep the sample working.

[go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master/source/source/Camera/Basic/CaptureFromFileCamera/CaptureFromFileCamera.cpp#L18)

``` sourceCode cpp
const auto fileCamera = std::string(ZIVID_SAMPLE_DATA_DIR) + "/FileCameraZividOne.zfc";
auto camera = zivid.createFileCamera(fileCamera);
```

-----

Note:

The quality of the point cloud you get from FileCameraZividOne.zfc is
not representative of the Zivid 3D cameras.

-----

## Configure

As with all cameras there are settings that can be configured. These may
be set manually, or you use our Capture Assistant.

### Capture Assistant

It can be difficult to know what settings to configure. Luckily we have
the Capture Assistant. This is available in the Zivid SDK to help
configure camera settings.

[go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master/source/source/Camera/Basic/CaptureAssistant/CaptureAssistant.cpp#L19)

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
about what each settings does, please see
[camera-settings](https://support.zivid.com/latest/rst/reference-articles/camera-settings.html).
Note that Zivid Two has a set of [standard
settings](https://support.zivid.com/latest//reference-articles/standard-acquisition-settings-zivid-two.html).

#### Single Acquisition

We can create settings for a single capture.

[go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master/source/source/Camera/Basic/Capture/Capture.cpp#L20)

``` sourceCode cpp
const auto settings =
	Zivid::Settings{ Zivid::Settings::Experimental::Engine::phase,
					Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{
						Zivid::Settings::Acquisition::Aperture{ 5.66 },
						Zivid::Settings::Acquisition::ExposureTime{ std::chrono::microseconds{ 6500 } } } },
					Zivid::Settings::Processing::Filters::Outlier::Removal::Enabled::yes,
					Zivid::Settings::Processing::Filters::Outlier::Removal::Threshold{ 5.0 } };
```

#### Multi Acquisition HDR

We may also create settings to be used in an HDR capture.

[go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master/source/source/Camera/Basic/CaptureHDR/CaptureHDR.cpp#L20)

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

#### 2D Settings

It is possible to only capture a 2D image. This is faster than a 3D
capture. 2D settings are configured as follows.

[go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master/source/source/Camera/Basic/Capture2D/Capture2D.cpp#L21)

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

### From File

Zivid Studio can store the current settings to `*.yml` files. These can
be read and applied in the API. You may find it easier to modify the
settings in these (human-readable) yaml-files in your preferred editor.

``` sourceCode cpp
const auto settings = Zivid::Settings("Settings.yml");
```

## Capture

Now we can capture a 3D image. Whether there is a single acquisition or
multiple acquisitions (HDR) is given by the number of `acquisitions` in
`settings`.

[go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master/source/source/Camera/Basic/Capture/Capture.cpp#L29)

``` sourceCode cpp
const auto frame = camera.capture(settings);
```

### Capture2D

If we only want to capture a 2D image, which is faster than 3D, we can
do so via the 2D API.

[go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master/source/source/Camera/Basic/Capture2D/Capture2D.cpp#L32)

``` sourceCode cpp
const auto frame2D = camera.capture(settings2D);
```

## Save

We can now save our results.

[go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master/source/source/Camera/Basic/Capture/Capture.cpp#L31)

``` sourceCode cpp
const auto *dataFile = "Frame.zdf";
frame.save(dataFile);
```

The API detects which format to use. See [Point
Cloud](https://support.zivid.com/latest//reference-articles/point-cloud-structure-and-output-formats.html)
for a list of supported formats.

-----

Tip:

You can open and view `Frame.zdf` file in [Zivid
Studio](https://support.zivid.com/latest//getting-started/studio-guide.html).

-----

### Save 2D

If we capture a 2D image, we can save it.

[go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master/source/source/Camera/Basic/Capture2D/Capture2D.cpp#L35)

``` sourceCode cpp
const auto image = frame2D.imageRGBA();
const auto *imageFile = "Image.png";
std::cout << "Saving image to file: " << imageFile << std::endl;
image.save(imageFile);
```

## Conclusion

This tutorial shows how to use the Zivid SDK to connect to, configure,
capture, and save from the Zivid camera.
