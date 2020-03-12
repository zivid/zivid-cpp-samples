## Introduction

This tutorial describes how to use Zivid SDK to capture point clouds and 2D images.

1. [Initialize](#initialize)
2. [Connect](#connect)
   1. [Specific Camera](#connect---specific-camera)
   2. [Virtual Camera](#connect---virtual-camera)
3. [Configure](#configure)
   1. [Capture Assistant](#capture-assistant)
   2. [Manual Configuration](#manual-configuration)
      1. [Single](#single-frame)
      2. [HDR](#hdr-frame)
      3. [From File](#from-file)
      4. [2D](#2d-settings)
4. [Capture](#capture)
    1. [HDR](#capture-hdr)
    2. [2D](#capture-2d)
5. [Save](#save)

### Prerequisites

You should have installed Zivid SDK and C++ samples. For more details see [Instructions][installation-instructions-url].

## Initialize

Before calling any of the APIs in the Zivid SDK, we have to start up the Zivid Application. This is done through a simple instantiation of the application ([go to source][start_app-url]).
```cpp
Zivid::Application zivid;
```

## Connect

Now we can connect to the camera ([go to source][connect-url]).
```cpp
auto camera = zivid.connectCamera();
```

### Connect - Specific Camera

Sometime multiple cameras are connected to the same computer. It might then be necessary to work with a specific camera in the code. This can be done by providing the serial number of the wanted camera.
```cpp
auto camera = zivid.connectCamera(Zivid::SerialNumber{ "2020C0DE" });
```

---
**Note** 

The serial number of your camera is shown in the Zivid Studio.

---

You may also list all cameras connected to the computer, and view their serial numbers through
```cpp
auto cameras = zivid.cameras();
for(auto cam : cameras)
{
    std::cout << "Connecting camera: " << cam.serialNumber() << std::endl;
}
```

### Connect - Virtual Camera

You may want to experiment with the SDK, without access to a physical camera. Minor changes are required to keep the sample working ([go to source][filecamera-url]).
```cpp
auto zdfFile = Zivid::Environment::dataPath() + "/MiscObjects.zdf";
auto camera = zivid.createFileCamera(zdfFile);
```

---
**Note**

The quality of the point cloud you get from *MiscObjects.zdf* is not representative of the Zivid One+.

---

## Configure

As with all cameras there are settings that can be configured. These may be set manually, or you use our Capture Assistant.

### Capture Assistant

It can be difficult to know what settings to configure. Luckily we have the Capture Assistant. This is available in the Zivid SDK to help configure camera settings ([go to source][captureassistant-url]).
```cpp
Zivid::CaptureAssistant::SuggestSettingsParameters suggestSettingsParameters(
    std::chrono::milliseconds{ 1200 }, Zivid::CaptureAssistant::AmbientLightFrequency::none);

std::cout << "Running Capture Assistant with parameters: " << suggestSettingsParameters << std::endl;
const auto settingsVector{ Zivid::CaptureAssistant::suggestSettings(camera, suggestSettingsParameters) };
```

These settings can be used in an [HDR capture](#capture-hdr), which we will discuss later.

As opposed to manual configuration of settings, there are only two parameters to consider with Capture Assistant.

1. **Maximum Capture Time** in number of milliseconds.
    1. Minimum capture time is 200ms. This allows only one frame to be captured.
    2. The algorithm will combine multiple frames if the budget allows.
    3. The algorithm will attempt to cover as much of the dynamic range in the scene as possible.
    4. A maximum capture time of more than 1 second will get good coverage in most scenarios.
2. **Ambient light compensation**
    1. May restrict capture assistant to exposure periods that are multiples of the ambient light period.
    2. 60Hz is found in (amongst others) Japan, Americas, Taiwan, South Korea and Philippines.
    3. 50Hz is found in most rest of the world.

### Manual configuration

We may choose to configure settings manually ([go to source][settings-url]). For more information about what each settings does, please see [Zivid One+ Camera Settings][kb-camera_settings-url].

#### Single Frame

We can configure settings for an individual frame directly to the camera.
```cpp
camera << Zivid::Settings::Iris{ 20 }
       << Zivid::Settings::ExposureTime{ std::chrono::microseconds{ 8333 } }
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
```

#### HDR Frame

We may also set a list of settings to be used in an [HDR capture](#capture-hdr).
```cpp
std::vector<Zivid::Settings> settingsVector;
for(const size_t iris : { 20U, 25U, 30U })
{
    std::cout << "Add settings for frame with iris = " << iris << std::endl;
    auto setting = Zivid::Settings::Settings();
    setting.set(Zivid::Settings::Iris{ iris });
    settingsVector.emplace_back(setting);
}
```

### From File

Zivid Studio can store the current settings to .yml files. These can be read and applied in the API. You may find it easier to modify the settings in these (human-readable) yaml-files in your preferred editor.
```cpp
camera.setSettings(Zivid::Settings("frame_01.yml"));
```
You may also apply settings from file while connecting to the camera.
```cpp
auto camera = zivid.connectCamera(Zivid::Settings("frame_01.yml"));
```

### 2D Settings

It is possible to only capture a 2D image. This is faster than a 3D capture, and can be used . 2D settings are configured as follows ([go to source][settings2d-url]).
```cpp
auto settings = Zivid::Settings2D();
settings.set(Zivid::Settings2D::ExposureTime{ std::chrono::microseconds{ 10000 } });
settings.set(Zivid::Settings2D::Gain{ 1.0 });
settings.set(Zivid::Settings2D::Iris{ 35 });
settings.set(Zivid::Settings2D::Brightness{ 1.0 });
```

## Capture

Now we can capture a frame. The default capture is a single 3D point cloud ([go to source][capture-url]).
```cpp
auto frame = camera.capture();
```

### Capture HDR

As was revealed in the [Capture Assistant](#capture-assistant) section, a capture may consist of multiple frames. In order to capture multiple frames, and combine them, we can do as follows ([go to source][captureHDR-url])
```cpp
const auto hdrFrame{ Zivid::HDR::capture(camera, settingsVector) };
```
It is possible to [manually create](#hdr-frame) the `settingsVector`, if not set via [Capture Assistant](#capture-assistant).

### Capture 2D

If we only want to capture a 2D image, which is faster than 3D, we can do so via the 2D API ([go to source][capture2d-url]).
```cpp
auto frame = camera.capture2D(settings);
```

## Save

We can now save our results ([go to source][save-url]).
```cpp
const auto resultFile = "result.zdf";
frame.save(resultFile);
```
The API detects which format to use. See [Point Cloud][kb-point_cloud-url] for a list of supported formats.

## Conclusion

This tutorial shows how to use the Zivid SDK to connect to, configure and capture from the Zivid camera.

[//]: ### "Recommended further reading"

[installation-instructions-url]: ../../../README.md#instructions
[start_app-url]: Capture/Capture.cpp#L10
[connect-url]: Capture/Capture.cpp#L15
[captureassistant-url]: CaptureAssistant/CaptureAssistant.cpp#L18-L22
[settings-url]: Capture/Capture.cpp#L18-L19
[kb-camera_settings-url]: https://zivid.atlassian.net/wiki/spaces/ZividKB/pages/99713044/Zivid+One+Camera+Settings
[capture-url]: Capture/Capture.cpp#L22
[capture2d-url]: Capture2D/Capture2D.cpp#L22
[settings2d-url]: Capture2D/Capture2D.cpp#L16-L19
[captureHDR-url]: CaptureAssistant/CaptureAssistant.cpp#L31
[save-url]: Capture/Capture.cpp#L25
[kb-point_cloud-url]: https://zivid.atlassian.net/wiki/spaces/ZividKB/pages/427396/Point+Cloud
[filecamera-url]: CaptureFromFile/CaptureFromFile.cpp#L13-L17
