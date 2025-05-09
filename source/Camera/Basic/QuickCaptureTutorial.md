# Quick Capture Tutorial

Note\! This tutorial has been generated for use on Github. For original
tutorial see:
[QuickCaptureTutorial](https://support.zivid.com/latest/getting-started/quick-capture-tutorial.html)



---

*Contents:*
[**Introduction**](#Introduction) |
[**Initialize**](#Initialize) |
[**Connect**](#Connect) |
[**Configure**](#Configure) |
[**Capture**](#Capture) |
[**Save**](#Save) |
[**Utilize**](#Utilize)

---



## Introduction

This tutorial describes the most basic way to use the Zivid SDK to
capture point clouds.

**Prerequisites**

  - Install [Zivid
    Software](https://support.zivid.com/latest//getting-started/software-installation.html).
  - For Python: install
    [zivid-python](https://github.com/zivid/zivid-python#installation)

## Initialize

Calling any of the APIs in the Zivid SDK requires initializing the Zivid
application and keeping it alive while the program runs.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L13))

``` sourceCode cpp
Zivid::Application zivid;
```

## Connect

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L16))

``` sourceCode cpp
auto camera = zivid.connectCamera();
```

## Configure

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureWithSettingsFromYML/CaptureWithSettingsFromYML.cpp#L76))

``` sourceCode cpp
const auto settings = Zivid::Settings(settingsPath);
```

## Capture

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L25))

``` sourceCode cpp
const auto frame = camera.capture2D3D(settings);
```

## Save

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L32-L34))

``` sourceCode cpp
const auto dataFile = "Frame.zdf";
frame.save(dataFile);
.. tab-item:: Export
```

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L36-L38))

``` sourceCode cpp
const auto dataFilePLY = "PointCloud.ply";
frame.save(dataFilePLY);
```

For other exporting options, see [Point
Cloud](https://support.zivid.com/latest//reference-articles/point-cloud-structure-and-output-formats.html)
for a list of supported formats

## Utilize

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Basic/FileFormats/ReadIterateZDF/ReadIterateZDF.cpp#L22-L23))

``` sourceCode cpp
const auto pointCloud = frame.pointCloud();
const auto data = pointCloud.copyData<Zivid::PointXYZColorRGBA_SRGB>();
```

-----

Tip:

1.  You can export Preset settings to YML from [Zivid
    Studio](https://support.zivid.com/latest//getting-started/studio-guide.html)

\#. You can open and view `Frame.zdf` file in [Zivid
Studio](https://support.zivid.com/latest//getting-started/studio-guide.html).
.. rubric:: Conclusion

This tutorial shows the most basic way to use the Zivid SDK to connect
to, capture, and save from the Zivid camera.

For a more in-depth tutorial check out the complete
[CaptureTutorial](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Basic/CaptureTutorial.md).
