# Quick Capture Tutorial



---

*Contents:*
1. [Introduction](#Introduction)
2. [Initialize](#Initialize)
3. [Connect](#connect)
4. [Configure](#configure)
5. [Capture](#capture)
6. [Save](#save)

---
## Introduction

This tutorial describes the most basic way to use the Zivid SDK to
capture point clouds.

For MATLAB see [Zivid Quick Capture Tutorial for
MATLAB](https://github.com/zivid/zivid-matlab-samples/blob/master/source/Camera/Basic/QuickCaptureTutorial.md)

**Prerequisites**

  - Install [Zivid
    Software](https://support.zivid.com/latest//getting-started/software-installation.html).
  - For Python: install
    [zivid-python](https://github.com/zivid/zivid-python#installation)

## Initialize

Calling any of the APIs in the Zivid SDK requires initializing the Zivid
application and keeping it alive while the program runs.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L14))

``` sourceCode cpp
Zivid::Application zivid;
```

## Connect

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L18))

``` sourceCode cpp
auto camera = zivid.connectCamera();
```

## Configure

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

## Capture

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L32))

``` sourceCode cpp
const auto frame = camera.capture(settings);
```

## Save

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L35-L38))

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

**Conclusion**

This tutorial shows the most basic way to use the Zivid SDK to connect
to, capture, and save from the Zivid camera.

For a more in-depth tutorial check out the complete
[CaptureTutorial](https://github.com/zivid/zivid-cpp-samples/tree/master/Camera/Basic/CaptureTutorial).
