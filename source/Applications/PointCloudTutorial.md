
## Introduction

This tutorial describes how to use Zivid SDK to work with [Point Cloud][kb-point_cloud-url] data.

1. [Frame](#frame)
   1. [Capture](#capture)
   2. [Load](#load)
   3. [Visualize](#visualize)
3.  [Point Cloud](#point-cloud)
    1. [Get handle](#get-handle)
    2. [Copy](#copy)
    3.  [Transform](#transform)

### Prerequisites

You should have installed Zivid SDK and cloned C++ samples. For more details see [Instructions][installation-instructions-url].

## Frame

### Capture

The ```Zivid::Frame``` can be captured with a camera ([go to source][frame-capture]).
```cpp
const auto frame = camera.capture(settings);
```
Check [Capture Tutorial][capture-tutorial] for more details.

### Load
It can also be loaded from a ZDF file ([go to source][frame-from-file]).
```cpp
const Zivid::Frame frame = Zivid::Frame("/path/to/Zivid3D.zdf");
```
### Visualize

Having the frame allows you to visualize the point cloud ([go to source][visualize-point-cloud]).

```cpp
Zivid::Visualization::Visualizer visualizer;

visualizer.showMaximized();
visualizer.show(frame);
visualizer.resetToFit();

visualizer.run();
```

## Point Cloud

### Get handle

You can now get a handle to the point cloud data on the GPU ([go to source][point-cloud]).
```cpp
const auto pointCloud = frame.pointCloud();
```
The method ```Zivid::Frame::pointCloud()``` does not perform any copying from GPU memory.

Note: ```Zivid::Camera::capture()``` method returns as soon as the camera is done capturing. At that point the handle from ```Zivid::Frame::pointCloud()``` is available instantly as well. However, the actual point cloud data becomes available only after the GPU is finished processing. This processing will occur on the GPU in the background, and any calls to data-copy functions (see section [Copy](#copy) below) will wait for this to finish before proceeding with the requested copy operation.

### Copy

You can now selectively copy data based on what is required. Data can even be copied directly into your own pre-allocated memory. This is the complete list of output data formats and how to copy them from the GPU.


|Return type|Functions for copying from GPU|Data per pixel|Total data copied|
|-|-|-|-|
|```Zivid::Array2D<Zivid::PointXYZ>```| ```PointCloud::copyPointsXYZ()``` or ```PointCloud::copyData<Zivid::PointXYZ>()```| 12 bytes |28 MB |
|```Zivid::Array2D<Zivid::PointXYZW>```| ```PointCloud::copyPointsXYZW()``` or ```PointCloud::copyData<Zivid::PointXYZW>()```| 16 bytes |37 MB |
|```Zivid::Array2D<Zivid::PointZ>```| ```PointCloud::copyPointsZ()``` or ```PointCloud::copyData<Zivid::PointZ>()```| 4 bytes |9 MB |
|```Zivid::Array2D<Zivid::ColorRGBA>```| ```PointCloud::copyColorsRGBA()``` or ```PointCloud::copyData<Zivid::ColorRGBA>()```| 4 bytes |9 MB |
|```Zivid::Image<Zivid::ColorRGBA>```| ```PointCloud::copyImageRGBA()```| 4 bytes |9 MB |
|```Zivid::Array2D<Zivid::PointXYZColorRGBA>```| ```PointCloud::copyPointsXYZColorsRGBA()``` or ```PointCloud::copyData<PointXYZColorRGBA>()```| 16 bytes |37 MB |
|```Zivid::Array2D<Zivid::PointXYZColorBGRA>```| ```PointCloud::copyPointsXYZColorsBGRA()``` or ```PointCloud::copyData<PointXYZColorBGRA>()```| 16 bytes |37 MB |
|```Zivid::Array2D<Zivid::SNR>```| ```PointCloud::copySNRs()``` or ```PointCloud::copyData<Zivid::SNR>()```| 4 bytes |9 MB |

#### Copy selected data from GPU to system memory (Zivid-allocated)

If you are only concerned about e.g. RGB color data of the point cloud, you can copy only that data to the system memory.
```cpp
auto rgba = frame.pointCloud().copyColorsRGBA(); /* Colors are copied from the GPU and into a
Zivid::Array2D<Zivid::ColorRGBA>, which takes ownership of the data. */

auto *dataPtr = const_cast<void *>(static_cast<const void *>(rgba.data())); /* Cast the data pointer as
a void*, since this is what the OpenCV matrix constructor requires. */

cv::Mat rgba(rgba.height(), rgba.width(), CV_8UC4, dataPtr); /* We wrap this block of data in an OpenCV
matrix. This is possible since the layout of Zivid::ColorRGBA exactly matches the layout of CV_8UC4.
No copying occurs in this step. */
```

####  Copy selected data from GPU to system memory (user-allocated)

In the above example, ownership of the data was held by the returned ```Zivid::Array2D<>``` objects. Alternatively, you may provide a pre-allocated memory buffer to ```Zivid::PointCloud::copyData(dataPtr)```; the type of ```dataPtr``` defines what shall be copied (```PointXYZ```, ```ColorRGBA```, etc.).

Now let us look at the exact same use case as above. However, this time, we allow OpenCV to allocate the necessary storage. Then we ask the Zivid API to copy data directly from the GPU into this memory location.

```cpp
const auto pointCloud = frame.pointCloud(); /* Get a handle to the full point cloud on the GPU. */

auto rgba = cv::Mat(pointCloud.height(), pointCloud.width(), CV_8UC4); /* Allocate an OpenCV matrix with
an appropriate size. */

auto *dataPtr = reinterpret_cast<Zivid::ColorRGBA *>(rgba.data); /* Cast the OpenCV data pointer to
ColorRGBA* so the Zivid API can understand which data to copy. */

pointCloud.copyData(dataPtr); /* Copy RGBA data straight from GPU memory into the OpenCV memory buffer. */
```

## Transform

You may want to change the point cloud's origin from the camera to the robot base frame or scale the point cloud to e.g. change it from millimeters to meters. This can be done by transforming the point cloud using ```Zivid::PointCloud::transform(Zivid::Matrix4x4)``` ([go to source][transform]).

```cpp
pointCloud.transform(transformationMatrix);
```

## Conclusion

This tutorial shows how to use the Zivid SDK to extract the point cloud, manipulate it, transform it, and visualize it.

[//]: ### "Recommended further reading"

[installation-instructions-url]: ../../README.md#instructions
[frame-from-file]:Basic/FileFormats/ReadIterateZDF/ReadIterateZDF.cpp#L15-L17
[frame-capture]:../Camera/Basic/Capture/Capture.cpp#L28
[capture-tutorial]:../Camera/Basic/CaptureTutorial.md#L158
[point-cloud]:Advanced/Downsample/Downsample.cpp#L181
[transform]:Advanced/MultiCamera/StitchByTransformationFromZDF/StitchByTransformationFromZDF.cpp#L148
[visualize-point-cloud]:Basic/Visualization/CaptureVis3D/CaptureVis3D.cpp#L26-L35
[kb-point_cloud-url]: https://zivid.atlassian.net/wiki/spaces/ZividKB/pages/520061383
