# Point Cloud Tutorial

Note\! This tutorial has been generated for use on Github. For original
tutorial see:
[PointCloudTutorial](https://support.zivid.com/latest/academy/applications/point-cloud-tutorial.html)



---

*Contents:*
[**Introduction**](#Introduction) |
[**Frame**](#Frame) |
[**Point**](#Point-Cloud) |
[**Transform**](#Transform) |
[**Downsample**](#Downsample) |
[**Normals**](#Normals) |
[**Visualize**](#Visualize) |
[**Conclusion**](#Conclusion) |
[**Version**](#Version-History)

---



## Introduction

This tutorial describes how to use Zivid SDK to work with [Point
Cloud](https://support.zivid.com/latest//reference-articles/point-cloud-structure-and-output-formats.html)
data.

-----

Tip:

> If you prefer watching a video, our webinar [Getting your point cloud
> ready for your
> application](https://www.zivid.com/webinars-page?wchannelid=ffpqbqc7sg&wmediaid=h66zph71vo)
> covers the Point Cloud Tutorial.

**Prerequisites**

  - Install [Zivid
    Software](https://support.zivid.com/latest//getting-started/software-installation.html).
  - For Python: install
    [zivid-python](https://github.com/zivid/zivid-python#installation)

## Frame

The `Zivid::Frame` contains the point cloud and color image (stored on
compute device memory) and the capture and camera information.

### Capture

When you capture with Zivid, you get a frame in return.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L25))

``` sourceCode cpp
const auto frame = camera.capture2D3D(settings);
```

Check
[CaptureTutorial](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Basic/CaptureTutorial.md)
for detailed instructions on how to capture.

### Load

The frame can also be loaded from a ZDF file.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Basic/FileFormats/ReadIterateZDF/ReadIterateZDF.cpp#L17-L19))

``` sourceCode cpp
const auto dataFile = std::string(ZIVID_SAMPLE_DATA_DIR) + "/Zivid3D.zdf";
std::cout << "Reading ZDF frame from file: " << dataFile << std::endl;
const auto frame = Zivid::Frame(dataFile);
```

## Point Cloud

### Get handle from Frame

You can now get a handle to the point cloud data on the GPU.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Basic/FileFormats/ReadIterateZDF/ReadIterateZDF.cpp#L22))

``` sourceCode cpp
const auto pointCloud = frame.pointCloud();
```

Point cloud contains XYZ, RGB, and SNR, laid out on a 2D grid.

For more info check out [Point Cloud
Structure](https://support.zivid.com/latest//reference-articles/point-cloud-structure-and-output-formats.html).

The method `Zivid::Frame::pointCloud()` does not perform any copying
from GPU memory.

-----

Note:

`Zivid::Camera::capture()` method returns at some moment in time after
the camera completes capturing raw images. The handle from
`Zivid::Frame::pointCloud()` is available instantly. However, the actual
point cloud data becomes available only after the processing on the GPU
is finished. Any calls to data-copy functions (section below) will block
and wait for processing to finish before proceeding with the requested
copy operation.

For detailed explanation, see [Point Cloud Capture
Process](https://support.zivid.com/latest/academy/camera/point-cloud-capture-process.html).

-----

### Copy from GPU to CPU memory

You can now selectively copy data based on what is required. This is the
complete list of output data formats and how to copy them from the GPU.

| Return type                                | Copy functions                                                                         | Data per pixel | Total data |
| ------------------------------------------ | -------------------------------------------------------------------------------------- | -------------- | ---------- |
| `Zivid::Array2D<Zivid::PointXYZ>`          | `PointCloud::copyPointsXYZ()` or `PointCloud::copyData<Zivid::PointXYZ>()`             | 12 bytes       | 28 MB      |
| `Zivid::Array2D<Zivid::PointXYZW>`         | `PointCloud::copyPointsXYZW()` or `PointCloud::copyData<Zivid::PointXYZW>()`           | 16 bytes       | 37 MB      |
| `Zivid::Array2D<Zivid::PointZ>`            | `PointCloud::copyPointsZ()` or `PointCloud::copyData<Zivid::PointZ>()`                 | 4 bytes        | 9 MB       |
| `Zivid::Array2D<Zivid::ColorRGBA>`         | `PointCloud::copyColorsRGBA()` or `PointCloud::copyData<Zivid::ColorRGBA>()`           | 4 bytes        | 9 MB       |
| `Zivid::Array2D<Zivid::SNR>`               | `PointCloud::copySNRs()` or `PointCloud::copyData<Zivid::SNR>()`                       | 4 bytes        | 9 MB       |
| `Zivid::Array2D<Zivid::PointXYZColorRGBA>` | `PointCloud::copyData<PointXYZColorRGBA>()`                                            | 16 bytes       | 37 MB      |
| `Zivid::Array2D<Zivid::PointXYZColorBGRA>` | `PointCloud::copyPointsXYZColorsBGRA()` or `PointCloud::copyData<PointXYZColorBGRA>()` | 16 bytes       | 37 MB      |
| `Zivid::Image<Zivid::ColorRGBA>`           | `PointCloud::copyImageRGBA()`                                                          | 4 bytes        | 9 MB       |
| `Zivid::Image<Zivid::ColorBGRA>`           | `PointCloud::copyImageBGRA()`                                                          | 4 bytes        | 9 MB       |
| `Zivid::Image<Zivid::ColorsRGB>`           | `PointCloud::copyImagesRGB()`                                                          | 4 bytes        | 9 MB       |

Here is an example of how to copy data.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Basic/FileFormats/ReadIterateZDF/ReadIterateZDF.cpp#L23))

``` sourceCode cpp
const auto data = pointCloud.copyData<Zivid::PointXYZColorRGBA>();
```

#### Memory allocation options

In terms of memory allocation, there are two ways to copy data:

  - The Zivid SDK can allocate a memory buffer and copy data to it.
  - A user can pass a pointer to a pre-allocated memory buffer, and the
    Zivid SDK will copy the data to the pre-allocated memory buffer.

We present examples for the two memory allocation options using OpenCV.

**Copy selected data from GPU to CPU memory (Zivid-allocated)**

If you are only concerned about e.g. RGB color data of the point cloud,
you can copy only that data to the CPU memory.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Advanced/AllocateMemoryForPointCloudData/AllocateMemoryForPointCloudData.cpp#L70-L93))

``` sourceCode cpp
std::cout << "Capturing frame" << std::endl;
frame = camera.capture(settings);
pointCloud = frame.pointCloud();
std::cout << "Copying colors with Zivid API from GPU to CPU" << std::endl;
auto colors = pointCloud.copyColorsBGRA();

std::cout << "Casting the data pointer as a void*, since this is what the OpenCV matrix constructor requires."
		<< std::endl;
auto *dataPtrZividAllocated = const_cast<void *>(static_cast<const void *>(colors.data()));

std::cout << "Wrapping this block of data in an OpenCV matrix. This is possible since the layout of \n"
		<< "Zivid::ColorBGRA exactly matches the layout of CV_8UC4. No copying occurs in this step."
		<< std::endl;
const cv::Mat bgraZividAllocated(colors.height(), colors.width(), CV_8UC4, dataPtrZividAllocated);

std::cout << "Displaying image" << std::endl;
cv::imshow("BGRA image Zivid Allocated", bgraZividAllocated);
cv::waitKey(0);
  .. rubric:: Copy selected data from GPU to CPU memory (user-allocated)

  In the above example, ownership of the data was held by the returned :code:`Zivid::Array2D<>` objects.
  Alternatively, you may provide a pre-allocated memory buffer to :code:`Zivid::PointCloud::copyData(dataPtr)`.
  The type of :code:`dataPtr` defines what shall be copied (:code:`PointXYZ`, :code:`ColorRGBA`, etc.).

  Now let us look at the exact same use case as above.
  However, this time, we allow OpenCV to allocate the necessary storage.
  Then we ask the Zivid API to copy data directly from the GPU into this memory location.
```

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Advanced/AllocateMemoryForPointCloudData/AllocateMemoryForPointCloudData.cpp#L52-L66))

``` sourceCode cpp
std::cout << "Allocating the necessary storage with OpenCV API based on resolution info before any capturing"
		<< std::endl;
auto bgraUserAllocated = cv::Mat(resolution.height(), resolution.width(), CV_8UC4);
std::cout << "Capturing frame" << std::endl;
auto frame = camera.capture(settings);
auto pointCloud = frame.pointCloud();

std::cout << "Copying data with Zivid API from the GPU into the memory location allocated by OpenCV"
		<< std::endl;
pointCloud.copyData(&(*bgraUserAllocated.begin<Zivid::ColorBGRA>()));

std::cout << "Displaying image" << std::endl;
cv::imshow("BGRA image User Allocated", bgraUserAllocated);
cv::waitKey(0);
```

## Transform

You may want to
[transform](https://support.zivid.com/latest//academy/applications/transform.html)
the point cloud to change its origin from the camera to the robot base
frame or, e.g., [scale the point cloud by transforming it from mm to
m](https://support.zivid.com/latest//academy/applications/transform/transform-millimeters-to-meters.html).

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/HandEyeCalibration/UtilizeHandEyeCalibration/UtilizeHandEyeCalibration.cpp#L236))

``` sourceCode cpp
pointCloud.transform(baseToCameraTransform);
```

## Downsample

Sometimes you might not need a point cloud with as `high spatial
resolution (High spatial resolution means more detail and less distance
between points)` as given from the camera. You may then
[downsample](https://support.zivid.com/latest//academy/applications/downsampling.html)
the point cloud.

-----

Note:

> [Sampling
> (3D)](https://support.zivid.com/latest/reference-articles/settings/sampling.html)
> describes a hardware-based sub-/downsample method that reduces the
> resolution of the point cloud during capture while also reducing the
> acquisition and capture time.

-----

Downsampling can be done in-place, which modifies the current point
cloud.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/Downsample/Downsample.cpp#L57))

``` sourceCode cpp
pointCloud.downsample(Zivid::PointCloud::Downsampling::by2x2);
```

It is also possible to get the downsampled point cloud as a new point
cloud instance, which does not alter the existing point cloud.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/Downsample/Downsample.cpp#L50))

``` sourceCode cpp
auto downsampledPointCloud = pointCloud.downsampled(Zivid::PointCloud::Downsampling::by2x2);
```

Zivid SDK supports the following downsampling rates: `by2x2`, `by3x3`,
and `by4x4`, with the possibility to perform downsampling multiple
times.

## Normals

Some applications require computing
[normals](https://support.zivid.com/latest//academy/applications/normals.html)
from the point cloud.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Advanced/CaptureHDRPrintNormals/CaptureHDRPrintNormals.cpp#L53-L54))

``` sourceCode cpp
std::cout << "Computing normals and copying them to CPU memory" << std::endl;
const auto normals = pointCloud.copyData<Zivid::NormalXYZ>();
```

The Normals API computes the normal at each point in the point cloud and
copies normals from the GPU memory to the CPU memory. The result is a
matrix of normal vectors, one for each point in the input point cloud.
The size of normals is equal to the size of the input point cloud.

## Visualize

Having the frame allows you to visualize the point cloud.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Basic/Visualization/CaptureVis3D/CaptureVis3D.cpp#L26-L35))

``` sourceCode cpp
std::cout << "Setting up visualization" << std::endl;
Zivid::Visualization::Visualizer visualizer;
std::cout << "Visualizing point cloud" << std::endl;
visualizer.showMaximized();
visualizer.show(frame);
visualizer.resetToFit();

std::cout << "Running visualizer. Blocking until window closes." << std::endl;
visualizer.run();
```

You can visualize the point cloud from the point cloud object as well.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/Downsample/Downsample.cpp#L16-L42))

``` sourceCode cpp
std::cout << "Getting point cloud from frame" << std::endl;
auto pointCloud = frame.pointCloud();
std::cout << "Setting up visualization" << std::endl;
Zivid::Visualization::Visualizer visualizer;

std::cout << "Visualizing point cloud" << std::endl;
visualizer.showMaximized();
visualizer.show(pointCloud);
visualizer.resetToFit();

std::cout << "Running visualizer. Blocking until window closes." << std::endl;
visualizer.run();
```

For more information, check out [Visualization
Tutorial](https://support.zivid.com/latest/academy/applications/visualization-tutorial.html),
where we cover point cloud, color image, depth map, and normals
visualization, with implementations using third party libraries.

## Conclusion

This tutorial shows how to use the Zivid SDK to extract the point cloud,
manipulate it, transform it, and visualize it.

## Version History

| SDK    | Changes                                                                                                                                                   |
| ------ | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 2.11.0 | Added support for SRGB color space.                                                                                                                       |
| 2.10.0 | [:orphan:](https://support.zivid.com/latest/academy/camera/monochrome-capture.html) introduces a faster alternative to `downsample_point_cloud_tutorial`. |
