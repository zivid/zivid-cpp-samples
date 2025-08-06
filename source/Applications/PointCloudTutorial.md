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
[**Voxel**](#Voxel-downsample) |
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

When you capture with Zivid, you get a frame in return. The point cloud
is stored in the frame, and the frame is stored in the GPU memory. The
capture can contain color or not, depending of the method that you call.
For more information see this `table with different capture
modes<capture-mode-table>`.

#### Capture with color

If you want to capture a point cloud with color, you can use the
`Zivid::Camera::capture2D3D()` method.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp#L25))

``` sourceCode cpp
const auto frame = camera.capture2D3D(settings);
```

#### Capture without color

If you want to capture a point cloud without color, you can use the
`Zivid::Camera::capture3D()` method.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureWithSettingsFromYML/CaptureWithSettingsFromYML.cpp#L116))

``` sourceCode cpp
const auto frame3D = camera.capture3D(settings);
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

`Zivid::Camera::capture2D3D()` and `Zivid::Camera::capture3D()` methods
return at some moment in time after the camera completes capturing raw
images. The handle from `Zivid::Frame::pointCloud()` is available
instantly. However, the actual point cloud data becomes available only
after the processing on the GPU is finished. Any calls to data-copy
functions (section below) will block and wait for processing to finish
before proceeding with the requested copy operation.

For detailed explanation, see [Point Cloud Capture
Process](https://support.zivid.com/latest/academy/camera/point-cloud-capture-process.html).

-----

### Unorganized point cloud

It is possible to convert the organized point cloud to an unorganized
point cloud. While doing so, all NaN values are removed, and the point
cloud is flattened to a 1D array.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/MultiCamera/StitchByTransformation/StitchByTransformation.cpp#L181))

``` sourceCode cpp
const auto unorganizedPointCloud = frame.pointCloud().toUnorganizedPointCloud();
```

#### Combining multiple unorganized point clouds

The unorganized point cloud can be extended with additional unorganized
point clouds.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/MultiCamera/StitchByTransformationFromZDF/StitchByTransformationFromZDF.cpp#L46))

``` sourceCode cpp
stitchedPointCloud.extend(currentPointCloud.transform(transformationMatrixZivid));
```

### Copy from GPU to CPU memory

You can now selectively copy data based on what is required. This is the
complete list of output data formats and how to copy them from the GPU.
Most of these APIs also applies to the unorganized point cloud.

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
const auto data = pointCloud.copyData<Zivid::PointXYZColorRGBA_SRGB>();
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
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Advanced/AllocateMemoryForPointCloudData/AllocateMemoryForPointCloudData.cpp#L73-L95))

``` sourceCode cpp
std::cout << "Capturing frame" << std::endl;
frame = camera.capture2D3D(settings);
std::cout << "Copying colors with Zivid API from GPU to CPU" << std::endl;
auto colors = frame.frame2D().value().imageBGRA_SRGB();

std::cout << "Casting the data pointer as a void*, since this is what the OpenCV matrix constructor requires."
		<< std::endl;
auto *dataPtrZividAllocated = const_cast<void *>(static_cast<const void *>(colors.data()));

std::cout << "Wrapping this block of data in an OpenCV matrix. This is possible since the layout of \n"
		<< "Zivid::ColorBGRA_SRGB exactly matches the layout of CV_8UC4. No copying occurs in this step."
		<< std::endl;
const cv::Mat bgraZividAllocated(colors.height(), colors.width(), CV_8UC4, dataPtrZividAllocated);

std::cout << "Displaying image" << std::endl;
cv::imshow("BGRA image Zivid Allocated", bgraZividAllocated);
cv::waitKey(CI_WAITKEY_TIMEOUT_IN_MS);
  .. rubric:: Copy selected data from GPU to CPU memory (user-allocated)

  In the above example, ownership of the data was held by the returned :code:`Zivid::Array2D<>` objects.
  Alternatively, you may provide a pre-allocated memory buffer to :code:`Zivid::PointCloud::copyData(dataPtr)`.
  The type of :code:`dataPtr` defines what shall be copied (:code:`PointXYZ`, :code:`ColorRGBA`, etc.).

  Now let us look at the exact same use case as above.
  However, this time, we allow OpenCV to allocate the necessary storage.
  Then we ask the Zivid API to copy data directly from the GPU into this memory location.
```

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Advanced/AllocateMemoryForPointCloudData/AllocateMemoryForPointCloudData.cpp#L55-L69))

``` sourceCode cpp
std::cout << "Allocating the necessary storage with OpenCV API based on resolution info before any capturing"
		<< std::endl;
auto bgraUserAllocated = cv::Mat(resolution.height(), resolution.width(), CV_8UC4);
std::cout << "Capturing frame" << std::endl;
auto frame = camera.capture2D3D(settings);
auto pointCloud = frame.pointCloud();

std::cout << "Copying data with Zivid API from the GPU into the memory location allocated by OpenCV"
		<< std::endl;
pointCloud.copyData(&(*bgraUserAllocated.begin<Zivid::ColorBGRA_SRGB>()));

std::cout << "Displaying image" << std::endl;
cv::imshow("BGRA image User Allocated", bgraUserAllocated);
cv::waitKey(CI_WAITKEY_TIMEOUT_IN_MS);
  .. rubric:: Copy unorganized point cloud data from GPU to CPU memory (Open3D-tensor)
```

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/MultiCamera/StitchByTransformation/StitchByTransformation.cpp#L92-L116))

``` sourceCode cpp
open3d::t::geometry::PointCloud copyToOpen3D(const Zivid::UnorganizedPointCloud &pointCloud)
{
	using namespace open3d::core;
	auto device = Device("CPU:0");
	auto xyzTensor = Tensor({ static_cast<int64_t>(pointCloud.size()), 3 }, Dtype::Float32, device);
	auto rgbTensor = Tensor({ static_cast<int64_t>(pointCloud.size()), 3 }, Dtype::Float32, device);
pointCloud.copyData(reinterpret_cast<Zivid::PointXYZ *>(xyzTensor.GetDataPtr<float>()));

// Open3D does not store colors in 8-bit
const auto rgbaColors = pointCloud.copyColorsRGBA_SRGB();
for(size_t i = 0; i < pointCloud.size(); ++i)
{
	const auto r = static_cast<float>(rgbaColors(i).r) / 255.0f;
	const auto g = static_cast<float>(rgbaColors(i).g) / 255.0f;
	const auto b = static_cast<float>(rgbaColors(i).b) / 255.0f;
	rgbTensor.SetItem(TensorKey::Index(i), Tensor::Init({ r, g, b }));
}

open3d::t::geometry::PointCloud cloud(device);
cloud.SetPointPositions(xyzTensor);
cloud.SetPointColors(rgbTensor);
return cloud;
```

> }

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

Transformation can be done in-place:

  - `Zivid::PointCloud::transform()`
  - `Zivid::UnorganizedPointCloud::transform()`

or by creating a new instance:

  - `Zivid::PointCloud::transformed()`
  - `Zivid::UnorganizedPointCloud::transformed()`

The following example shows how create a new instance of
`Zivid::UnorganizedPointCloud` with a transformation applied to it. Note
that in this sample is is not necessary to create a new instance, as the
untransformed point cloud is not used after the transformation.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/MultiCamera/StitchByTransformation/StitchByTransformation.cpp#L183))

``` sourceCode cpp
const auto transformedUnorganizedPointCloud = unorganizedPointCloud.transformed(transformationMatrix);
```

Even the in-place API returns the transformed point cloud, so you can
use it directly, as in the example below.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/MultiCamera/StitchByTransformationFromZDF/StitchByTransformationFromZDF.cpp#L46))

``` sourceCode cpp
stitchedPointCloud.extend(currentPointCloud.transform(transformationMatrixZivid));
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

-----

Note:

`Zivid::UnorganizedPointCloud` does not support downsampling, but it
does support voxel downsampling, see `voxel_downsample`.

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

## Voxel downsample

`Zivid::UnorganizedPointCloud` supports voxel downsampling. The API
takes two arguments:

1.  `voxelSize` - the size of the voxel in millimeters.
2.  `minPointsPerVoxel` - the minimum number of points per voxel to keep
    it.

Voxel downsampling subdivides 3D space into a grid of cubic voxels with
a given size. If a given voxel contains a number of points at or above
the given limit, all those source points are replaced with a single
point with the following properties:

  - Position (XYZ) is an SNR-weighted average of the source points'
    positions, i.e. a high-confidence source point will have a greater
    influence on the resulting position than a low-confidence one.
  - Color (RGBA) is the average of the source points' colors.
  - Signal-to-noise ratio (SNR) is sqrt(sum(SNR^2)) of the source
    points' SNR values, i.e. the SNR of a new point will increase with
    both the number and the confidence of the source points that were
    used to compute its position.

Using minPointsPerVoxel \> 1 is particularly useful for removing noise
and artifacts from unorganized point clouds that are a combination of
point clouds captured from different angles. This is because a given
artifact is most likely only present in one of the captures, and
minPointsPerVoxel can be used to only fill voxels that both captures
"agree" on.

([go to
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/MultiCamera/StitchByTransformation/StitchByTransformation.cpp#L188))

``` sourceCode cpp
const auto finalPointCloud = stitchedPointCloud.voxelDownsampled(0.5, 1);
```

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
source](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Basic/Visualization/CaptureVis3D/CaptureVis3D.cpp#L29-L38))

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

| SDK    | Changes                                                                                                                                                           |
| ------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 2.16.0 | Added support for `Zivid::UnorganizedPointCloud`. `transformed` is added as a function to `Zivid::PointCloud` (also available in `Zivid::UnorganizedPointCloud`). |
| 2.11.0 | Added support for SRGB color space.                                                                                                                               |
| 2.10.0 | [:orphan:](https://support.zivid.com/latest/academy/camera/monochrome-capture.html) introduces a faster alternative to `downsample_point_cloud_tutorial`.         |
