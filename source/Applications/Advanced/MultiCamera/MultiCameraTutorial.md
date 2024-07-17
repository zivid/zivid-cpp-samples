## Introduction

This tutorial describes how to use Zivid SDK to calibrate multiple cameras against each other. The result of this calibration is a transformation matrix from each camera to a primary camera. While the primary camera is, by default, the first camera connected to, it can easily be controlled via its Serial Number. The transformation matrices can be used to transform one point cloud into the coordinate frame of another. This is a very good first step in stitching point clouds together.

1. [Calibrate](#calibrate)
    1. [Connect to cameras](#connect-to-cameras)
    2. [Capture calibration object](#capture-calibration-object)
    3. [Detect checkerboard feature points](#detect-checkerboard-feature-points)
    4. [Perform Multi-Camera Calibration](#perform-multi-camera-calibration)
    5. [Save transformation matrices](#save-transformation-matrices-to-yaml)
2. [Stitch](#stitch)
    1. [Load associated transformation matrices and map to point cloud](#load-associated-transformation-matrices-and-map-to-point-cloud)
    2. [Apply transformation matrix and stitch transformed point cloud with previous](#apply-transformation-matrix-and-stitch-transformed-point-cloud-with-previous)
        1. [Transform](#transform)
        2. [Combine data in PCL point cloud](#combine-data-in-PCL-point-cloud)
            1. [Initialize point cloud memory](#initialize-point-cloud-memory)
            2. [Stitch and add color](#stitch-and-add-color)
    3. [Visualize stitched point cloud](#visualize-stitched-point-cloud)
    4. [Save stitched point cloud](#save-stitched-point-cloud)

### Prerequisites

You should have installed Zivid SDK and C++ samples. For more details see [Instructions][installation-instructions-url]. This tutorial will not go into details about the basics of the SDK, such as initialization and capture. Please see [Capture Tutorial][capture_tutorial-url] for that.

## Calibrate

In this section, we will connect to multiple cameras, capture images of [the calibration object][calibration_object-url], calculate transformation matrices, and save them to a YAML file. There are two samples:

1. [MultiCameraCalibration][multi_camera_calibration_sample-url] - Connects to cameras and captures from each
2. [MultiCameraCalibrationFromZDF][multi_camera_calibration_sample_from_files-url] - Loads existing point clouds from ZDF files

### Connect to cameras

In this tutorial, we will connect to all available cameras. We can do so via ([go to source][connect_all_cameras-url])

```cpp
auto cameras = zivid.cameras();
```

### Capture calibration object

We are now ready to capture the calibration object. We assume that all cameras will get good captures of the calibration object with Capture Assistant. You may use Zivid Studio to quickly verify that the calibration object is in view. The detection API ([Zivid::Calibration::detectCalibrationBoard][detect_calibration_board-url]) will notify the user if the quality is not good enough for calibration.

Capture in the sample is performed with Capture Assistant, this was covered in the [Capture Tutorial][capture_tutorial_capture_assistant-url]. The assisted capture has been wrapped in the function `assistedCapture` ([go to source][capture_with_ca-url]).

```cpp
const auto frame = assistedCapture(camera);
```

When we load the point cloud from the file, we simply replace this line of code with:

```cpp
const auto frame = Zivid::Frame(fileName);
```

### Detect checkerboard feature points

The calibration object we use in this tutorial is a checkerboard. Before we can run calibration, we must detect feature points from the checkerboard from all cameras ([go to source][detect_calibration_board-url]).

```cpp
const auto detectionResult = Zivid::Calibration::detectCalibrationBoard(frame);
```

We may, at this point, verify that the capture had good enough quality. `detectionResult` is of a type that can be tested directly. It overloads the bool operator to provide this information. When it passes the quality test, we save the detection result and the serial number of the camera used ([go to source][verify_checkerboard_capture_quality-url]).

```cpp
if(detectionResult)
{
    detectionResults.push_back(detectionResult);
    serialNumbers.push_back(serial);
}
else
{
    throw std::runtime_error(
        "Could not detect checkerboard. Please ensure it is visible from all cameras.");
}
```

### Perform Multi-Camera Calibration

Now that we have detected all feature points in all captures from all cameras, we can perform the multi-camera calibration ([go to source][calibrate-url]).

```cpp
const auto results = Zivid::Calibration::calibrateMultiCamera(detectionResults);
```

The returned `results` can be checked directly as to whether or not calibration was successful ([go to source][check_calibration-url]). Again, the type overloads the bool operator.

```cpp
if(results)
{
    std::cout << "Multi-camera calibration OK." << std::endl;
...
}
else
{
    std::cout << "Multi-camera calibration FAILED." << std::endl;
}
```

`results` contains two vectors ([go to source][extract_from_results-url]):

1. `transforms` - contains all transformation matrices
2. `residuals` - contains an indication of the calibration error

```cpp
const auto &transforms = results.transforms();
const auto &residuals = results.residuals();
```

### Save transformation matrices to YAML

Later we will use the results, so we store the transformation in YAML files. To do this, we use our API. It is important to keep track of which transformation matrix belongs to which camera. More precisely, the pose of the camera during calibration. Thus, we use the camera's serial number as an identifier, and we use it on the file name. ([go to source][saveTransformationMatricesToYAML-url]).

```cpp
transforms[i].save(transformationMatricesSavePath + "\\" + serialNumbers[i] + ".yaml");
```

## Stitch

Now that we have our transformation matrices, we can easily combine point clouds or "stitch" them together. With the transformation matrix, we can transform all points from one camera into the coordinate frame of another camera.

In this section, we will:

1. Load associated transformation matrices and map to point cloud
2. Apply transformation matrix and stitch transformed point cloud with previous
3. Visualize stitched point cloud
4. Save to PLY

Again there are two samples:

1. [StitchByTransformation][stitch_by_transformation-url] - Connects to cameras and captures from each
2. [StitchByTransformationFromZDF][stitch_by_transformation_from_files-url] - Loads existing point clouds from ZDF files

### Load associated transformation matrices and map to point cloud

To apply the correct transformation matrix, we must map it to its corresponding frame. When we capture directly from the camera, we use its serial number, and when we load from the file, we use the serial number from the ZDF frame. When we use the serial number of a camera or the serial number from the ZDF file, we expect to find an exact match on the YAML file name.
The way we map transformation matrix to point cloud is quite similar to capturing directly from the camera and when loaded from a file. The main difference is when capturing with a camera, we access the serial number and search for a YAML file with that name. Then we map a transformation matrix with camera.([go to source][stitch_by_transformation_map_camera-url]).

```cpp
const auto transformsMappedToCameras =
    getTransformationMatricesFromYAML(transformationMatricesfileList, cameras);
```

On the other hand, when loading from ZDF files, we need to find a ZDF file from the list of files and extract his serial number from the frame. And then, we search for a YAML file on the same file list that uses that serial number as its name. Then we map a transformation matrix with a Frame ([go to source][stitch_by_transformation_map_zdf-url]).

```cpp
const auto transformsMappedToFrames =
    getTransformationMatricesAndFramesFromZDF(transformationMatricesAndZdfFileList);
```

If we now capture from the cameras in the order they are stored in `transformsMappedToCameras`, the order of `frames` will be correct ([go to source][stitch_by_transformation_capture-url]).

```cpp
// Capture from all cameras
auto frames = std::vector<Zivid::Frame>();
for(const auto &transformAndCamera : transformsMappedToCameras)
{
    std::cout << "Imaging from camera: " << transformAndCamera.mCamera.serialNumber() << std::endl;
    const auto frame = assistedCapture(transformAndCamera.mCamera);
    frames.push_back(frame);
}
```

### Apply transformation matrix and stitch transformed point cloud with previous

We have mapped capture with a transformation matrix. Thus, we are ready to transform and stitch.

#### Transform

The Zivid SDK supports transformation before data is copied from GPU.

Then we transform, with capture directly from camera ([go to source][stitch_by_transform_transformation-url]):

```cpp
auto pointCloud = frames.at(i).pointCloud();
pointCloud.transform(transformsMappedToCameras.at(i).mTransformationMatrix);
```

or with point cloud loaded from ZDF file ([go to source][stitch_by_transform_transformation_from_files-url]):

```cpp
auto pointCloud = transformsMappedToFrames.at(i).mFrame.pointCloud();
pointCloud.transform(transformsMappedToFrames.at(i).mTransformationMatrix);
```

#### Combine data in PCL point cloud

We use PCL to create the new stitched point cloud. This is because we want to visualize the point cloud and save the point cloud to PLY.

##### Initialize point cloud memory

First, we will initialize the new point cloud and allocate memory. To retain color, we will allocate memory for both XYZ and RGB (type `pcl::PointXYZRGB`). We count the maximum number of points (`maxNumberOfPoints`) while we capture or load the frames.

When captured directly from camera ([go to source][stitch_by_transformation_count-url])

```cpp
auto frames = std::vector<Zivid::Frame>();
auto maxNumberOfPoints = 0;
for(const auto &transformAndCamera : transformsMappedToCameras)
{
    std::cout << "Imaging from camera: " << transformAndCamera.mCamera.serialNumber() << std::endl;
    const auto frame = assistedCapture(transformAndCamera.mCamera);
    maxNumberOfPoints += frame.pointCloud().width() * frame.pointCloud().height();
    frames.push_back(frame);
}
```

When loaded from ZDF files ([go to source][stitch_by_transformation_from_files_count-url])

```cpp
// Loop through frames to find the final size
auto maxNumberOfPoints = 0;
for(const auto &frameMap : transformsMappedToFrames)
{
    maxNumberOfPoints += frameMap.mFrame.pointCloud().width() * frameMap.mFrame.pointCloud().height();
}
```

In both cases, the stitched point cloud is initialized the same way:

```cpp
pcl::PointCloud<pcl::PointXYZRGB> stitchedPointCloud;

stitchedPointCloud.points.resize(maxNumberOfPoints);
```

##### Copy data, stitch and add color

We can get points and color for the frame from GPU as follows ([go to source][stitch_by_transform_copy_from_gpu-url]):

```cpp
const auto xyz = pointCloud.copyPointsXYZ();
const auto rgba = pointCloud.copyColorsRGBA();
```

We can then use this to associate color with XYZ when we copy data into the PCL point cloud ([go to source][stitch_by_transform_stitch_and_color-url]).

Note:

1. In order to save memory, we ignore `NaN`s.

```cpp
// Stitch, and add color
const auto rgba = pointCloud.copyColorsRGBA();
const auto xyz = pointCloud.copyPointsXYZ();
for(size_t j = 0; j < pointCloud.size(); j++)
{
    if(!isnan(xyz(j).x))
    {
        stitchedPointCloud.points[validPoints].x =
            xyz(j).x;
        stitchedPointCloud.points[validPoints].y =
            xyz(j).y;
        stitchedPointCloud.points[validPoints].z =
            xyz(j).z;
        if(useRGB)
        {
            stitchedPointCloud.points[validPoints].r =
                rgba(j).r;
            stitchedPointCloud.points[validPoints].g =
                rgba(j).g;
            stitchedPointCloud.points[validPoints].b =
                rgba(j).b;
        }
        else
        {
            stitchedPointCloud.points[validPoints].rgb = rgbList.at(i);
        }
        validPoints++;
    }
}
```

We have an option to give each point a color to indicate which camera it was captured by ([go to source][stitch_by_transform_color-url]).

```cpp
const auto rgbList = std::array<std::uint32_t, 16>{
    0xFFB300, // Vivid Yellow
    0x803E75, // Strong Purple
    0xFF6800, // Vivid Orange
...
```

Now we can free up memory that would have been occupied with `NaN`s ([go to source][stitch_by_transform_resize-url]).

```cpp
stitchedPointCloud.points.resize(k);
```

### Visualize stitched point cloud

[go to source][stitch_by_transform_visualize-url]

```cpp
//Simple Cloud Visualization
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPTR(new pcl::PointCloud<pcl::PointXYZRGB>);
*cloudPTR = stitchedPointCloud;

std::cout << "Run the PCL visualizer. Block until window closes" << std::endl;
pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
viewer.showCloud(cloudPTR);
std::cout << "Press r to centre and zoom the viewer so that the entire cloud is visible" << std::endl;
std::cout << "Press q to me exit the viewer application" << std::endl;
while(!viewer.wasStopped())
{
}
```

### Save stitched point cloud

```cpp
pcl::io::savePLYFileBinary(stitchedPointCloudFileName, stitchedPointCloud);
```

## Conclusion

This tutorial shows how to use the Zivid SDK to calibrate multiple cameras and use the calibration to combine point clouds from multiple cameras into the same coordinate frame.

[//]: ### "Recommended further reading"

[installation-instructions-url]: ../../../../README.md#instructions
[capture_tutorial-url]: ../../../../Camera/Basic/CaptureTutorial.md
[multi_camera_calibration_sample-url]: MultiCameraCalibration/MultiCameraCalibration.cpp
[multi_camera_calibration_sample_from_files-url]: MultiCameraCalibration/MultiCameraCalibrationFromZDF.cpp
[calibration_object-url]: https://support.zivid.com/latest/academy/applications/hand-eye/calibration-object.html
[connect_all_cameras-url]: MultiCameraCalibration/MultiCameraCalibration.cpp#L48
[capture_tutorial_capture_assistant-url]: ../../../../Camera/Basic/CaptureTutorial.md#capture-assistant
[detect_calibration_board-url]: MultiCameraCalibration/MultiCameraCalibration.cpp#L69
[capture_with_ca-url]: MultiCameraCalibration/MultiCameraCalibration.cpp#L67
[verify_checkerboard_capture_quality-url]: MultiCameraCalibration/MultiCameraCalibration.cpp#L70-L79
[calibrate-url]: MultiCameraCalibration/MultiCameraCalibration.cpp#L83
[check_calibration-url]: MultiCameraCalibration/MultiCameraCalibration.cpp#L84-L100
[extract_from_results-url]: MultiCameraCalibration/MultiCameraCalibration.cpp#L87-L88
[saveTransformationMatricesToYAML-url]: MultiCameraCalibration/MultiCameraCalibration.cpp#L91
[stitch_by_transformation-url]: StitchByTransformation/StitchByTransformation.cpp
[stitch_by_transformation_from_files-url]: StitchByTransformationFromZDF/StitchByTransformationFromZDF.cpp
[stitch_by_transformation_map_camera-url]: StitchByTransformation/StitchByTransformation.cpp#L124-L125
[stitch_by_transformation_map_zdf-url]: StitchByTransformation/StitchByTransformationFromZDF.cpp#L121-L122
[stitch_by_transformation_capture-url]: StitchByTransformation/StitchByTransformation.cpp#L127-L136
[stitch_by_transformation_count-url]: StitchByTransformation/StitchByTransformation.cpp#L128-L136
[stitch_by_transformation_from_files_count-url]: StitchByTransformationFromZDF/StitchByTransformationFromZDF.cpp#L124-L129
[stitch_by_transform_transformation-url]: StitchByTransformation/StitchByTransformation.cpp#L148-L151
[stitch_by_transform_transformation_from_files-url]: StitchByTransformationFromZDF/StitchByTransformationFromZDF.cpp#L141-L144
[stitch_by_transform_copy_from_gpu-url]: StitchByTransformation/StitchByTransformation.cpp#L154-L155
[stitch_by_transform_stitch_and_color-url]: StitchByTransformation/StitchByTransformation.cpp#L153-L181
[stitch_by_transform_color-url]: StitchByTransformation/StitchByTransformation.cpp#L68-L85
[stitch_by_transform_resize-url]: StitchByTransformation/StitchByTransformation.cpp#L143
[stitch_by_transform_visualize-url]: StitchByTransformation/StitchByTransformation.cpp#L187-L197
