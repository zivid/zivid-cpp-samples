# C++ samples

This repository contains cpp code samples for Zivid SDK v2.15.0. For
tested compatibility with earlier SDK versions, please check out
[accompanying
releases](https://github.com/zivid/zivid-cpp-samples/tree/master/../../releases).

![image](https://www.zivid.com/hubfs/softwarefiles/images/zivid-generic-github-header.png)



---

*Contents:*
[**Tutorials**](#Tutorials-list) |
[**Samples**](#Samples-list) |
[**Installation**](#Installation) |
[**Support**](#Support) |
[**License**](#License) |
[**Development**](#Development)

---



## Tutorials list

  - [QuickCaptureTutorial](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Basic/QuickCaptureTutorial.md)
  - [CaptureTutorial](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Basic/CaptureTutorial.md)
  - [PointCloudTutorial](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/PointCloudTutorial.md)

## Samples list

There are two main categories of samples: **Camera** and
**Applications**. The samples in the **Camera** category focus only on
how to use the camera. The samples in the **Applications** category use
the output generated by the camera, such as the 3D point cloud, a 2D
image or other data from the camera. These samples shows how the data
from the camera can be used.

  - **Camera**
      - **Basic**
          - [Capture](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Basic/Capture/Capture.cpp) - Capture colored point cloud, save 2D image, save 3D ZDF,
            and export PLY, using the Zivid camera.
          - [CaptureFromFileCamera](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Basic/CaptureFromFileCamera/CaptureFromFileCamera.cpp) - Capture point clouds, with color, with the Zivid file
            camera.
          - [CaptureHDRCompleteSettings](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Basic/CaptureHDRCompleteSettings/CaptureHDRCompleteSettings.cpp) - Capture point clouds, with color, from the Zivid camera
            with fully configured settings.
          - [CaptureWithSettingsFromYML](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Basic/CaptureWithSettingsFromYML/CaptureWithSettingsFromYML.cpp) - Capture images and point clouds, with and without color,
            from the Zivid camera with settings from YML file.
      - **Advanced**
          - [AllocateMemoryForPointCloudData](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Advanced/AllocateMemoryForPointCloudData/AllocateMemoryForPointCloudData.cpp) - Two methods to copy point cloud data from GPU memory to
            CPU memory, to be consumed by OpenCV.
          - [Capture2DAnd3D](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Advanced/Capture2DAnd3D/Capture2DAnd3D.cpp) - Capture 2D and 3D with the Zivid camera.
          - [CaptureHalconViaGenICam](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Advanced/CaptureHalconViaGenICam/CaptureHalconViaGenICam.cpp) - Capture and save a point cloud, with colors, using GenICam
            interface and Halcon C++ SDK.
          - [CaptureHalconViaZivid](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Advanced/CaptureHalconViaZivid/CaptureHalconViaZivid.cpp) - Capture a point cloud, with colors, using Zivid SDK,
            transform it to a Halcon point cloud and save it using
            Halcon C++ SDK.
          - [CaptureHDRLoop](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Advanced/CaptureHDRLoop/CaptureHDRLoop.cpp) - Cover the same dynamic range in a scene with different
            acquisition settings to optimize for quality, speed, or to
            find a compromise.
          - [CaptureHDRPrintNormals](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Advanced/CaptureHDRPrintNormals/CaptureHDRPrintNormals.cpp) - Capture Zivid point clouds, compute normals and print a
            subset.
          - [CaptureViaGenICam](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Advanced/CaptureViaGenICam/CaptureViaGenICam.cpp) - Capture using the GenICam interface.
          - [MultiCameraCaptureInParallel](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Advanced/MultiCameraCaptureInParallel/MultiCameraCaptureInParallel.cpp) - Capture point clouds with multiple cameras in parallel.
          - [MultiCameraCaptureSequentially](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Advanced/MultiCameraCaptureSequentially/MultiCameraCaptureSequentially.cpp) - Capture point clouds with multiple cameras sequentially.
          - [MultiCameraCaptureSequentiallyWithInterleavedProcessing](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Advanced/MultiCameraCaptureSequentiallyWithInterleavedProcessing/MultiCameraCaptureSequentiallyWithInterleavedProcessing.cpp) - Capture point clouds with multiple cameras sequentially
            with interleaved processing.
      - **InfoUtilOther**
          - [AutomaticNetworkConfigurationForCameras](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/InfoUtilOther/AutomaticNetworkConfigurationForCameras/AutomaticNetworkConfigurationForCameras.cpp) - \* Automatically configure the IP addresses of connected
            cameras to match the network of the user's PC.
          - [CameraInfo](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/InfoUtilOther/CameraInfo/CameraInfo.cpp) - List connected cameras and print camera version and state
            information for each connected camera.
          - [CameraUserData](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/InfoUtilOther/CameraUserData/CameraUserData.cpp) - Store user data on the Zivid camera.
          - [CaptureWithDiagnostics](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/InfoUtilOther/CaptureWithDiagnostics/CaptureWithDiagnostics.cpp) - Capture point clouds, with color, from the Zivid camera,
            with settings from YML file and diagnostics enabled.
          - [FirmwareUpdater](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/InfoUtilOther/FirmwareUpdater/FirmwareUpdater.cpp) - Update firmware on the Zivid camera.
          - [FrameInfo](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/InfoUtilOther/FrameInfo/FrameInfo.cpp) - Read frame info from the Zivid camera.
          - [GetCameraIntrinsics](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/InfoUtilOther/GetCameraIntrinsics/GetCameraIntrinsics.cpp) - Read intrinsic parameters from the Zivid camera (OpenCV
            model) or estimate them from the point cloud.
          - [NetworkConfiguration](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/InfoUtilOther/NetworkConfiguration/NetworkConfiguration.cpp) - Uses Zivid API to change the IP address of the Zivid
            camera.
          - [SettingsInfo](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/InfoUtilOther/SettingsInfo/SettingsInfo.cpp) - Read settings info from the Zivid camera.
          - [Warmup](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/InfoUtilOther/Warmup/Warmup.cpp) - Short example of a basic way to warm up the camera with
            specified time and capture cycle.
          - [ZividBenchmark](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/InfoUtilOther/ZividBenchmark/ZividBenchmark.cpp) - Zividbenchmark is a sample that will test the average
            speed of different operations on your computer.
      - **Maintenance**
          - [CorrectCameraInField](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Maintenance/CorrectCameraInField/CorrectCameraInField.cpp) - Correct the dimension trueness of a Zivid camera.
          - [ResetCameraInField](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Maintenance/ResetCameraInField/ResetCameraInField.cpp) - Reset infield correction on a camera.
          - [VerifyCameraInField](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Maintenance/VerifyCameraInField/VerifyCameraInField.cpp) - Check the dimension trueness of a Zivid camera.
          - [VerifyCameraInFieldFromZDF](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Camera/Maintenance/VerifyCameraInFieldFromZDF/VerifyCameraInFieldFromZDF.cpp) - Check the dimension trueness of a Zivid camera from a ZDF
            file.
  - **Applications**
      - **Basic**
          - **Visualization**
              - [CaptureFromFileCameraVis3D](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Basic/Visualization/CaptureFromFileCameraVis3D/CaptureFromFileCameraVis3D.cpp) - Capture point clouds, with color, with the Zivid file
                camera and visualize them.
              - [CaptureHDRVisNormals](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Basic/Visualization/CaptureHDRVisNormals/CaptureHDRVisNormals.cpp) - Capture Zivid point clouds, with color and normals,
                and visualize it in 3D and as a normal map.
              - [CaptureVis3D](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Basic/Visualization/CaptureVis3D/CaptureVis3D.cpp) - Capture point clouds, with color, from the Zivid
                camera, and visualize it.
              - [CaptureWritePCLVis3D](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Basic/Visualization/CaptureWritePCLVis3D/CaptureWritePCLVis3D.cpp) - Capture point clouds, with color, from the Zivid
                camera, save it to PCD file format, and visualize it.
              - [ProjectImageStartAndStop](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Basic/Visualization/ProjectImageStartAndStop/ProjectImageStartAndStop.cpp) - Start the Image Projection and Stop it.
              - [ReadAndProjectImage](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Basic/Visualization/ReadAndProjectImage/ReadAndProjectImage.cpp) - Read a 2D image from file and project it using the
                camera projector.
              - [ReadPCLVis3D](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Basic/Visualization/ReadPCLVis3D/ReadPCLVis3D.cpp) - Read point cloud from PCL file and visualize it.
          - **FileFormats**
              - [ReadIterateZDF](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Basic/FileFormats/ReadIterateZDF/ReadIterateZDF.cpp) - Read point cloud data from a ZDF file, iterate through
                it, and extract individual points.
      - **Advanced**
          - [CaptureUndistort2D](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Advanced/CaptureUndistort2D/CaptureUndistort2D.cpp) - Use camera intrinsics to undistort a 2D image.
          - [CreateDepthMap](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Advanced/CreateDepthMap/CreateDepthMap.cpp) - Convert point cloud from a ZDF file to OpenCV format,
            extract depth map and visualize it.
          - [Downsample](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Advanced/Downsample/Downsample.cpp) - Downsample point cloud from a ZDF file.
          - [GammaCorrection](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Advanced/GammaCorrection/GammaCorrection.cpp) - Capture 2D image with gamma correction.
          - [HandEyeCalibration](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Advanced/HandEyeCalibration/HandEyeCalibration/HandEyeCalibration.cpp) - Perform Hand-Eye calibration.
          - [MaskPointCloud](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Advanced/MaskPointCloud/MaskPointCloud.cpp) - Mask point cloud from a ZDF file and convert to PCL
            format, extract depth map and visualize it.
          - [ProjectAndFindMarker](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Advanced/ProjectAndFindMarker/ProjectAndFindMarker.cpp) - Show a marker using the projector, capture a set of 2D
            images to find the marker coordinates (2D and 3D).
          - [ReprojectPoints](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Advanced/ReprojectPoints/ReprojectPoints.cpp) - Illuminate checkerboard (Zivid Calibration Board) corners
            by getting checkerboard pose
          - [ROIBoxViaArucoMarker](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Advanced/ROIBoxViaArucoMarker/ROIBoxViaArucoMarker.cpp) - Filter the point cloud based on a ROI box given relative
            to the ArUco marker on a Zivid Calibration Board.
          - [ROIBoxViaCheckerboard](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Advanced/ROIBoxViaCheckerboard/ROIBoxViaCheckerboard.cpp) - Filter the point cloud based on a ROI box given relative
            to the Zivid Calibration Board.
          - [TransformPointCloudFromMillimetersToMeters](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Advanced/TransformPointCloudFromMillimetersToMeters/TransformPointCloudFromMillimetersToMeters.cpp) - Transform point cloud data from millimeters to meters.
          - [TransformPointCloudViaArucoMarker](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Advanced/TransformPointCloudViaArucoMarker/TransformPointCloudViaArucoMarker.cpp) - Transform a point cloud from camera to ArUco marker
            coordinate frame by estimating the marker's pose from the
            point cloud.
          - [TransformPointCloudViaCheckerboard](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Advanced/TransformPointCloudViaCheckerboard/TransformPointCloudViaCheckerboard.cpp) - Transform a point cloud from camera to checkerboard (Zivid
            Calibration Board) coordinate frame by getting checkerboard
            pose from the API.
          - **HandEyeCalibration**
              - [PoseConversions](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Advanced/HandEyeCalibration/PoseConversions/PoseConversions.cpp) - Convert to/from Transformation Matrix (Rotation Matrix
                + Translation Vector)
              - [UtilizeHandEyeCalibration](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Advanced/HandEyeCalibration/UtilizeHandEyeCalibration/UtilizeHandEyeCalibration.cpp) - Transform single data point or entire point cloud from
                camera to robot base reference frame using Hand-Eye
                calibration
          - **MultiCamera**
              - [MultiCameraCalibration](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Advanced/MultiCamera/MultiCameraCalibration/MultiCameraCalibration.cpp) - Use captures of a calibration object to generate
                transformation matrices to a single coordinate frame,
                from connected cameras.
              - [MultiCameraCalibrationFromZDF](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Advanced/MultiCamera/MultiCameraCalibrationFromZDF/MultiCameraCalibrationFromZDF.cpp) - Use captures of a calibration object to generate
                transformation matrices to a single coordinate frame,
                from a ZDF files.
              - [StitchByTransformation](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Advanced/MultiCamera/StitchByTransformation/StitchByTransformation.cpp) - Use transformation matrices from Multi-Camera
                calibration to transform point clouds into single
                coordinate frame, from connected cameras.
              - [StitchByTransformationFromZDF](https://github.com/zivid/zivid-cpp-samples/tree/master/source/Applications/Advanced/MultiCamera/StitchByTransformationFromZDF/StitchByTransformationFromZDF.cpp) - Use transformation matrices from Multi-Camera
                calibration to transform point clouds into single
                coordinate frame, from a ZDF files.

## Installation

1.  [Install Zivid
    Software](https://support.zivid.com/latest//getting-started/software-installation.html)
2.  [Download Zivid Sample
    Data](https://support.zivid.com/latest//api-reference/samples/sample-data.html)

**Windows**

Launch the Command Prompt by pressing `Win` + `R` keys on the keyboard,
then type `cmd` and press `Enter`.

Navigate to a location where you want to clone the repository, then run
to following command:

``` sourceCode bat
git clone https://github.com/zivid/zivid-cpp-samples
```

Configure the sample solution with CMake, open it in Visual Studio,
build it, run it. For more information see [Configure C++ Samples With
CMake and Build Them in Visual Studio in
Windows](https://support.zivid.com/latest/api-reference/samples/cpp/configure-cpp-samples-with-cmake-and-build-them-in-visual-studio-on-windows.html).

**Ubuntu**

Open the Terminal by pressing `Ctrl` + `Alt` + `T` keys on the keyboard.

Navigate to a location where you want to clone the repository, then run
to following command:

``` sourceCode bash
git clone https://github.com/zivid/zivid-cpp-samples
cd zivid-cpp-samples
```

Build the project:

``` sourceCode bash
mkdir build
cd build
cmake <options, see below> ../source
make -j
```

Some of the samples depend on external libraries, in particular Eigen 3,
OpenCV, PCL, or HALCON. If you don't want to install those, you can
disable the samples depending on them by passing the following options,
respectively, to `cmake`: `-DUSE_EIGEN3=OFF`, `-DUSE_OPENCV=OFF`,
`-DUSE_PCL=OFF`, `-DUSE_HALCON=OFF`.

If you do want to use them:

  - **Eigen 3**: Set `-DEIGEN3_INCLUDE_DIR=<path>` where `<path>` is the
    root directory of your Eigen3 installation (the folder containing
    Eigen/Core, Eigen/Dense etc.)
  - **PCL** and **OpenCV**: If a recent enough version is installed on
    your system, these should just work. If not, set `-DPCL_DIR=<path>`
    / `-DOpenCV_DIR=<path>` where `<path>` is the directory containing
    `PCLConfig.cmake` and `OpenCVConfig.cmake`, respectively.
  - **HALCON**: If a recent enough version is installed on your system,
    these should just work.

The samples can now be run from the build directory, for instance like
this:

``` sourceCode bash
./CaptureFromFileCameraVis3D
```

### HALCON

Zivid offers two ways of interfacing with HALCON:

1.  Through the Zivid SDK, utilizing the C++/C\# libraries available for
    HALCON. We provide samples for both
    [C++](https://support.zivid.com/latest//api-reference/samples/cpp.html)
    and
    [C\#](https://support.zivid.com/latest//api-reference/samples/csharp.html).
    (**Recommended**)
2.  Directly through a GenICam GenTL producer that comes with the [Zivid
    Software](https://support.zivid.com/latest//getting-started/software-installation.html).

Zivid and HALCON are compatible with Windows 10 and 11, and Ubuntu
20.04, 22.04, 24.04.

-----

Note:

> Support for Ubuntu 18.04 is removed since SDK 2.10.

-----

To set up and use Zivid in one of these operating systems, please follow
their respective instructions on the following pages:

  - [Install Zivid + HALCON for
    Windows](https://support.zivid.com/latest/api-reference/samples/halcon/install-zivid-halcon-for-windows.html)
  - [Install Zivid + HALCON for
    LINUX](https://support.zivid.com/latest/api-reference/samples/halcon/install-zivid-halcon-for-linux.html)
  - [Create a HALCON "Hello World"
    Program](https://support.zivid.com/latest/api-reference/samples/halcon/create-a-halcon-hello-world.html)
  - [How to Run a HALCON
    Sample](https://support.zivid.com/latest/api-reference/samples/halcon/how-to-run-a-halcon-sample.html)
  - [Debug in
    HALCON](https://support.zivid.com/latest/api-reference/samples/halcon/halcon-debug.html)
  - [HALCON Sample
    Videos](https://support.zivid.com/latest/api-reference/samples/halcon/halcon-sample-videos.html)

The following HALCON versions have been tested and confirmed to work
with Zivid cameras:

  - 19.05 Progress, 20.05 Progress, 21.11 Progress, 24.05 Progress,
    24.11 Progress-Steady

We recommend using one of the HALCON versions we have tested.

## Support

For more information about the Zivid cameras, please visit our
[Knowledge Base](https://support.zivid.com/latest). If you run into any
issues please check out
[Troubleshooting](https://support.zivid.com/latest/support/troubleshooting.html).

## License

Zivid Samples are distributed under the [BSD
license](https://github.com/zivid/zivid-cpp-samples/tree/master/LICENSE).

## Development

To run continuous integration locally, use
[Docker](https://www.docker.com). With Docker installed, run this
command:

``` sourceCode bash
docker run -it -v <unix-style-repo-path>:/host -w /host/continuous-integration/linux ubuntu:20.04
```

Where `<unix-style-repo-path>` is the unix-style path to the repo on
your computer. On Linux, use `$PWD` for this. On Windows you need to
translate the windows-style path to a unix-style one (e.g.
`/c/Users/alice/Documents/zivid-cpp-samples`).

Now run `./setup.sh` to install dependencies. Once setup has completed,
you can run `./lint.sh && ./build.sh` repeatedly to check your code.

-----

Tip:

If your build hangs, try to increase the memory available to Docker.
