# C++ samples

This repository contains  **C++** code samples for **Zivid** version 2.5. For tested compatibility with earlier SDK versions, please check out [appropiate release](releases).

[![Build Status][ci-badge]][ci-url]
![Zivid Image][header-image]

---

*Contents:*
[**Samples**](#Samples-list) |
[**Instructions**](#Instructions) |
[**Support**](#Support) |
[**Licence**](#Licence) |
[**Development**](#Development)


## Samples list

There are two main categories of samples: **Camera** and
**Applications**. The samples in the **Camera** category focus only on
how to use the camera. The samples in the **Applications** category use
the output generated by the camera, such as the 3D point cloud, a 2D
image or other data from the camera. These samples shows how the data
from the camera can be used.

  - **Camera**
      - **Basic**
          - [CaptureHDRCompleteSettings](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureHDRCompleteSettings/CaptureHDRCompleteSettings.cpp)
            - Capture point clouds, with color, from the Zivid camera
            with fully configured settings.
          - [CaptureAssistant](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureAssistant/CaptureAssistant.cpp)
            - Capture Assistant to capture point clouds, with color,
            from the Zivid camera.
          - [CaptureFromFileCamera](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureFromFileCamera/CaptureFromFileCamera.cpp)
            - Capture point clouds, with color, from the Zivid file
            camera. Currently supported by Zivid One.
          - [CaptureHDR](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureHDR/CaptureHDR.cpp)
            - Capture HDR point clouds, with color, from the Zivid
            camera.
          - [Capture2D](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture2D/Capture2D.cpp)
            - Capture 2D images from the Zivid camera.
          - [CaptureWithSettingsFromYML](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/CaptureWithSettingsFromYML/CaptureWithSettingsFromYML.cpp)
            - Capture point clouds, with color, from the Zivid camera,
            with settings from YML file.
          - [Capture](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Basic/Capture/Capture.cpp)
            - Capture point clouds, with color, from the Zivid camera.
      - **Advanced**
          - [MultiCameraCaptureInParallel](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Advanced/MultiCameraCaptureInParallel/MultiCameraCaptureInParallel.cpp)
            - Capture point clouds with multiple cameras in parallel.
          - [AllocateMemoryForPointCloudData](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Advanced/AllocateMemoryForPointCloudData/AllocateMemoryForPointCloudData.cpp)
            - Two methods to copy point cloud data from GPU memory to
            CPU memory, to be consumed by OpenCV.
          - [MultiCameraCaptureSequentially](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Advanced/MultiCameraCaptureSequentially/MultiCameraCaptureSequentially.cpp)
            - Capture point clouds with multiple cameras sequentially.
          - [CaptureHDRPrintNormals](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Advanced/CaptureHDRPrintNormals/CaptureHDRPrintNormals.cpp)
            - Capture Zivid point clouds, compute normals and print a
            subset.
          - [CaptureHDRLoop](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Advanced/CaptureHDRLoop/CaptureHDRLoop.cpp)
            - Cover the same dynamic range in a scene with different
            acquisition settings to optimize for quality, speed, or to
            find a compromise.
      - **InfoUtilOther**
          - [CameraUserData](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/InfoUtilOther/CameraUserData/CameraUserData.cpp)
            - Store user data on the Zivid camera.
          - [FirmwareUpdater](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/InfoUtilOther/FirmwareUpdater/FirmwareUpdater.cpp)
            - Update firmware on the Zivid camera.
          - [ZividBenchmark](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/InfoUtilOther/ZividBenchmark/ZividBenchmark.cpp)
            - Zividbenchmarks is a sample that will test the average
            speed of different operations on your computer. It will
            provide
          - [Warmup](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/InfoUtilOther/Warmup/Warmup.cpp)
            - Short example of a basic way to warm up the camera with
            specified time and capture cycle.
          - [SettingsInfo](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/InfoUtilOther/SettingsInfo/SettingsInfo.cpp)
            - Read settings info from the Zivid camera.
          - [PrintVersionInfo](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/InfoUtilOther/PrintVersionInfo/PrintVersionInfo.cpp)
            - List connected cameras and print version information.
          - [GetCameraIntrinsics](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/InfoUtilOther/GetCameraIntrinsics/GetCameraIntrinsics.cpp)
            - Read intrinsic parameters from the Zivid camera (OpenCV
            model).
      - **Maintenance**
          - [ResetCameraInField](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Maintenance/ResetCameraInField/ResetCameraInField.cpp)
            - Reset in-field correction on a camera.
          - [VerifyCameraInField](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Maintenance/VerifyCameraInField/VerifyCameraInField.cpp)
            - Check the dimension trueness of a Zivid camera.
          - [CorrectCameraInField](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Camera/Maintenance/CorrectCameraInField/CorrectCameraInField.cpp)
            - Correct the dimension trueness of a Zivid camera.
  - **Applications**
      - **Basic**
          - **Visualization**
              - [CaptureFromFileCameraVis3D](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Basic/Visualization/CaptureFromFileCameraVis3D/CaptureFromFileCameraVis3D.cpp)
                - Capture point clouds, with color, from the virtual
                Zivid camera, and visualize it. Currently supported by
                Zivid One.
              - [ReadPCLVis3D](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Basic/Visualization/ReadPCLVis3D/ReadPCLVis3D.cpp)
                - Read point cloud from PCL file and visualize it.
              - [CaptureVis3D](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Basic/Visualization/CaptureVis3D/CaptureVis3D.cpp)
                - Capture point clouds, with color, from the Zivid
                camera, and visualize it.
              - [CaptureHDRVisNormals](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Basic/Visualization/CaptureHDRVisNormals/CaptureHDRVisNormals.cpp)
                - Capture Zivid point clouds, with color and normals,
                and visualize it in 3D and as a normal map.
              - [CaptureWritePCLVis3D](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Basic/Visualization/CaptureWritePCLVis3D/CaptureWritePCLVis3D.cpp)
                - Capture point clouds, with color, from the Zivid
                camera, save it to PCD file format, and visualize it.
          - **FileFormats**
              - [ReadIterateZDF](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Basic/FileFormats/ReadIterateZDF/ReadIterateZDF.cpp)
                - Read point cloud data from a ZDF file, iterate through
                it, and extract individual points.
      - **Advanced**
          - [CaptureUndistortRGB](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/CaptureUndistortRGB/CaptureUndistortRGB.cpp)
            - Use camera intrinsics to undistort an RGB image.
          - [CreateDepthMap](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/CreateDepthMap/CreateDepthMap.cpp)
            - Convert point cloud from ZDF file to OpenCV format,
            extract depth map and visualize it.
          - [ROIBoxViaArucoMarker](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/ROIBoxViaArucoMarker/ROIBoxViaArucoMarker.cpp)
            - Filter the point cloud based on a ROI box given relative
            to the ArUco marker.
          - [Downsample](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/Downsample/Downsample.cpp)
            - Downsample point cloud from ZDF file.
          - [TransformPointCloudViaArucoMarker](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/TransformPointCloudViaArucoMarker/TransformPointCloudViaArucoMarker.cpp)
            - Transform a point cloud from camera to ArUco Marker
            coordinate frame by estimating the marker's pose from the
          - [MaskPointCloud](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/MaskPointCloud/MaskPointCloud.cpp)
            - Mask point cloud from ZDF file and convert to PCL format,
            extract depth map and visualize it.
          - [HandEyeCalibration](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/HandEyeCalibration/HandEyeCalibration/HandEyeCalibration.cpp)
            - Perform Hand-Eye calibration.
          - **HandEyeCalibration**
              - [UtilizeEyeInHandCalibration](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/HandEyeCalibration/UtilizeEyeInHandCalibration/UtilizeEyeInHandCalibration.cpp)
                - Transform single data point or entire point cloud from
                camera frame to robot base frame using Eye-in-Hand
                calibration matrix.
              - [PoseConversions](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/HandEyeCalibration/PoseConversions/PoseConversions.cpp)
                - Convert to/from Transformation Matrix (Rotation Matrix + Translation Vector)
          - **MultiCamera**
              - [MultiCameraCalibrationFromZDF](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/MultiCamera/MultiCameraCalibrationFromZDF/MultiCameraCalibrationFromZDF.cpp)
                - Use captures of a calibration object to generate
                transformation matrices to a single coordinate frame,
                from ZDF files.
              - [StitchByTransformationFromZDF](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/MultiCamera/StitchByTransformationFromZDF/StitchByTransformationFromZDF.cpp)
                - Use transformation matrices from Multi-Camera
                calibration to transform point clouds into single
                coordinate frame, from ZDF files.
              - [MultiCameraCalibration](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/MultiCamera/MultiCameraCalibration/MultiCameraCalibration.cpp)
                - Use captures of a calibration object to generate
                transformation matrices to a single coordinate frame,
                from connected cameras.
              - [StitchByTransformation](https://github.com/zivid/zivid-cpp-samples/tree/master//source/Applications/Advanced/MultiCamera/StitchByTransformation/StitchByTransformation.cpp)
                - Use transformation matrices from Multi-Camera
                calibration to transform point clouds into single
                coordinate frame, from connected cameras.

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

``` sourceCode 
git clone https://github.com/zivid/zivid-cpp-samples
```

Configure the sample solution with CMake, open it in Visual Studio,
build it, run it. For more information see [Configure C++ Samples With
CMake and Build Them in Visual Studio in
Windows](https://support.zivid.com/latest/rst/api-reference/samples/cpp/configure-cpp-samples-with-cmake-and-build-them-in-visual-studio-on-windows.html).

**Ubuntu**

Open the Terminal by pressing `Ctrl` + `Alt` + `T` keys on the keyboard.

Navigate to a location where you want to clone the repository, then run
to following command:

``` sourceCode 
git clone https://github.com/zivid/zivid-cpp-samples
cd zivid-cpp-samples
```

Build the project:

``` sourceCode 
mkdir build
cd build
cmake <options, see below> ../source
make -j
```

Some of the samples depend on external libraries, in particular Eigen 3,
OpenCV or PCL. If you don't want to install those, you can disable the
samples depending on them by passing the following options,
respectively, to `cmake`: `-DUSE_EIGEN3=OFF`, `-DUSE_OPENCV=OFF`,
`-DUSE_PCL=OFF`.

If you do want to use them:

  - **Eigen 3**: Set `-DEIGEN3_INCLUDE_DIR=<path>` where `<path>` is the
    root directory of your Eigen3 installation (the folder containing
    Eigen/Core, Eigen/Dense etc.)
  - **PCL** and **OpenCV**: If a recent enough version is installed on
    your system, these should just work. If not, set `-DPCL_DIR=<path>`
    / `-DOpenCV_DIR=<path>` where `<path>` is the directory containing
    `PCLConfig.cmake` and `OpenCVConfig.cmake`, respectively.

Some of the samples depend on ArUco libraries in OpenCV with extra modules (https://github.com/opencv/opencv_contrib) and these are dissabled by default. Enable them by passing the following option to cmake: -DUSE_ARUCO=ON.

The samples can now be run from the build directory, for instance like
this:

``` sourceCode 
./CaptureFromFileCameraVis3D
```

## Support

For more information about the Zivid cameras, please visit our
[Knowledge Base](https://support.zivid.com/latest). If you run into any
issues please check out
[Troubleshooting](https://support.zivid.com/latest/rst/support/troubleshooting.html).

## Licence

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

Tip: If your build hangs, try to increase the memory available to Docker.

-----

[ci-badge]: https://img.shields.io/github/workflow/status/zivid/zivid-cpp-samples/Main%20CI%20workflow/master
[ci-url]: https://github.com/zivid/zivid-cpp-samples/actions?query=workflow%3A%22Main+CI+workflow%22+branch%3Amaster
[header-image]: https://www.zivid.com/hubfs/softwarefiles/images/zivid-generic-github-header.png
