# Multi Camera

To use multiple Zivid Two cameras, set [custom IP configuration](https://support.zivid.com/latest/academy/getting-started/zivid-software-installation/zivid-two-network-configuration.html#custom-ip-configuration). 

## Multi Camera Calibration

To fully understand Multi Camera Calibration, please see the [tutorial](https://support.zivid.com/latest/academy/applications/multi-camera-calibration.html) in our Knowledge Base. There is also a [programming tutorial][MultiCameraTutorial-url] that walks through the samples in detail.

-----------------

[**MultiCameraCalibration**]([MultiCameraCalibration-url]) and [**MultiCameraCalibrationFromZDF**]([MultiCameraCalibrationFromZDF-url]):

* These applications show how to use the Zivid SDK to calibrate multiple cameras against each other. The result is a transformation matrix from all cameras into one camera. In other words, all point clouds can be represented in the same coordinate frame. The sample is broken down as follows:
   1. Connect to cameras (only in case of [MultiCameraCalibration]([MultiCameraCalibration-url]))
   2. Capture calibration object or load from ZDF files
   3. Detect checkerboard 3D feature points
   4. Perform Multi-Camera Calibration
   5. Save transformation matrices to a YAML file

-----------------

The following applications assume that the **Transformation Matrices** between the cameras are known.

[**StitchByTransformation**]([StitchByTransformation-url]) and [**StitchByTransformationFromZDF**]([StitchByTransformationFromZDF-url]):

* These applications show how to use transformation matrices to convert multiple point clouds into the same coordinate frame.
   1. Load transformation matrices and map to point cloud. See [tutorial]([MultiCameraTutorial_Map-url]) for more information.
   2. Apply transformation matrixes with corresponding point clouds and stitch them with the first point cloud
   3. Visualize stitched point cloud
   4. Save stitched point cloud

[MultiCameraTutorial-url]: MultiCameraTutorial.md
[MultiCameraCalibration-url]: MultiCameraCalibration/MultiCameraCalibration.cpp
[MultiCameraCalibrationFromZDF-url]: MultiCameraCalibrationFromZDF/MultiCameraCalibrationFromZDF.cpp
[StitchByTransformation-url]: StitchByTransformation/StitchByTransformation.cpp
[StitchByTransformationFromZDF-url]: StitchByTransformationFromZDF/StitchByTransformationFromZDF.cpp
[MultiCameraTutorial_Map-url]: MultiCameraTutorial.md#load-associated-transformation-matrices-and-map-to-capture-or-camera