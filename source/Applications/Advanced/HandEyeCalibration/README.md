# Hand Eye Calibration

To fully understand Hand-Eye Calibration, please see the [tutorial][Tutorial-url] in our Knowledge Base.

-----------------
The following applications creates a **Transformation Matrix** from data provided by a user

[HandEyeCalibration][HandEyeCalibration-url]:

* Application which walks through the collection of calibration poses
   1. Provide robot pose to application (manual entry)
   2. Application takes an image of the calibration object, and calculates pose
   3. Move robot to new position, enter command to Add Pose
   4. Repeat i.-iii. until 10-20 pose pairs are collected
   5. Enter command to perform calibration and return a **Transformation Matrix**

[ZividHandEyeCalibration][ZividHandEyeCalibration-url]

* [CLI application][CLI application-url] which takes a collection of pose pairs (e.g. output of steps i.-iii. in [HandEyeCalibration][HandEyeCalibration-url]) and returns a **Transformation Matrix**. This application comes with the Windows installer and is part of the tools deb for Ubuntu.

-----------------
The following applications assume that a **Transformation Matrix** has been found

[**UtilizeHandEyeCalibration**][UtilizeHandEyeCalibration-url]:

* Shows how to transform position and rotation (pose) in Camera co-ordinate system to Robot co-ordinate system.
* Example use case - "Bin Picking":
   1. Acquire point cloud of objects to pick with Zivid camera
   2. Find optimal picking pose for object and **transform to robot co-ordinate system**
   3. Use transformed pose to calculate robot path and execute pick

[**PoseConversions**][PoseConversions-url]:

* Zivid primarily operate with a (4x4) Transformation Matrix (Rotation Matrix + Translation Vector). This example shows how to use Eigen to convert to and from:
  * AxisAngle, Rotation Vector, Roll-Pitch-Yaw, Quaternion

[HandEyeCalibration-url]: HandEyeCalibration/HandEyeCalibration.cpp
[UtilizeHandEyeCalibration-url]: UtilizeHandEyeCalibration/UtilizeHandEyeCalibration.cpp
[ZividHandEyeCalibration-url]: https://support.zivid.com/latest/academy/applications/hand-eye/hand-eye-calibration-process.html
[Tutorial-url]: https://support.zivid.com/latest/academy/applications/hand-eye.html
[PoseConversions-url]: PoseConversions/PoseConversions.cpp
[CLI application-url]: https://support.zivid.com/latest/academy/applications/hand-eye/zivid_CLI_tool_for_hand_eye_calibration.html