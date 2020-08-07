# Hand Eye Calibration

To fully understand Hand-Eye Calibration, please see the [tutorial][Tutorial-url] in our Knowledge Base.

-----------------

[HandEyeCalibration][HandEyeCalibration-url]:

* Application which walks through the collection of calibration poses
   1. Provide robot pose to application (manual entry)
   2. Application takes an image of the calibration object, and calculates pose
   3. Move robot to new position, enter command to Add Pose
   4. Repeat 1.-3. until 10-20 pose pairs are collected
   5. Enter command to perform calibration and return a **Transformation Matrix**

[ZividHandEyeCalibration][ZividHandEyeCalibration-url]

* CLI application which takes a collection of pose pairs (e.g. output of steps 1.-3. in [HandEyeCalibration][HandEyeCalibration-url]) and returns a **Transformation Matrix**

-----------------
The following applications assume that a **Transformation Matrix** has been found

[**UtilizeEyeInHandCalibration**][UtilizeEyeInHandCalibration-url]:

* Shows how to transform position and rotation (pose) in Camera co-ordinate system to Robot co-ordinate system.
* Example use case - "Bin Picking":
   1. Acquire point cloud of objects to pick with Zivid camera
   2. Find optimal picking pose for object and **transform to robot co-ordinate system**
   3. Use transformed pose to calculate robot path and execute pick

[**PoseConversions**][PoseConversions-url]:

* Zivid primarily operate with a (4x4) Transformation Matrix (Rotation Matrix + Translation Vector). This example shows how to use Eigen to convert to and from:
  * AxisAngle, Rotation Vector, Roll-Pitch-Yaw, Quaternion

[HandEyeCalibration-url]: HandEyeCalibration/HandEyeCalibration.cpp
[UtilizeEyeInHandCalibration-url]: UtilizeEyeInHandCalibration/UtilizeEyeInHandCalibration.cpp
[ZividHandEyeCalibration-url]: https://zivid.atlassian.net/wiki/spaces/ZividKB/pages/96469274
[Tutorial-url]: https://zivid.atlassian.net/wiki/spaces/ZividKB/pages/72450049
[PoseConversions-url]: PoseConversions/PoseConversions.cpp