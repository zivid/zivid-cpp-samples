# Hand Eye Calibration

To fully understand Hand-Eye Calibration, please see the [tutorial](https://zivid.atlassian.net/wiki/spaces/ZividKB/pages/72450049) in our Knowledge Base.

-----------------

[SampleHandEyeCalibration]([SampleHandEyeCalibration-url]):

* Application which walks through the collection of calibration poses
   1. Provide robot pose to application (manual entry)
   2. Application takes an image of the calibration object, and calculates pose
   3. Move robot to new position, enter command to Add Pose
   4. Repeat 1.-3. until 10-20 pose pairs are collected
   5. Enter command to perform calibration and return a **Transformation Matrix**

[ZividHandEyeCalibration](C:\Program Files\Zivid\bin\ZividHandEyeCalibration.exe): (no source)

* Application which takes a collection of pose pairs (e.g. output of steps 1.-3. in [SampleHandEyeCalibration]([SampleHandEyeCalibration-url])) and returns a **Transformation Matrix**

-----------------
The following applications assume that a **Transformation Matrix** has been found

[**UtilizeEyeInHandCalibration**]([UtilizeEyeInHandCalibration-url]):

* Shows how to transform position and rotation (pose) in Camera co-ordinate system to Robot co-ordinate system.
* Example use case - "Bin Picking":
   1. Acquire point cloud of objects to pick with Zivid camera
   2. Find optimal picking pose for object and **transform to robot co-ordinate system**
   3. Use transformed pose to calculate robot path and execute pick

[**PoseConversions**]([PoseConversions-url]):

* Zivid primarily operate with a (4x4) Transformation Matrix (Rotation Matrix + Translation Vector). This example shows how to use Eigen to convert to and from:
  * AxisAngle, Rotation Vector, Roll-Pitch-Yaw, Quaternion

[SampleHandEyeCalibration-url]: https://www.zivid.com/hubfs/softwarefiles/releases/1.6.0+7a245bbe-26/doc/cpp/zivid_sample_code.html#autotoc_md10
[UtilizeEyeInHandCalibration-url]: https://github.com/zivid/zivid-cpp-samples/blob/master/Applications/Advanced/HandEyeCalibration/UtilizeEyeInHandCalibration/UtilizeEyeInHandCalibration.cpp
[PoseConversions-url]: https://github.com/zivid/zivid-cpp-samples/blob/master/Applications/Advanced/HandEyeCalibration/PoseConversions/PoseConversions.cpp