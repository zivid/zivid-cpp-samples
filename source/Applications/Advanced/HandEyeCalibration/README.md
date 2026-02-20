# Hand-Eye Calibration

This page provides an overview of how to **perform**, **verify**, and **use Hand–Eye Calibration** with Zivid cameras.

If you are new to Hand–Eye Calibration, start with the [Hand–Eye Calibration – Concept & Theory][HandEyeTutorial-url], explaining:

- What Hand–Eye Calibration is
- The difference between **eye-in-hand** and **eye-to-hand**
- Best practices for dataset (point clouds and robot poses) acquisition

If you already know what you’re doing and just want to run calibration or check out our Hand-Eye calibration code, continue reading.

<!-- Use "Markdown All in One plugin in VS code to automatically generate and update TOC". -->

- [Quick Start: Just Calibrate](#quick-start-just-calibrate)
- [Programmatic Hand–Eye Calibration](#programmatic-handeye-calibration)
- [Using an Existing Hand–Eye Calibration](#using-an-existing-handeye-calibration)
- [Summary: Which Tool Should I Use?](#summary-which-tool-should-i-use)


---

## Quick Start: Just Calibrate

If your goal is **only to compute the Hand–Eye Transformation Matrix**, use one of the tools below and follow Zivid’s [best-practice guide for capture poses][ZividHandEyeCalibration-url].

### Hand–Eye Calibration GUI (Recommended)

**Note:** This is a **Python-based GUI application** (it is not related to the C++ samples).

- Tutorial: [Hand–Eye GUI Tutorial][HandEyeCalibrationGUITutorial-url]
- Application: [HandEyeCalibration GUI][HandEyeCalibrationGUI-url] (Python)

Best choice if you:

- Want a guided, no-code workflow

---

## Programmatic Hand–Eye Calibration

The following applications produce a Hand–Eye Transformation Matrix from robot poses and calibration captures.

### Minimal Hand-Eye Calibration Code Example

- Sample: [HandEyeCalibration][HandEyeCalibration-url]
- Tutorial: [Integrating Zivid Hand-Eye Calibration][hand-eye-procedure-url]

Workflow:

1. User inputs robot pose in the form of a 4x4 transformation matrix (manual entry)
2. Camera captures the calibration object
3. User moves the robot to a new capture pose and enters the command to add a new pose
4. First three steps are repeated (typically 10–20 pose pairs)
5. User enters the command to perform calibration and the application returns a Hand-Eye Transformation Matrix

Use this if you:

- Want the simplest integration example
- Are building your own calibration pipeline

---

### Hand Eye Calibration CLI Tool

- Tutorial: [Zivid CLI Tool for Hand–Eye Calibration][CLI application-url]
- Installed with:
  - Windows Zivid installer
  -  `tools` deb package on Ubuntu

Use this if you:

- Already have a dataset (robot poses + point clouds)
- Want a command-line, batch-style workflow

---

## Using an Existing Hand–Eye Calibration

The following applications assume that a **Hand–Eye Transformation Matrix already exists**.

### Utilize Hand-Eye Calibration

- Sample: [UtilizeHandEyeCalibration][UtilizeHandEyeCalibration-url]
- Tutorial: [How To Use The Result Of Hand-Eye Calibration][UtilizeHandEyeCalibrationTutorial-url]

Demonstrates how to:

- Transform poses from camera coordinates to robot coordinates
- Use the transform in real applications (e.g., bin picking)

Example workflow:

1. Capture a point cloud with a Zivid camera
2. Find an object pick pose in camera coordinate system
3. Transform the pose into robot coordinate system
4. Plan and execute the robot motion

---

### Pose Conversions

- Sample: [PoseConversions][PoseConversions-url]
- Application: [PoseConversions GUI][PoseConversionsGUI-url] (Python)
- Theory: [Conversions Between Common Orientation Representations][PoseConversionsTheory-url]

Zivid primarily operates with a (4x4) Transformation Matrix (Rotation Matrix + Translation Vector). This example shows how to convert to and from:

- Axis–Angle
- Rotation Vector
- Roll–Pitch–Yaw
- Quaternion

Useful for integrating with robot controllers.

---

## Summary: Which Tool Should I Use?

| Goal | Recommended Tool |
|------|------------------|
| Conceptual understanding | [Knowledge Base article][HandEyeTutorial-url] |
| Guided calibration | [Hand–Eye GUI (Python)][HandEyeCalibrationGUITutorial-url] |
| Minimal integration example | [HandEyeCalibration][HandEyeCalibration-url] |
| Existing dataset | [Hand–Eye GUI (Python)][HandEyeCalibrationGUITutorial-url]|
| Use calibration result | [UtilizeHandEyeCalibration][UtilizeHandEyeCalibrationTutorial-url] |
| Verify visually | [Hand–Eye GUI (Python)][HandEyeCalibrationGUITutorial-url] |
| Verify physically | [Hand–Eye GUI][HandEyeCalibrationGUITutorial-url]  |

[HandEyeTutorial-url]: https://support.zivid.com/latest/academy/applications/hand-eye.html

[HandEyeCalibration-url]: HandEyeCalibration/HandEyeCalibration.cpp

[HandEyeCalibrationGUI-url]: https://github.com/zivid/zivid-python-samples/blob/master/source/applications/advanced/hand_eye_calibration/hand_eye_gui.py
[HandEyeCalibrationGUITutorial-url]: https://support.zivid.com/en/latest/academy/applications/hand-eye/hand-eye-gui.html

[UtilizeHandEyeCalibration-url]: UtilizeHandEyeCalibration/UtilizeHandEyeCalibration.cpp
[UtilizeHandEyeCalibrationTutorial-url]: https://support.zivid.com/en/latest/academy/applications/hand-eye/how-to-use-the-result-of-hand-eye-calibration.html

[ZividHandEyeCalibration-url]: https://support.zivid.com/latest/academy/applications/hand-eye/hand-eye-calibration-process.html
[hand-eye-procedure-url]: https://support.zivid.com/en/latest/academy/applications/hand-eye/hand-eye-calibration-process.html#custom-integration

[PoseConversions-url]: PoseConversions/PoseConversions.cpp
[PoseConversionsGUI-url]: https://github.com/zivid/zivid-python-samples/blob/master/source/applications/advanced/hand_eye_calibration/pose_conversion_gui.py
[PoseConversionsTheory-url]: https://support.zivid.com/en/latest/reference-articles/pose-conversions.html

[CLI application-url]: https://support.zivid.com/latest/academy/applications/hand-eye/zivid_CLI_tool_for_hand_eye_calibration.html
