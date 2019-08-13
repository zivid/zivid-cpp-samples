

# cpp-extra-samples

This repository contains additional **C++** code samples for **Zivid**.

The basic samples are available at [https://www.zivid.com/downloads](https://www.zivid.com/downloads).
The Windows Zivid installer adds these samples in C:\Users\Public\Documents\Zivid\samples and they should build out of the box using Visual Studio 2015 or 2017.
Check out our [tutorial on configuring and building these samples on Ubuntu](https://zivid.atlassian.net/wiki/spaces/ZividKB/pages/59441336/Configure+C+Samples+with+CMake+and+then+build+them+using+make+in+Ubuntu).

## Samples list

- [**Zivid**](https://github.com/zivid/cpp-extra-samples/tree/master/Zivid)
	- [**CaptureHDRCompleteSettings**](https://github.com/zivid/cpp-extra-samples/tree/master/Zivid/CaptureHDRCompleteSettings/CaptureHDRCompleteSettings.cpp) - This example shows how to acquire an HDR image from the Zivid camera with fully configured settings for each frame.
	- [**CaptureHDRLoop**](https://github.com/zivid/cpp-extra-samples/tree/master/Zivid/CaptureHDRLoop/CaptureHDRLoop.cpp) - This example shows how to acquire HDR images from the Zivid camera in a loop (while actively changing some HDR settings).
	- [**CaptureSavePLY**](https://github.com/zivid/cpp-extra-samples/tree/master/Zivid/CaptureSavePLY/CaptureSavePLY.cpp) - This example shows how to capture a Zivid point cloud and save it to a .PLY file format.
	- [**ConnectToSerialNumberCamera**](https://github.com/zivid/cpp-extra-samples/tree/master/Zivid/ConnectToSerialNumberCamera/ConnectToSerialNumberCamera.cpp) - This example shows how to connect to a specific Zivid camera based on its serial number.
	- [**GetCameraIntrinsics**](https://github.com/zivid/cpp-extra-samples/tree/master/Zivid/GetCameraIntrinsics/GetCameraIntrinsics.cpp) - This example shows how to read the intrinsic calibration parameters of the Zivid camera (OpenCV model).
	- [**ReadIterateZDF**](https://github.com/zivid/cpp-extra-samples/tree/master/Zivid/ReadIterateZDF/ReadIterateZDF.cpp) - This example shows how to import, display and iterate over a Zivid point cloud from a .ZDF
file.
	- [**ZDF2PLY**](https://github.com/zivid/cpp-extra-samples/tree/master/Zivid/ZDF2PLY/ZDF2PLY.cpp) - This example shows how to convert a Zivid point cloud from a .ZDF file format to a .PLY file format.

- [**ZividPCL**](https://github.com/zivid/cpp-extra-samples/tree/master/ZividPCL)
	- [**ZDF2PCD**](https://github.com/zivid/cpp-extra-samples/blob/master/ZividPCL/ZDF2PCD/ZDF2PCD.cpp) - This example shows how to convert a Zivid point cloud from a .ZDF file format to a .PCD file format.
	- [**ReadPCLVis3D**](https://github.com/zivid/cpp-extra-samples/blob/master/ZividPCL/ReadPCLVis3D/ReadPCLVis3D.cpp) - This example shows how to read a PCL point cloud and visualize it.
	- [**CaptureWritePCLVis3D**](https://github.com/zivid/cpp-extra-samples/blob/master/ZividPCL/CaptureWritePCLVis3D/CaptureWritePCLVis3D.cpp) - This example shows how capture a Zivid point cloud, save it to a .PCD file format, and visualize it.
	- [**CaptureFromFileWritePCLVis3D**](https://github.com/zivid/cpp-extra-samples/blob/master/ZividPCL/CaptureFromFileWritePCLVis3D/CaptureFromFileWritePCLVis3D.cpp) - This example shows how capture a Zivid point cloud from an emulated Zivid camera, save it to a .PCD file format, and visualize it.
	- **Dependencies:**
		- [PCL](http://pointclouds.org/) version 1.9.1

- [**Downsample**](https://github.com/zivid/cpp-extra-samples/tree/master/Downsample)  - This example shows how to import a Zivid point cloud from a .ZDF file and downsample it.
	- **Dependencies:**
		- [Eigen](http://eigen.tuxfamily.org/) version 3.3.90 or newer

## Instructions

[**Install Zivid Software**](https://zivid.atlassian.net/wiki/spaces/ZividKB/pages/59080712/Zivid+Software+Installation).
Note: The version tested with Zivid cameras is 1.4.0.

#### Windows

Launch the Command Prompt by pressing *Win + R* keys on the keyboard, then type cmd and press Enter.

Navigage to a location where you want to clone the repository, then run to following command:

```
git clone https://github.com/zivid/cpp-extra-samples
```
[comment]: <> (Choose a sample solution and configure it with CMake.)
[comment]: <> (Launch Visual Studio, open, build, and run the sample solution.)

Choose a sample solution, configure it with CMake, open it in Visual Studio, build it, run it. If you are uncertain about doing this, check out our tutorials for configuring [**Zivid**](https://zivid.atlassian.net/wiki/spaces/ZividKB/pages/61472793/Configure+Zivid+Extra+Samples+with+CMake+and+build+them+using+Visual+Studio+in+Windows), [**Zivid & PCL**](https://zivid.atlassian.net/wiki/spaces/ZividKB/pages/44204066/Configure+C+Zivid+Point+Cloud+Library+PCL+Samples+with+CMake+and+build+them+using+Visual+Studio+in+Windows), and [**Downsample**](https://zivid.atlassian.net/wiki/spaces/ZividKB/pages/76480513/Configure+C+Downsample+Sample+with+CMake+and+build+it+using+Visual+Studio+in+Windows) samples with CMake and building them using Visual Studio in Windows.

#### Ubuntu

Open the Terminal by pressing *Ctrl + Alt + T* keys on the keyboard.

Navigage to a location where you want to clone the repository, then run to following command:

```
git clone https://github.com/zivid/cpp-extra-samples
```

 Navigage to the sample solution directory:
```
cd cpp-extra-samples/Zivid
```

Build the sample project:
```
mkdir build
cd build
cmake ..
make -j
```

Run the sample:
```
./ReadZDF
```

## Support

If you need assistance with using Zivid cameras, visit our [**Knowledge Base**](https://help.zivid.com/) or contact us at [customersuccess@zivid.com](mailto:customersuccess@zivid.com).

## Licence

Zivid Samples are distributed under the [BSD license](https://github.com/zivid/cpp-extra-samples/blob/master/LICENSE).

