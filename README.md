
# cpp-extra-samples

This repository contains additional **C++** code samples for **Zivid**.

The basic samples are available at [https://www.zivid.com/downloads](https://www.zivid.com/downloads). The Windows Zivid installer adds these samples in C:\Users\Public\Documents\Zivid\samples and they should build out of the box using Visual Studio 2015 or 2017. Check out our tutorial on configuring and building these samples on Ubuntu.

## Samples list

- **Zivid**
	- **CaptureHDRCompleteSettings** - This example shows how to acquire an HDR image from the Zivid camera with fully configured settings for each frame.
	- **CaptureHDRLoop** - This example shows how to acquire HDR images from the Zivid camera in a loop (while actively changing some HDR settings).
	- **CaptureSavePLY** - This example shows how to capture a Zivid point cloud and save it to a .PLY file format.
	- **ConnectToSerialNumberCamera** - This example shows how to connect to a specific Zivid camera based on its serial number.
	- **GetCameraIntrinsics** - This example shows how to read the intrinsic calibration parameters of the Zivid camera (OpenCV model).
	- **ReadZDF** - This example shows how to import and display a Zivid point cloud from a .ZDF file.
	- **ZDF2PLY** - This example shows how to convert a Zivid point cloud from a .ZDF file format to a .PLY file format.

- **ZividPCL**
	- **ZDF2PCD** - This example shows how to convert a Zivid point cloud from a .ZDF file format to a .PCD file format.
	- **ReadPCLVis3D** - This example shows how to read a PCL point cloud and visualize it.
	- **CaptureWritePCLVis3D** - This example shows how capture a Zivid point cloud, save it to a .PCD file format, and visualize it.
	- **CaptureFromFileWritePCLVis3D** - This example shows how capture a Zivid point cloud from an emulated Zivid camera, save it to a .PCD file format, and visualize it.

## Instructions

[**Install**](https://zivid.atlassian.net/wiki/spaces/ZividKB/pages/59080712/Zivid+Software+Installation) Zivid Software.
Note: The version tested with Zivid cameras is 1.3.0.

#### Windows

Launch the Command Prompt by pressing *Win + R* keys on the keyboard, then type cmd and press Enter.

Navigage to a location where you want to clone the repository, then run to following command:

```
git clone https://github.com/zivid/cpp-extra-samples
```
[comment]: <> (Choose a sample solution and configure it with CMake.)
[comment]: <> (Launch Visual Studio, open, build, and run the sample solution.)

Choose a sample solution, configure it with CMake, open it in Visual Studio, build it, run it. If you are uncertain about doing this, check out our tutorials for configuring [**Zivid**](https://zivid.atlassian.net/wiki/spaces/ZividKB/pages/61472793/Configure+Zivid+Extra+Samples+with+CMake+and+build+them+using+Visual+Studio+in+Windows) and  [**Zivid & PCL**](https://zivid.atlassian.net/wiki/spaces/ZividKB/pages/61472793/Configure+Zivid+Extra+Samples+with+CMake+and+build+them+using+Visual+Studio+in+Windows) samples with CMake and building them using Visual Studio in Windows.

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

Zivid Samples are distributed under the [BSD license](LICENSE).

