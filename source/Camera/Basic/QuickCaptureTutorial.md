## Introduction

This tutorial shows how few API calls are required to capture a point cloud with Zivid SDK.

1. [Connect](#connect)
2. [Capture](#capture)
3. [Save](#save)

### Prerequisites

You should have installed Zivid SDK and C++ samples. For more details see [Instructions][installation-instructions-url].

Before calling any of the APIs in the Zivid SDK, we have to start up the Zivid Application. This is done through a simple instantiation of the application ([go to source][start_app-url]).
```cpp
Zivid::Application zivid;
```

## Connect

First we have to connect to the camera ([go to source][connect-url]).
```cpp
auto camera = zivid.connectCamera();
```

## Capture

Now we can capture a frame. The default capture is a single 3D point cloud ([go to source][capture-url]).
```cpp
auto frame = camera.capture();
```

## Save

We can now save our results. By default the 3D point cloud is saved in Zivid format `.zdf` ([go to source][save-url]).
```cpp
auto resultFile = "Result.zdf";
frame.save(resultFile);
```

## Conclusion

This tutorial showed how few API calls are required to capture a point cloud with Zivid SDK.

### Recommended further reading

[The complete Capture Tutorial](CaptureTutorial.md)

[installation-instructions-url]: ../../../README.md#instructions
[start_app-url]: Capture/Capture.cpp#L10
[connect-url]: Capture/Capture.cpp#L15
[capture-url]: Capture/Capture.cpp#L22
[save-url]: Capture/Capture.cpp#L25
