# Rust wrapper for zivid-sdk C API

This uses the [C bindings](../../../public/cpp/source/C_API/CoreCAPI/). Follow the instructions in the public samples
[README](https://github.com/zivid/zivid-cpp-samples/blob/master/README.md#installation) for how to build them. This
projects assumes the library `libCoreCAPI.so` is
built in [../../../../build/C_API/CoreCAPI/]

## Building and running the Rust Wrapper

```shell
cd zivid
LD_LIBRARY_PATH=../../../../build/C_API/CoreCAPI/ cargo run
```
