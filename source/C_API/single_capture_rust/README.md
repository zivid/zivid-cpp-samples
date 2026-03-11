# Rust wrapper for zivid-sdk C API

Follow the instructions in the public samples
[README](https://github.com/zivid/zivid-cpp-samples/blob/master/README.md#installation) for how to build the rust sample. This project assumes the library `libCoreCAPI.so` is
built in [build/C_API/CoreCAPI/], and uses the [C bindings](source/C_API/CoreCAPI/).

## Running the Rust Wrapper Sample

Make sure you have the [latest sample data](https://support.zivid.com/en/latest/api-reference/samples/sample-data.html) downloaded to your system. The settings file used in the sample will be fetched from here.

Then, run the following, from the directory of this README:

```shell
cd zivid
LD_LIBRARY_PATH=../../../../build/C_API/CoreCAPI/ cargo run
```
