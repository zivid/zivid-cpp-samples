/*
Capture a 2D+3D frame with the Zivid SDK and build DLPack tensors directly from the Zivid DeviceArrays on the CUDA
device.

This sample captures a combined 2D+3D frame and converts two common buffers to DLPack: the point cloud (PointXYZ) and
the color image (ColorRGBA_SRGB). It constructs a DLManagedTensor from the raw building blocks each DeviceArray exposes -
the device pointer, the shape, the strides (in elements), the element data type, and the CUDA device - and validates its
fields. The DLPack data type is derived generically from the Zivid format's ValueType, so the same code path handles
float point data and 8-bit color without hardcoding per-format constants. This is the producer side of DLPack: a
consumer such as PyTorch (torch.from_dlpack), CuPy, or JAX would import the resulting tensor for zero-copy GPU
processing. The DeviceArray is kept alive by the tensor's manager context and released only when the tensor's deleter is
invoked, so the GPU memory stays valid for as long as the consumer holds the tensor.

To use this sample, you need CUDA installed and the DLPack headers available (see https://github.com/dmlc/dlpack).
Point DLPACK_INCLUDE_DIR at the directory containing dlpack/dlpack.h when configuring CMake.
 */

#include <Zivid/Application.h>
#include <Zivid/Color.h>
#include <Zivid/ComputeDevice.h>
#include <Zivid/DeviceArray.h>
#include <Zivid/Exception.h>
#include <Zivid/Frame.h>
#include <Zivid/Point.h>
#include <Zivid/PointCloud.h>
#include <Zivid/Settings.h>
#include <Zivid/Settings2D.h>

#include <dlpack/dlpack.h>

#include <cuda_runtime.h>

#include <cstdint>
#include <exception>
#include <iostream>
#include <string>
#include <type_traits>
#include <vector>

namespace
{
    void verifyCUDABackend(Zivid::ComputeBackend backend)
    {
        if(backend != Zivid::ComputeBackend::cuda)
        {
            throw std::runtime_error{
                "Only the CUDA compute backend is supported by the `CaptureAndConvertToDlpackTensorOnCuda` sample."
            };
        }
    }

    int cudaDeviceIdForPointer(const void *devicePointer)
    {
        cudaPointerAttributes attributes{};
        const auto status = cudaPointerGetAttributes(&attributes, devicePointer);
        if(status != cudaSuccess)
        {
            throw std::runtime_error{ std::string{ "cudaPointerGetAttributes failed: " } + cudaGetErrorString(status) };
        }
        return attributes.device;
    }

    template<typename Format>
    constexpr DLDataType dlDataType()
    {
        using ValueType = typename Format::ValueType;
        constexpr auto bits = static_cast<uint8_t>(sizeof(ValueType) * 8);
        constexpr uint16_t lanes = 1;
        if constexpr(std::is_same_v<ValueType, float>)
        {
            return DLDataType{ static_cast<uint8_t>(kDLFloat), bits, lanes };
        }
        else if constexpr(std::is_unsigned_v<ValueType>)
        {
            return DLDataType{ static_cast<uint8_t>(kDLUInt), bits, lanes };
        }
        else
        {
            static_assert(
                std::is_integral_v<ValueType> && std::is_signed_v<ValueType>,
                "Unsupported DeviceArray ValueType for DLPack conversion");
            return DLDataType{ static_cast<uint8_t>(kDLInt), bits, lanes };
        }
    }

    template<typename Format>
    struct DLPackManagerContext
    {
        Zivid::DeviceArray<Format> deviceArray;
        std::vector<int64_t> shape;
        std::vector<int64_t> strides;
    };

    template<typename Format>
    void deleteDLManagedTensor(DLManagedTensor *managedTensor)
    {
        delete static_cast<DLPackManagerContext<Format> *>(managedTensor->manager_ctx);
        delete managedTensor;
    }

    template<typename Format>
    DLManagedTensor *createDLManagedTensor(const Zivid::DeviceArray<Format> &deviceArray)
    {
        const auto shape = deviceArray.shape();
        const auto strides = deviceArray.strides();

        auto *context = new DLPackManagerContext<Format>{
            deviceArray,
            std::vector<int64_t>{ shape.begin(), shape.end() },
            std::vector<int64_t>{ strides.begin(), strides.end() },
        };

        auto *managedTensor = new DLManagedTensor{};
        managedTensor->dl_tensor.data = context->deviceArray.devicePointer();
        managedTensor->dl_tensor.device =
            DLDevice{ kDLCUDA, static_cast<int32_t>(cudaDeviceIdForPointer(context->deviceArray.devicePointer())) };
        managedTensor->dl_tensor.ndim = static_cast<int32_t>(context->shape.size());
        managedTensor->dl_tensor.dtype = dlDataType<Format>();
        managedTensor->dl_tensor.shape = context->shape.data();
        managedTensor->dl_tensor.strides = context->strides.data();
        managedTensor->dl_tensor.byte_offset = 0;
        managedTensor->manager_ctx = context;
        managedTensor->deleter = &deleteDLManagedTensor<Format>;

        return managedTensor;
    }

    void printDLManagedTensor(const DLManagedTensor &managedTensor)
    {
        const auto &tensor = managedTensor.dl_tensor;
        std::cout << "DLManagedTensor:\n";
        std::cout << " - data: " << tensor.data << '\n';
        std::cout << " - device: type=kDLCUDA(" << static_cast<int>(tensor.device.device_type)
                  << "), id=" << tensor.device.device_id << '\n';
        std::cout << " - ndim: " << tensor.ndim << '\n';
        std::cout << " - dtype: code=" << static_cast<int>(tensor.dtype.code)
                  << ", bits=" << static_cast<int>(tensor.dtype.bits)
                  << ", lanes=" << static_cast<int>(tensor.dtype.lanes) << '\n';
        std::cout << " - shape: ";
        for(int32_t dimension = 0; dimension < tensor.ndim; ++dimension)
        {
            std::cout << tensor.shape[dimension] << (dimension + 1 < tensor.ndim ? " x " : "");
        }
        std::cout << '\n';
        std::cout << " - strides (elements): ";
        for(int32_t dimension = 0; dimension < tensor.ndim; ++dimension)
        {
            std::cout << tensor.strides[dimension] << (dimension + 1 < tensor.ndim ? ", " : "");
        }
        std::cout << '\n';
    }

    template<typename Format>
    void convertAndValidate(const std::string &description, const Zivid::DeviceArray<Format> &deviceArray)
    {
        std::cout << "Building DLManagedTensor for " << description << " directly from the DeviceArray\n";
        auto *managedTensor = createDLManagedTensor(deviceArray);

        printDLManagedTensor(*managedTensor);

        if(managedTensor->dl_tensor.data != deviceArray.devicePointer())
        {
            throw std::runtime_error{ "Zero-copy check failed: DLManagedTensor does not reference the Zivid buffer." };
        }
        std::cout << "Zero-copy verified: DLManagedTensor references the Zivid device pointer.\n";

        std::cout << "Releasing the DLManagedTensor via its deleter (this is what a consumer calls when done)\n";
        managedTensor->deleter(managedTensor);
    }
} // namespace

int main(int /*argc*/, char * /*argv*/[])
{
    try
    {
        Zivid::Application zivid;
        verifyCUDABackend(zivid.computeDevice().backend());
        std::cout << "Zivid SDK CUDA device: " << zivid.computeDevice().model() << '\n';

        auto camera = zivid.connectCamera();

        const Zivid::Settings2D settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } };
        const Zivid::Settings settings{ Zivid::Settings::Acquisitions{ Zivid::Settings::Acquisition{} },
                                        Zivid::Settings::Color{ settings2D } };

        std::cout << "Capturing a 2D+3D frame\n";
        const auto frame = camera.capture2D3D(settings);
        const auto pointCloud = frame.pointCloud();

        const auto streamOrQueue = zivid.computeDevice().sdkStreamOrQueue();

        std::cout << "Getting GPU device buffers: point cloud XYZ (PointXYZ) and color (ColorRGBA_SRGB)\n";
        const auto pointsDeviceArray = pointCloud.devicePointsXYZ(streamOrQueue);
        const auto colorDeviceArray = pointCloud.imageDeviceArray<Zivid::ColorRGBA_SRGB>(streamOrQueue);

        convertAndValidate("point cloud (PointXYZ)", pointsDeviceArray);
        convertAndValidate("color image (ColorRGBA_SRGB)", colorDeviceArray);

        std::cout << "Each DLManagedTensor owned a reference to its DeviceArray, so the GPU memory stayed valid\n"
                     "for a consumer (e.g. torch.from_dlpack) until the tensor's deleter was invoked.\n";
    }
    catch(const std::exception &ex)
    {
        std::cerr << "Error: " << Zivid::toString(ex) << '\n';
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
