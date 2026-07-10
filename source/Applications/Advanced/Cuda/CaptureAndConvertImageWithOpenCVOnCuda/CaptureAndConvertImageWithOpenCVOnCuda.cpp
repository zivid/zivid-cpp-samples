/*
Capture a 2D image with the Zivid SDK and perform RGBA to grayscale conversion on a CUDA device using OpenCV.

This sample converts a 2D image from RGBA to grayscale both on the CUDA device (GPU) and on the CPU and then
benchmarks and compares the execution time of each approach.

To use this sample, you need to have CUDA installed on your system and OpenCV with CUDA support built.
 */

#include <Zivid/Application.h>
#include <Zivid/Color.h>
#include <Zivid/ComputeDevice.h>
#include <Zivid/DeviceArray.h>
#include <Zivid/Exception.h>
#include <Zivid/Frame2D.h>
#include <Zivid/Image.h>
#include <Zivid/Settings2D.h>

#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <exception>
#include <iostream>

namespace
{
    void verifyCUDABackend(Zivid::ComputeBackend backend)
    {
        if(backend != Zivid::ComputeBackend::cuda)
        {
            throw std::runtime_error{
                "Only CUDA compute backend is supported by `CaptureAndConvertImageWithOpenCVOnCuda` sample."
            };
        }
    }

    void printShape(const std::vector<int> &arr)
    {
        if(arr.size() == 3)
        {
            std::cout << "Height: " << arr[0] << ", Width: " << arr[1] << ", Channels: " << arr[2];
        }
        else
        {
            throw std::runtime_error{ "Unsupported number of dimensions for shape." };
        }
    }

    void printStrides(const std::vector<int> &arr)
    {
        if(arr.size() == 3)
        {
            std::cout << "Row Stride: " << arr[0] << ", Column Stride: " << arr[1] << ", Channel Stride: " << arr[2];
        }
        else
        {
            throw std::runtime_error{ "Unsupported number of dimensions for strides." };
        }
    }

    template<typename Format>
    void printDeviceArrayInfo(const Zivid::DeviceArray<Format> &deviceArray)
    {
        std::cout << "Device Array Info:\n";
        std::cout << " - Shape: ";
        printShape(deviceArray.shape());
        std::cout << '\n';
        std::cout << " - Strides: ";
        printStrides(deviceArray.strides());
        std::cout << '\n';
    }

    void printCVMatInfo(const cv::cuda::GpuMat &mat)
    {
        std::cout << "cv::cuda::GpuMat Info:\n";
        std::cout << " - Shape: ";
        std::cout << "Height: " << mat.rows << ", Width: " << mat.cols << ", Channels: " << mat.channels() << '\n';
        std::cout << " - Strides: ";
        std::cout << "Row Stride: " << mat.step1() << ", Column Stride: " << mat.elemSize()
                  << ", Channel Stride: " << mat.elemSize1() << '\n';
        std::cout << '\n';
    }

    cv::Mat convertRGBAToGrayscaleOnCudaDevice(const Zivid::Frame2D &frame2D)
    {
        cv::cuda::Stream stream{};
        const Zivid::CUDAStreamPtr userStream{ stream.cudaPtr() };
        const auto deviceBuffer = frame2D.imageDeviceArray<Zivid::ColorRGBA_SRGB>(userStream);
        printDeviceArrayInfo(deviceBuffer);

        const auto cudaBufferPtr = deviceBuffer.devicePointer();
        const auto strides = deviceBuffer.strides();
        const auto shape = deviceBuffer.shape();

        cv::cuda::GpuMat rgbaMat(shape[0], shape[1], CV_8UC4, cudaBufferPtr, strides[0]);
        printCVMatInfo(rgbaMat);
        cv::cuda::GpuMat grayMat;
        cv::cuda::cvtColor(rgbaMat, grayMat, cv::COLOR_RGBA2GRAY, 0, stream);

        cv::Mat resultGpu;
        grayMat.download(resultGpu, stream);
        stream.waitForCompletion();
        return resultGpu;
    }

    cv::Mat convertRGBAToGrayscaleOnCPU(const Zivid::Frame2D &frame2D)
    {
        auto imageRGBA = frame2D.imageRGBA_SRGB();
        auto imageData = const_cast<void *>(static_cast<const void *>(imageRGBA.data()));
        auto width = imageRGBA.width();
        auto height = imageRGBA.height();

        cv::Mat rgbaMat(height, width, CV_8UC4, imageData);
        cv::Mat grayMat;
        cv::cvtColor(rgbaMat, grayMat, cv::COLOR_RGBA2GRAY);

        return grayMat;
    }
} // namespace

int main(int /*argc*/, char * /*argv*/[])
{
    try
    {
        Zivid::Application zivid;
        verifyCUDABackend(zivid.computeDevice().backend());

        std::cout << "Zivid SDK CUDA device: " << zivid.computeDevice().model() << '\n';
        std::cout << "OpenCV CUDA device: ";
        cv::cuda::printShortCudaDeviceInfo(cv::cuda::getDevice());

        auto camera = zivid.connectCamera();
        Zivid::Settings2D settings2D{ Zivid::Settings2D::Acquisitions{ Zivid::Settings2D::Acquisition{} } };
        auto frame2D = camera.capture2D(settings2D);

        // Warmup conversion on CUDA device to avoid measuring CUDA initialization overhead in the benchmark
        convertRGBAToGrayscaleOnCudaDevice(frame2D);

        std::cout << "RGBA->Grayscale conversion on CUDA device" << std::endl;
        const auto gpuStart = cv::getTickCount();
        auto resultGpu = convertRGBAToGrayscaleOnCudaDevice(frame2D);
        const auto gpuEnd = cv::getTickCount();
        const auto gpuTimeMs = (gpuEnd - gpuStart) * 1000.0 / cv::getTickFrequency();

        std::cout << "RGBA->Grayscale conversion on CPU device" << std::endl;
        const auto cpuStart = cv::getTickCount();
        auto dstCpu = convertRGBAToGrayscaleOnCPU(frame2D);
        const auto cpuEnd = cv::getTickCount();
        const auto cpuTimeMs = (cpuEnd - cpuStart) * 1000.0 / cv::getTickFrequency();

        std::cout << "Saving resulting images" << std::endl;
        cv::imwrite("Result_GPU.png", resultGpu);
        cv::imwrite("Result_CPU.png", dstCpu);

        std::cout << "GPU RGBA->Grayscale time: " << gpuTimeMs << " ms\n";
        std::cout << "CPU RGBA->Grayscale time: " << cpuTimeMs << " ms\n";
    }
    catch(const std::exception &ex)
    {
        std::cerr << "Error: " << Zivid::toString(ex) << '\n';
    }
    return 0;
}
