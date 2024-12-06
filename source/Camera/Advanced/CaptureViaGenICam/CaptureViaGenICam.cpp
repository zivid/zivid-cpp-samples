/*
Capture using the GenICam interface.
*/

#include <Zivid/GenTLAddresses.h>
#include <Zivid/Zivid.h>

#include <GenTL.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>

namespace
{
    using namespace ZividGenTL::RegisterAddresses;

    struct PointXYZ
    {
        float x;
        float y;
        float z;
    };

    static_assert(sizeof(PointXYZ) == 12);

    struct RGBA
    {
        uint8_t r;
        uint8_t g;
        uint8_t b;
        uint8_t a;
    };

    static_assert(sizeof(RGBA) == 4);

    void checkAndThrow(GenTL::GC_ERROR errorCode, const std::string &message)
    {
        if(errorCode != GenTL::GC_ERR_SUCCESS)
        {
            std::string errorMessage(1024, '\0');
            auto errorMessageSize = errorMessage.size();
            auto err1 = GenTL::GCGetLastError(&errorCode, errorMessage.data(), &errorMessageSize);
            if(err1 != GenTL::GC_ERR_SUCCESS)
            {
                throw std::runtime_error{ message + " GenTL error code: " + std::to_string(errorCode) + "\n"
                                          + "Could not find the cause. Error code when getting cause: "
                                          + std::to_string(err1) };
            }
            throw std::runtime_error{ message + " GenTL error code: " + std::to_string(errorCode) + "\n"
                                      + "Cause: " + errorMessage };
        }
    }

    template<class Function, typename... Ts>
    void checkedTLCall(Function &function, const std::string &message, Ts &&...ts)
    {
        auto errorCode = function(std::forward<Ts>(ts)...);
        checkAndThrow(errorCode, message);
    }

    std::string intToHex(uint64_t i)
    {
        std::stringstream stream;
        stream << std::showbase << std::hex << i;
        return stream.str();
    }

    template<typename T>
    void setZividRegister(GenTL::PORT_HANDLE remDevHandle, uint64_t iAddress, T registerValue)
    {
        size_t registerValueSize = sizeof(T);
        checkedTLCall(
            GenTL::GCWritePort,
            "Failed to set register with RegisterAddress " + intToHex(iAddress),
            remDevHandle,
            iAddress,
            &registerValue,
            &registerValueSize);
    }

    double brightnessFromCameraFamily(GenTL::PORT_HANDLE remDevHandle)
    {
        std::array<char, 1024> modelNameBuffer{};
        size_t modelNameBufferSize = sizeof(modelNameBuffer);
        checkedTLCall(
            GenTL::GCReadPort,
            "Failed to read device model name.",
            remDevHandle,
            DeviceControl::deviceModelName,
            &modelNameBuffer,
            &modelNameBufferSize);

        auto modelName = static_cast<std::string>(modelNameBuffer.data());
        if(modelName.find("Zivid 2+") != std::string::npos && modelName.find('R') == std::string::npos)
        {
            // Zivid 2+ M130/L110/M60 needs additional configuration to go above 2.2
            // See https://support.zivid.com/en/latest/api-reference/zivid-config-files.html
            return 2.2;
        }
        if(modelName.find("Zivid 2") != std::string::npos)
        {
            return 1.8;
        }
        return 2.5;
    }

    template<typename T>
    void readBufferPartInfo(
        GenTL::DS_HANDLE dsHandle,
        GenTL::BUFFER_HANDLE bufferHandle,
        uint64_t partIndex,
        GenTL::BUFFER_PART_INFO_CMD infoCmd,
        T *infoBuffer)
    {
        GenTL::INFO_DATATYPE dataType = GenTL::INFO_DATATYPE_UNKNOWN;
        size_t infoBufferSize = sizeof(T);
        checkedTLCall(
            GenTL::DSGetBufferPartInfo,
            "Failed to get buffer part info.",
            dsHandle,
            bufferHandle,
            partIndex,
            infoCmd,
            &dataType,
            infoBuffer,
            &infoBufferSize);
    }

    std::vector<std::vector<PointXYZ>> copyXYZ(GenTL::DS_HANDLE dsHandle, GenTL::BUFFER_HANDLE bufferHandle)
    {
        void *baseAddress = nullptr;
        size_t width = 0;
        size_t height = 0;
        uint64_t xyzIndex = 0;

        readBufferPartInfo(dsHandle, bufferHandle, xyzIndex, GenTL::BUFFER_PART_INFO_BASE, &baseAddress);
        readBufferPartInfo(dsHandle, bufferHandle, xyzIndex, GenTL::BUFFER_PART_INFO_WIDTH, &width);
        readBufferPartInfo(dsHandle, bufferHandle, xyzIndex, GenTL::BUFFER_PART_INFO_HEIGHT, &height);

        auto buffer = static_cast<PointXYZ *>(baseAddress);

        std::vector<std::vector<PointXYZ>> xyz(height, std::vector<PointXYZ>(width));
        for(size_t row = 0; row < height; row++)
        {
            for(size_t column = 0; column < width; column++)
            {
                xyz[row][column].x = buffer->x;
                xyz[row][column].y = buffer->y;
                xyz[row][column].z = buffer->z;

                buffer++;
            }
        }

        return xyz;
    }

    std::vector<std::vector<float>> copyConfidence(GenTL::DS_HANDLE dsHandle, GenTL::BUFFER_HANDLE bufferHandle)
    {
        void *baseAddress = nullptr;
        size_t width = 0;
        size_t height = 0;
        uint64_t confidenceIndex = 1;

        readBufferPartInfo(dsHandle, bufferHandle, confidenceIndex, GenTL::BUFFER_PART_INFO_BASE, &baseAddress);
        readBufferPartInfo(dsHandle, bufferHandle, confidenceIndex, GenTL::BUFFER_PART_INFO_WIDTH, &width);
        readBufferPartInfo(dsHandle, bufferHandle, confidenceIndex, GenTL::BUFFER_PART_INFO_HEIGHT, &height);

        auto buffer = static_cast<float *>(baseAddress);

        std::vector<std::vector<float>> confidence(height, std::vector<float>(width));
        for(size_t row = 0; row < height; row++)
        {
            for(size_t column = 0; column < width; column++)
            {
                confidence[row][column] = *buffer;

                buffer++;
            }
        }

        return confidence;
    }

    std::vector<std::vector<RGBA>> copyRGBA(GenTL::DS_HANDLE dsHandle, GenTL::BUFFER_HANDLE bufferHandle)
    {
        void *baseAddress = nullptr;
        size_t width = 0;
        size_t height = 0;
        uint64_t rgbaIndex = 2;

        readBufferPartInfo(dsHandle, bufferHandle, rgbaIndex, GenTL::BUFFER_PART_INFO_BASE, &baseAddress);
        readBufferPartInfo(dsHandle, bufferHandle, rgbaIndex, GenTL::BUFFER_PART_INFO_WIDTH, &width);
        readBufferPartInfo(dsHandle, bufferHandle, rgbaIndex, GenTL::BUFFER_PART_INFO_HEIGHT, &height);

        auto buffer = static_cast<uint32_t *>(baseAddress);

        std::vector<std::vector<RGBA>> rgba(height, std::vector<RGBA>(width));
        for(size_t row = 0; row < height; row++)
        {
            for(size_t column = 0; column < width; column++)
            {
                rgba[row][column].a = (*buffer >> 24) & 0xFF;
                rgba[row][column].r = (*buffer >> 16) & 0xFF;
                rgba[row][column].g = (*buffer >> 8) & 0xFF;
                rgba[row][column].b = (*buffer >> 0) & 0xFF;

                buffer++;
            }
        }

        return rgba;
    }
} // namespace

int main()
{
    try
    {
        std::cout << "Initializing GenTL interface" << std::endl;
        checkedTLCall(GenTL::GCInitLib, "Failed to initialize GenTL library.");
        GenTL::TL_HANDLE tlHandle{};
        checkedTLCall(GenTL::TLOpen, "Failed initializing transport layer.", &tlHandle);

        bool8_t changedInterfaceList{ false };
        uint64_t timeout{ 10 };
        uint32_t numInterfaces{ 0 };
        checkedTLCall(
            GenTL::TLUpdateInterfaceList, "Failed updating interface list.", tlHandle, &changedInterfaceList, timeout);
        checkedTLCall(GenTL::TLGetNumInterfaces, "Failed getting number of interfaces.", tlHandle, &numInterfaces);

        std::array<char, 100> idBuffer{};
        size_t idBufferSize{ sizeof(idBuffer) };
        GenTL::IF_HANDLE ifHandle{};
        checkedTLCall(
            GenTL::TLGetInterfaceID, "Failed getting interface IDs.", tlHandle, 0, idBuffer.data(), &idBufferSize);
        checkedTLCall(GenTL::TLOpenInterface, "Failed opening interface.", tlHandle, idBuffer.data(), &ifHandle);

        std::cout << "Connecting to camera" << std::endl;
        uint64_t deviceTimeout{ 100 };
        bool8_t changedDeviceList{ false };
        uint32_t numDevices{ 0 };
        checkedTLCall(
            GenTL::IFUpdateDeviceList, "Failed updating device list.", ifHandle, &changedDeviceList, deviceTimeout);
        checkedTLCall(GenTL::IFGetNumDevices, "Failed getting number of devices.", ifHandle, &numDevices);
        if(!changedDeviceList || numDevices <= 0)
        {
            throw std::runtime_error{ "No camera(s) found." };
        }

        GenTL::DEVICE_ACCESS_STATUS deviceAccessFlags{ GenTL::DEVICE_ACCESS_EXCLUSIVE };
        std::array<char, 100> deviceIDBuffer{};
        size_t deviceIDBufferSize{ sizeof(deviceIDBuffer) };
        GenTL::DEV_HANDLE devHandle{};
        GenTL::PORT_HANDLE remDevHandle{};
        checkedTLCall(
            GenTL::IFGetDeviceID, "Failed getting device ID.", ifHandle, 0, deviceIDBuffer.data(), &deviceIDBufferSize);
        checkedTLCall(
            GenTL::IFOpenDevice,
            "Failed opening device.",
            ifHandle,
            deviceIDBuffer.data(),
            deviceAccessFlags,
            &devHandle);
        checkedTLCall(GenTL::DevGetPort, "Failed opening remote device.", devHandle, &remDevHandle);

        setZividRegister(remDevHandle, MultiAcquisitionFrameControl::resetAcquisitions, 1);

        std::cout << "Setting up HDR settings" << std::endl;
        std::vector<double> apertures = { 11.31, 5.66, 2.83 };
        for(double aperture : apertures)
        {
            setZividRegister(remDevHandle, AcquisitionSettingsControl::aperture, aperture);
            setZividRegister(
                remDevHandle, AcquisitionSettingsControl::brightness, brightnessFromCameraFamily(remDevHandle));
            setZividRegister(remDevHandle, MultiAcquisitionFrameControl::addAcquisition, 1);
        }

        // Showing all available capture settings
        setZividRegister(remDevHandle, SettingsControl::engine, Zivid::Settings::Engine::phase.value());
        setZividRegister(remDevHandle, SettingsControl::samplingColor, Zivid::Settings::Sampling::Color::rgb.value());
        setZividRegister(remDevHandle, SettingsControl::samplingPixel, Zivid::Settings::Sampling::Pixel::all.value());
        setZividRegister(remDevHandle, SettingsControl::regionOfInterestBoxEnabled, true);
        setZividRegister(remDevHandle, SettingsControl::regionOfInterestBoxPointOX, 1000.0F);
        setZividRegister(remDevHandle, SettingsControl::regionOfInterestBoxPointOY, 1000.0F);
        setZividRegister(remDevHandle, SettingsControl::regionOfInterestBoxPointOZ, 1000.0F);
        setZividRegister(remDevHandle, SettingsControl::regionOfInterestBoxPointAX, 1000.0F);
        setZividRegister(remDevHandle, SettingsControl::regionOfInterestBoxPointAY, -1000.0F);
        setZividRegister(remDevHandle, SettingsControl::regionOfInterestBoxPointAZ, 1000.0F);
        setZividRegister(remDevHandle, SettingsControl::regionOfInterestBoxPointBX, -1000.0F);
        setZividRegister(remDevHandle, SettingsControl::regionOfInterestBoxPointBY, 1000.0F);
        setZividRegister(remDevHandle, SettingsControl::regionOfInterestBoxPointBZ, 1000.0F);
        setZividRegister(remDevHandle, SettingsControl::regionOfInterestBoxExtentsMin, -1000.0);
        setZividRegister(remDevHandle, SettingsControl::regionOfInterestBoxExtentsMax, 1000.0);
        setZividRegister(remDevHandle, SettingsControl::regionOfInterestDepthEnabled, true);
        setZividRegister(remDevHandle, SettingsControl::regionOfInterestDepthRangeMin, 200.0);
        setZividRegister(remDevHandle, SettingsControl::regionOfInterestDepthRangeMax, 2000.0);
        setZividRegister(remDevHandle, SettingsControl::processingFiltersSmoothingGaussianEnabled, true);
        setZividRegister(remDevHandle, SettingsControl::processingFiltersSmoothingGaussianSigma, 1.5);
        setZividRegister(remDevHandle, SettingsControl::processingFiltersNoiseRemovalEnabled, true);
        setZividRegister(remDevHandle, SettingsControl::processingFiltersNoiseRemovalThreshold, 7.0);
        setZividRegister(remDevHandle, SettingsControl::processingFiltersNoiseSuppressionEnabled, true);
        setZividRegister(remDevHandle, SettingsControl::processingFiltersNoiseRepairEnabled, true);
        setZividRegister(remDevHandle, SettingsControl::processingFiltersOutlierRemovalEnabled, true);
        setZividRegister(remDevHandle, SettingsControl::processingFiltersOutlierRemovalThreshold, 5.0);
        setZividRegister(remDevHandle, SettingsControl::processingFiltersReflectionRemovalEnabled, true);
        setZividRegister(
            remDevHandle,
            SettingsControl::processingFiltersReflectionRemovalMode,
            Zivid::Settings::Processing::Filters::Reflection::Removal::Mode::global.value());
        setZividRegister(remDevHandle, SettingsControl::processingFiltersClusterRemovalEnabled, true);
        setZividRegister(remDevHandle, SettingsControl::processingFiltersClusterRemovalMaxNeighborDistance, 10.0);
        setZividRegister(remDevHandle, SettingsControl::processingFiltersClusterRemovalMinArea, 100.0);
        setZividRegister(
            remDevHandle, SettingsControl::processingFiltersExperimentalContrastDistortionCorrectionEnabled, true);
        setZividRegister(
            remDevHandle, SettingsControl::processingFiltersExperimentalContrastDistortionCorrectionStrength, 0.4);
        setZividRegister(
            remDevHandle, SettingsControl::processingFiltersExperimentalContrastDistortionRemovalEnabled, false);
        setZividRegister(
            remDevHandle, SettingsControl::processingFiltersExperimentalContrastDistortionRemovalThreshold, 0.5);
        setZividRegister(remDevHandle, SettingsControl::processingFiltersHoleRepairEnabled, true);
        setZividRegister(remDevHandle, SettingsControl::processingFiltersHoleRepairHoleSize, 0.2);
        setZividRegister(remDevHandle, SettingsControl::processingFiltersHoleRepairStrictness, 1);
        setZividRegister(
            remDevHandle,
            SettingsControl::processingResamplingMode,
            Zivid::Settings::Processing::Resampling::Mode::disabled.value());
        setZividRegister(remDevHandle, SettingsControl::processingColorBalanceRed, 1.0);
        setZividRegister(remDevHandle, SettingsControl::processingColorBalanceGreen, 1.0);
        setZividRegister(remDevHandle, SettingsControl::processingColorBalanceBlue, 1.0);
        setZividRegister(remDevHandle, SettingsControl::processingColorGamma, 1.0);
        setZividRegister(
            remDevHandle,
            SettingsControl::processingColorExperimentalMode,
            Zivid::Settings::Processing::Color::Experimental::Mode::automatic.value());

        std::array<char, 1024> dataStreamIdBuffer{};
        size_t dataStreamBufferSize = sizeof(dataStreamIdBuffer);
        GenTL::DS_HANDLE dsHandle{};
        checkedTLCall(
            GenTL::DevGetDataStreamID,
            "Failed getting datastream ID.",
            devHandle,
            0,
            dataStreamIdBuffer.data(),
            &dataStreamBufferSize);
        checkedTLCall(
            GenTL::DevOpenDataStream, "Failed opening datastream.", devHandle, dataStreamIdBuffer.data(), &dsHandle);

        uint32_t payloadSize = 0;
        size_t payloadSizeSize = sizeof(payloadSize);
        checkedTLCall(
            GenTL::GCReadPort,
            "Failed to read payload size.",
            remDevHandle,
            TransportLayerControl::payloadSize,
            &payloadSize,
            &payloadSizeSize);

        GenTL::BUFFER_HANDLE bufferHandle{ GENTL_INVALID_HANDLE };
        std::array<char, 1024> privateBufferData{};
        checkedTLCall(
            GenTL::DSAllocAndAnnounceBuffer,
            "Failed announcing and allocating buffers. Error code: ",
            dsHandle,
            payloadSize,
            &privateBufferData,
            &bufferHandle);
        checkedTLCall(GenTL::DSQueueBuffer, "Failed to queue buffer.", dsHandle, bufferHandle);

        GenTL::EVENT_HANDLE newBufferEvent{ GENTL_INVALID_HANDLE };
        checkedTLCall(
            GenTL::GCRegisterEvent, "Failed to register event.", dsHandle, GenTL::EVENT_NEW_BUFFER, &newBufferEvent);

        uint64_t numImages{ 1 };
        setZividRegister(remDevHandle, AcquisitionControl::acquisitionMode, apertures.size());
        checkedTLCall(
            GenTL::DSStartAcquisition,
            "Failed to start acquisition loop.",
            dsHandle,
            GenTL::ACQ_START_FLAGS_DEFAULT,
            numImages);

        std::cout << "Capturing MultiAcquisitionFrame" << std::endl;
        setZividRegister(remDevHandle, AcquisitionControl::acquisitionStart, true);

        GenTL::EVENT_NEW_BUFFER_DATA newBufferData{};
        size_t newBufferDataSize = sizeof(newBufferData);
        // First capture can be potentially slow because GPU kernels get compiled
        auto captureTimeout = std::chrono::seconds{ 100 };
        checkedTLCall(
            GenTL::EventGetData,
            "Failed to capture.",
            newBufferEvent,
            &newBufferData,
            &newBufferDataSize,
            std::chrono::milliseconds(captureTimeout).count());

        setZividRegister(remDevHandle, AcquisitionControl::acquisitionStop, true);
        checkedTLCall(GenTL::DSStopAcquisition, "Failed to stop acquisition.", dsHandle, GenTL::ACQ_STOP_FLAGS_DEFAULT);

        const auto xyz = copyXYZ(dsHandle, bufferHandle);
        const auto confidence = copyConfidence(dsHandle, bufferHandle);
        const auto rgba = copyRGBA(dsHandle, bufferHandle);

        std::cout << "Values in the central pixel:" << std::endl;
        int row = xyz.size() / 2;
        int col = xyz[0].size() / 2;
        std::cout << "  XYZ: " << std::fixed << std::setprecision(2) << "[" << xyz[row][col].x << ", "
                  << xyz[row][col].y << ", " << xyz[row][col].z << "]" << std::endl;
        std::cout << "  RGB: " << std::dec << "[" << static_cast<int>(rgba[row][col].r) << ", "
                  << static_cast<int>(rgba[row][col].g) << ", " << static_cast<int>(rgba[row][col].b) << "]"
                  << std::endl;
        std::cout << "  Confidence: " << std::fixed << std::setprecision(2) << confidence[row][col] << std::endl;

        checkedTLCall(GenTL::DSFlushQueue, "Failed to flush queue.", dsHandle, GenTL::ACQ_QUEUE_ALL_DISCARD);
        checkedTLCall(GenTL::DSRevokeBuffer, "Failed to revoke buffer.", dsHandle, bufferHandle, nullptr, nullptr);

        checkedTLCall(GenTL::DSClose, "Failed closing datastream.", dsHandle);
        checkedTLCall(GenTL::DevClose, "Failed closing device.", devHandle);
        checkedTLCall(GenTL::IFClose, "Failed closing interface.", ifHandle);
        checkedTLCall(GenTL::TLClose, "Failed closing transport layer.", tlHandle);
        checkedTLCall(GenTL::GCCloseLib, "Failed uninitializing GenTL library.");
    }
    catch(const std::runtime_error &e)
    {
        std::cerr << "Error: " << Zivid::toString(e) << std::endl;
        std::cout << "Press enter to exit." << std::endl;
        std::cin.get();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
