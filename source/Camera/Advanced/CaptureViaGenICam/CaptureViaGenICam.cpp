/*
Capture using the GenICam interface.
*/

#include <Zivid/GenTLAddresses.h>
#include <Zivid/Zivid.h>
#include <clipp.h>
#include <string>

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

    std::string settingsFolder(GenTL::PORT_HANDLE remDevHandle)
    {
        auto cameraModelValue = Zivid::CameraInfo::Model{}.value();
        auto cameraModelSize = sizeof(cameraModelValue);
        checkedTLCall(
            GenTL::GCReadPort,
            "Failed to read CameraInfoModel.",
            remDevHandle,
            ZividGenTL::RegisterAddresses::DeviceControl::cameraInfoModel,
            &cameraModelValue,
            &cameraModelSize);

        switch(cameraModelValue)
        {
            case Zivid::CameraInfo::Model::ValueType::zividTwo:
            case Zivid::CameraInfo::Model::ValueType::zividTwoL100: return "zivid2";
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusM130:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusM60:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusL110: return "zivid2Plus";
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusMR130:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusMR60:
            case Zivid::CameraInfo::Model::ValueType::zivid2PlusLR110: return "zivid2Plus/R";
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusSmall:
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusMedium:
            case Zivid::CameraInfo::Model::ValueType::zividOnePlusLarge: break;

            default:
                throw std::runtime_error(
                    "Unhandled enum value '" + Zivid::CameraInfo::Model{ cameraModelValue }.toString() + "'");
        }
        throw std::invalid_argument("Invalid camera model");
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

    void loadSettingsFromFile(GenTL::PORT_HANDLE remDevHandle, const std::string &settingsFile)
    {
        size_t settingsFileSize = settingsFile.size();
        checkedTLCall(
            GenTL::GCWritePort,
            "Failed to load settings from file.",
            remDevHandle,
            ZividGenTL::RegisterAddresses::SettingsControl::loadSettingsFromFile,
            settingsFile.data(),
            &settingsFileSize);
    }
} // namespace

int main(int argc, char *argv[])
{
    try
    {
        std::string settingsPath;
        bool showHelp = false;

        auto cli =
            (clipp::option("-h", "--help").set(showHelp) % "Show help message",
             clipp::option("--settings-path")
                 & clipp::value("path", settingsPath) % "Path to the camera settings YML file");

        if(!clipp::parse(argc, argv, cli) || showHelp)
        {
            auto fmt = clipp::doc_formatting{}.alternatives_min_split_size(1).surround_labels("\"", "\"");
            std::cout << "USAGE:" << std::endl;
            std::cout << clipp::usage_lines(cli, argv[0], fmt) << std::endl;
            std::cout << "OPTIONS:" << std::endl;
            std::cout << clipp::documentation(cli) << std::endl;
            return showHelp ? EXIT_SUCCESS : EXIT_FAILURE;
        }

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
        auto deviceTimeout = std::chrono::seconds{ 100 };
        bool8_t changedDeviceList{ false };
        uint32_t numDevices{ 0 };
        checkedTLCall(
            GenTL::IFUpdateDeviceList,
            "Failed updating device list.",
            ifHandle,
            &changedDeviceList,
            std::chrono::milliseconds(deviceTimeout).count());
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

        if(settingsPath.empty())
        {
            settingsPath =
                std::string(ZIVID_SAMPLE_DATA_DIR) + "/Settings/" + settingsFolder(remDevHandle) + "/Settings01.yml";
        }
        std::cout << "Loading settings from file " << settingsPath << std::endl;
        loadSettingsFromFile(remDevHandle, settingsPath);

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
        checkedTLCall(
            GenTL::DSStartAcquisition,
            "Failed to start acquisition loop.",
            dsHandle,
            GenTL::ACQ_START_FLAGS_DEFAULT,
            numImages);

        std::cout << "Capturing..." << std::endl;
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
