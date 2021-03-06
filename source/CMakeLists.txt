cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(ZividCppSamples)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(CompilerOptions)

set(ZIVID_VERSION 2.2.0)

option(USE_EIGEN3 "Enable samples which depend on Eigen 3" ON)
option(USE_OPENCV "Enable samples which depend on OpenCV" ON)
option(USE_PCL "Enable samples which depend on Point Cloud Library (PCL)" ON)

set(SAMPLES
    Camera/Basic/Capture
    Camera/Basic/Capture2D
    Camera/Basic/CaptureAssistant
    Camera/Basic/CaptureFromFileCamera
    Camera/Basic/CaptureWithSettingsFromYML
    Camera/Basic/CaptureHDR
    Camera/Basic/CaptureHDRCompleteSettings
    Camera/Advanced/CaptureHDRLoop
    Camera/InfoUtilOther/CameraUserData
    Camera/InfoUtilOther/GetCameraIntrinsics
    Camera/InfoUtilOther/FirmwareUpdater
    Camera/InfoUtilOther/ZividBenchmark
    Camera/Maintenance/VerifyCameraInField
    Camera/Maintenance/CorrectCameraInField
    Applications/Basic/Visualization/CaptureFromFileCameraVis3D
    Applications/Basic/Visualization/CaptureVis3D
    Applications/Basic/Visualization/ReadPCLVis3D
    Applications/Basic/Visualization/CaptureWritePCLVis3D
    Applications/Basic/FileFormats/ReadIterateZDF
    Applications/Advanced/CaptureUndistortRGB
    Applications/Advanced/Downsample
    Applications/Advanced/MaskPointCloud
    Applications/Advanced/HandEyeCalibration/HandEyeCalibration
    Applications/Advanced/HandEyeCalibration/UtilizeEyeInHandCalibration
    Applications/Advanced/HandEyeCalibration/PoseConversions
    Applications/Advanced/CreateDepthMap
    Applications/Advanced/MultiCamera/MultiCameraCalibration
    Applications/Advanced/MultiCamera/MultiCameraCalibrationFromZDF
    Applications/Advanced/MultiCamera/StitchByTransformation
    Applications/Advanced/MultiCamera/StitchByTransformationFromZDF)

set(Eigen3_DEPENDING UtilizeEyeInHandCalibration PoseConversions)
set(PCL_DEPENDING MaskPointCloud ReadPCLVis3D CaptureWritePCLVis3D StitchByTransformation StitchByTransformationFromZDF)
set(OpenCV_DEPENDING CaptureUndistortRGB UtilizeEyeInHandCalibration PoseConversions CreateDepthMap MaskPointCloud MultiCameraCalibration MultiCameraCalibrationFromZDF StitchByTransformation StitchByTransformationFromZDF)
set(Visualization_DEPENDING CaptureVis3D CaptureLiveVis3D CaptureFromFileCameraVis3D Downsample CaptureWritePCLVis3D CaptureUndistortRGB CreateDepthMap)
set(Clipp_DEPENDING CameraUserData MultiCameraCalibrationFromZDF StitchByTransformation StitchByTransformationFromZDF)

find_package(Zivid ${ZIVID_VERSION} COMPONENTS Core REQUIRED)

macro(disable_samples DEPENDENCY_NAME)
    message("${DEPENDENCY_NAME} samples have been disabled:")
    foreach(SAMPLE ${SAMPLES})
        get_filename_component(SAMPLE_NAME ${SAMPLE} NAME)
        set(DEPENDENCY_LIST "${DEPENDENCY_NAME}_DEPENDING")
        if(${SAMPLE_NAME} IN_LIST ${DEPENDENCY_LIST})
            message("  - ${SAMPLE}")
            list(REMOVE_ITEM SAMPLES ${SAMPLE})
        endif()
    endforeach()
endmacro()

if(USE_EIGEN3)
    if(NOT DEFINED EIGEN3_INCLUDE_DIR)
        message(FATAL_ERROR "Please point EIGEN3_INCLUDE_DIR to the root directory of your Eigen3 installation (the folder containing Eigen/Core, Eigen/Dense etc.), or disable the Eigen samples (-DUSE_EIGEN3=OFF).")
    else()
        if(NOT IS_DIRECTORY "${EIGEN3_INCLUDE_DIR}/Eigen")
            message(FATAL_ERROR "EIGEN3_INCLUDE_DIR was set to ${EIGEN3_INCLUDE_DIR}, which is not a valid Eigen 3 directory. Please correct this, or disable the Eigen samples (-DUSE_EIGEN3=OFF).")
        endif()
    endif()
else()
    disable_samples("Eigen3")
endif()

if(USE_PCL)
    find_package(PCL 1.2)
    if(NOT PCL_FOUND)
        message(FATAL_ERROR "Point Cloud Library (PCL) not found. Please point PCL_DIR to the directory of your PCL installation (containing the file PCLConfig.cmake), or disable the PCL samples with -DUSE_PCL=OFF.")
    endif()
else()
    disable_samples("PCL")
endif()

if(USE_OPENCV)
    find_package(OpenCV 4.1.0 COMPONENTS core highgui calib3d)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV not found. Please point OpenCV_DIR to the directory of your OpenCV installation (containing the file OpenCVConfig.cmake), or disable the OpenCV samples  with -DUSE_OPENCV=OFF.")
    endif()
else()
    disable_samples("OpenCV")
endif()

message(STATUS "All samples: ${SAMPLES}")

if(WIN32)
    set(ZIVID_SAMPLE_DATA_DIR "$ENV{PROGRAMDATA}/Zivid")
elseif(UNIX)
    set(ZIVID_SAMPLE_DATA_DIR "/usr/share/Zivid/data")
else()
    message(FATAL_ERROR "Unsupported operating system")
endif()
file(TO_CMAKE_PATH "${ZIVID_SAMPLE_DATA_DIR}" ZIVID_SAMPLE_DATA_DIR)

foreach(SAMPLE ${SAMPLES})
    get_filename_component(SAMPLE_NAME ${SAMPLE} NAME)

    add_executable(${SAMPLE_NAME} ${SAMPLE}/${SAMPLE_NAME}.cpp)

    target_link_libraries(${SAMPLE_NAME} Zivid::Core)
    
    target_compile_definitions(${SAMPLE_NAME} PRIVATE ZIVID_SAMPLE_DATA_DIR="${ZIVID_SAMPLE_DATA_DIR}")

    if(${SAMPLE_NAME} IN_LIST Visualization_DEPENDING)
        target_link_libraries(${SAMPLE_NAME} Zivid::Visualization)
    endif()

    if(${SAMPLE_NAME} IN_LIST Eigen3_DEPENDING)
        target_include_directories(${SAMPLE_NAME} SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
    endif()

    if(${SAMPLE_NAME} IN_LIST PCL_DEPENDING)
        target_link_libraries(${SAMPLE_NAME} ${PCL_LIBRARIES})
        target_include_directories(${SAMPLE_NAME} SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
    endif()

    if(${SAMPLE_NAME} IN_LIST OpenCV_DEPENDING)
        target_link_libraries(${SAMPLE_NAME} ${OpenCV_LIBS})
    endif()

    if(${SAMPLE_NAME} IN_LIST Clipp_DEPENDING)
        target_include_directories(${SAMPLE_NAME} SYSTEM PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/3rd-party/clipp/include)
    endif()

endforeach()

target_compile_definitions(ZividBenchmark PRIVATE OS_NAME=\"${CMAKE_SYSTEM_NAME}\")
