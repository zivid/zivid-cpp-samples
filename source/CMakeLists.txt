cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(CppExtraSamples)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(CompilerOptions)

set(ZIVID_VERSION 1.8.0)

option(USE_EIGEN3 "Enable samples which depend on Eigen 3" ON)
option(USE_OPENCV "Enable samples which depend on OpenCV" ON)
option(USE_PCL "Enable samples which depend on Point Cloud Library (PCL)" ON)

set(SAMPLES
    Camera/Basic/Capture
    Camera/Basic/Capture2D
    Camera/Basic/CaptureAssistant
    Camera/Basic/CaptureFromFile
    Camera/Basic/CaptureHDR
    Camera/Basic/CaptureHDRLoop
    Camera/InfoUtilOther/CameraUserData
    Camera/InfoUtilOther/GetCameraIntrinsics
    Applications/Basic/Visualization/CaptureFromFileVis3D
    Applications/Basic/Visualization/CaptureLiveVis3D
    Applications/Basic/Visualization/CaptureVis3D
    Applications/Basic/Visualization/ReadPCLVis3D
    Applications/Basic/Visualization/CaptureWritePCLVis3D
    Applications/Basic/FileFormats/ReadIterateZDF
    Applications/Advanced/CaptureUndistortRGB
    Applications/Advanced/Downsample
    Applications/Advanced/HandEyeCalibration/HandEyeCalibration
    Applications/Advanced/HandEyeCalibration/UtilizeEyeInHandCalibration
    Applications/Advanced/HandEyeCalibration/PoseConversions)

set(Eigen3_DEPENDING Downsample UtilizeEyeInHandCalibration PoseConversions)
set(PCL_DEPENDING ReadPCLVis3D CaptureWritePCLVis3D CaptureFromFileWritePCLVis3D ZDF2PCD)
set(OpenCV_DEPENDING ZDF2OpenCV CaptureUndistortRGB UtilizeEyeInHandCalibration PoseConversions)
set(Vis3D_DEPENDING CaptureVis3D CaptureLiveVis3D CaptureFromFileVis3D Downsample CaptureFromFileWritePCLVis3D CaptureWritePCLVis3D ZDF2OpenCV ReadIterateZDF CaptureUndistortRGB)

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
    find_package(OpenCV 4.0.1 COMPONENTS core highgui calib3d)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV not found. Please point OpenCV_DIR to the directory of your OpenCV installation (containing the file OpenCVConfig.cmake), or disable the OpenCV samples  with -DUSE_OPENCV=OFF.")
    endif()
else()
    disable_samples("OpenCV")
endif()

add_custom_target(
    CopyZdf
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_SOURCE_DIR}/Zivid3D.zdf
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_CFG_INTDIR}
)

if(WIN32)
    add_custom_target(
        CopyDlls
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                $ENV{ZIVID_INSTALL_FOLDER}/$<$<CONFIG:Debug>:bin_debug>$<$<CONFIG:Release>:bin>/plugins/
                ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_CFG_INTDIR}/plugins
    )
endif()

message(STATUS "All samples: ${SAMPLES}")

foreach(SAMPLE ${SAMPLES})
    get_filename_component(SAMPLE_NAME ${SAMPLE} NAME)

    add_executable(${SAMPLE_NAME} ${SAMPLE}/${SAMPLE_NAME}.cpp)

    target_link_libraries(${SAMPLE_NAME} Zivid::Core)

    if(${SAMPLE_NAME} IN_LIST Vis3D_DEPENDING)
        target_link_libraries(${SAMPLE_NAME} Zivid::Vis3D)
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

    add_dependencies(${SAMPLE_NAME} CopyZdf)

    if(WIN32)
        add_dependencies(${SAMPLE_NAME} CopyDlls)
    endif()
endforeach()

# TODO: Generalize how input file dependencies are copied, see issue #46
add_custom_target(
    CopyHandEyeFiles
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_SOURCE_DIR}/Applications/Advanced/HandEyeCalibration/robotTransform.yaml
            ${CMAKE_CURRENT_SOURCE_DIR}/Applications/Advanced/HandEyeCalibration/handEyeTransform.yaml
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_CFG_INTDIR}
)
if(TARGET UtilizeEyeInHandCalibration)
    add_dependencies("UtilizeEyeInHandCalibration" CopyHandEyeFiles)
endif()
if(TARGET PoseConversions)
    add_dependencies("PoseConversions" CopyHandEyeFiles)
endif()
