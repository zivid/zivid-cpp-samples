cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(CppExtraSamples)

set(CMAKE_CXX_STANDARD 11)
set(ZIVID_VERSION 1.5.0)

option(USE_EIGEN3 "Enable samples which depend on Eigen 3" ON)
option(USE_OPENCV "Enable samples which depend on OpenCV" ON)
option(USE_PCL "Enable samples which depend on Point Cloud Library (PCL)" ON)
option(USE_VIS3D "Enable samples which depend on the Zivid Cloud Visualizer. These can be hard to get to build on Linux due to a specific Qt dependency" ON)

find_package(Zivid ${ZIVID_VERSION} COMPONENTS Core REQUIRED)
find_package(PCL 1.2 QUIET)
find_package(OpenCV 4.0.1 QUIET OPTIONAL_COMPONENTS core highgui)

set(SAMPLES
    Zivid/CaptureHDRCompleteSettings
    Zivid/CaptureHDRLoop
    Zivid/CaptureSavePLY
    Zivid/ConnectToSerialNumberCamera
    Zivid/GetCameraIntrinsics
    Zivid/ReadZDF
    Zivid/ZDF2PLY
    ZividPCL/ReadPCLVis3D
    ZividPCL/CaptureWritePCLVis3D
    ZividPCL/CaptureFromFileWritePCLVis3D
    ZividPCL/ZDF2PCD
    ZividOpenCV/ZDF2OpenCV
    ZividOpenCV/CaptureUndistortRGB
    Downsample)

set(Eigen3_DEPENDING Downsample)
set(PCL_DEPENDING ReadPCLVis3D CaptureWritePCLVis3D CaptureFromFileWritePCLVis3D ZDF2PCD)
set(OpenCV_DEPENDING ZDF2OpenCV CaptureUndistortRGB)
set(Vis3D_DEPENDING Downsample CaptureFromFileWritePCLVis3D CaptureWritePCLVis3D ZDF2OpenCV CaptureHDRLoop CaptureHDRCompleteSettings ReadZDF CaptureUndistortRGB)

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
    if(NOT PCL_FOUND)
        message(FATAL_ERROR "Point Cloud Library (PCL) not found. Please point PCL_DIR to the directory of your PCL installation (containing the file PCLConfig.cmake), or disable the PCL samples with -DUSE_PCL=OFF.")
    endif()
else()
    disable_samples("PCL")
endif()

if(USE_OPENCV)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV not found. Please point OpenCV_DIR to the directory of your OpenCV installation (containing the file OpenCVConfig.cmake), or disable the OpenCV samples  with -DUSE_OPENCV=OFF.")
    endif()
else()
    disable_samples("OpenCV")
endif()

if(NOT USE_VIS3D)
    disable_samples("Vis3D")
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
        target_include_directories(${SAMPLE_NAME} PRIVATE ${EIGEN3_INCLUDE_DIR})
    endif()

    if(${SAMPLE_NAME} IN_LIST PCL_DEPENDING)
        target_link_libraries(${SAMPLE_NAME} ${PCL_LIBRARIES})
        target_include_directories(${SAMPLE_NAME} PRIVATE ${PCL_INCLUDE_DIRS})
    endif()

    if(${SAMPLE_NAME} IN_LIST OpenCV_DEPENDING)
        target_link_libraries(${SAMPLE_NAME} ${OpenCV_LIBS})
    endif()

    add_dependencies(${SAMPLE_NAME} CopyZdf)
    if(WIN32)
        add_dependencies(${SAMPLE_NAME} CopyDlls)
    endif()
endforeach()
