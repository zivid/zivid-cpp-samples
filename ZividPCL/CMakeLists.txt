project(ZividPCLSamples)

cmake_minimum_required(VERSION 3.3.2 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 11)

find_package(Zivid COMPONENTS Core 1.2.1 REQUIRED)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
    
set(SAMPLE_VIS_NAMES ReadPCLVis3D CaptureWritePCLVis3D CaptureFromFileWritePCLVis3D ZDF2PCD)

foreach(SAMPLE_NAME ${SAMPLE_VIS_NAMES})

	add_executable(${SAMPLE_NAME} ${SAMPLE_NAME}/${SAMPLE_NAME}.cpp)

	target_link_libraries(${SAMPLE_NAME}
		Zivid::Core
		Zivid::Vis3D
		${PCL_LIBRARIES}
	)

endforeach()

get_filename_component(ONE_ABOVE_CURRENT_SOURCE_DIR ../ ABSOLUTE)

file(COPY ${ONE_ABOVE_CURRENT_SOURCE_DIR}/Zivid3D.zdf DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${ONE_ABOVE_CURRENT_SOURCE_DIR}/Zivid3D.zdf DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Release)
file(COPY ${ONE_ABOVE_CURRENT_SOURCE_DIR}/Zivid3D.zdf DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Debug)

file(COPY $ENV{ZIVID_INSTALL_FOLDER}/bin_debug/plugins/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Debug/plugins/)
file(COPY $ENV{ZIVID_INSTALL_FOLDER}/bin/plugins/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Release/plugins/)