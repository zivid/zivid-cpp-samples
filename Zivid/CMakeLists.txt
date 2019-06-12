project(ZividExtraSamples)

cmake_minimum_required(VERSION 3.3.2 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 11)

find_package(Zivid COMPONENTS Core 1.2.1 REQUIRED)
    
set(SAMPLE_NAMES CaptureHDRCompleteSettings CaptureHDRLoop CaptureSavePLY ConnectToSerialNumberCamera GetCameraIntrinsics ReadZDF ZDF2PLY)

foreach(SAMPLE_NAME ${SAMPLE_NAMES})

	add_executable(${SAMPLE_NAME} ${SAMPLE_NAME}/${SAMPLE_NAME}.cpp)

	target_link_libraries(${SAMPLE_NAME}
		Zivid::Core
		Zivid::Vis3D
	)

endforeach()

get_filename_component(ONE_ABOVE_CURRENT_SOURCE_DIR ../ ABSOLUTE)

file(COPY ${ONE_ABOVE_CURRENT_SOURCE_DIR}/Zivid3D.zdf DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${ONE_ABOVE_CURRENT_SOURCE_DIR}/Zivid3D.zdf DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Release)
file(COPY ${ONE_ABOVE_CURRENT_SOURCE_DIR}/Zivid3D.zdf DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Debug)

file(COPY $ENV{ZIVID_INSTALL_FOLDER}/bin_debug/plugins/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Debug/plugins/)
file(COPY $ENV{ZIVID_INSTALL_FOLDER}/bin/plugins/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/Release/plugins/)