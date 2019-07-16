project(ZividPCLSamples)

cmake_minimum_required(VERSION 3.3.2 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 11)

find_package(Zivid COMPONENTS Core REQUIRED)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_custom_target(
    SampleDependencies
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            $ENV{ZIVID_INSTALL_FOLDER}/$<$<CONFIG:Debug>:bin_debug>$<$<CONFIG:Release>:bin>/plugins/
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_CFG_INTDIR}/plugins
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_SOURCE_DIR}/../Zivid3D.zdf
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_CFG_INTDIR}
)
    
set(SAMPLE_NAMES ReadPCLVis3D CaptureWritePCLVis3D CaptureFromFileWritePCLVis3D ZDF2PCD)

foreach(SAMPLE_NAME ${SAMPLE_NAMES})

	add_executable(${SAMPLE_NAME} ${SAMPLE_NAME}/${SAMPLE_NAME}.cpp)

	target_link_libraries(${SAMPLE_NAME}
		Zivid::Core
		Zivid::Vis3D
		${PCL_LIBRARIES}
	)
	
	add_dependencies(${SAMPLE_NAME} SampleDependencies)

endforeach()