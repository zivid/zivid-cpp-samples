project(ZividOpenCVSamples)

cmake_minimum_required(VERSION 3.3.2 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 11)

find_package(Zivid COMPONENTS Core REQUIRED)
find_package(OpenCV 4.0.1 COMPONENTS core highgui REQUIRED )

add_custom_target(
    SampleDependencies
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            $ENV{ZIVID_INSTALL_FOLDER}/$<$<CONFIG:Debug>:bin_debug>$<$<CONFIG:Release>:bin>/plugins/
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_CFG_INTDIR}/plugins
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_SOURCE_DIR}/../Zivid3D.zdf
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_CFG_INTDIR}
)

IF( NOT OpenCV_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable OpenCV_DIR to the include directory of your OpenCV installation.")
ENDIF()
INCLUDE_DIRECTORIES ( "${OpenCV_DIR}" )

set(SAMPLE_NAMES ZDF2OpenCV)

foreach(SAMPLE_NAME ${SAMPLE_NAMES})
    
	add_executable(${SAMPLE_NAME} ${SAMPLE_NAME}/${SAMPLE_NAME}.cpp)
	
	target_link_libraries(${SAMPLE_NAME}
		Zivid::Core
		Zivid::Vis3D
		${OpenCV_LIBS}
	)
	
	add_dependencies(${SAMPLE_NAME} SampleDependencies)

endforeach()