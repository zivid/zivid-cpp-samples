project(Downsample)

cmake_minimum_required(VERSION 3.3.2 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 11)

find_package(Zivid COMPONENTS Core REQUIRED)

add_custom_target(
    SampleDependencies
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            $ENV{ZIVID_INSTALL_FOLDER}/$<$<CONFIG:Debug>:bin_debug>$<$<CONFIG:Release>:bin>/plugins/
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_CFG_INTDIR}/plugins
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_SOURCE_DIR}/../Zivid3D.zdf
            ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_CFG_INTDIR}
)

IF( NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ENDIF()
INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )

    
add_executable(Downsample Downsample.cpp)

add_dependencies(Downsample SampleDependencies)

target_link_libraries(Downsample
	Zivid::Core
	Zivid::Vis3D
)

