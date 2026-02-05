project(
    CoreCAPI
    LANGUAGES
        C
        CXX
)

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_C_VISIBILITY_PRESET hidden)

include(GenerateExportHeader)

set(SOURCES ZividCapi.cpp)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

generate_export_header(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME} PUBLIC Zivid::Core)

target_include_directories(${PROJECT_NAME} PUBLIC include/)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

set(POSITION_INDEPENDENT_CODE ON)
