set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)

option(WARNINGS "Enable compiler warnings" OFF)

if(WARNINGS)
    option(WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    if(WARNINGS)
        if(WARNINGS_AS_ERRORS)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
        endif()

        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Weverything")
        set(WARNINGS_THAT_SHOULD_BE_IGNORED # WHY it is ok to ignore
            c++98-compat                    # Code base should be modern
            c++98-compat-pedantic           # Code base should be modern
            newline-eof                     # Legacy warning, no benefit when using modern compilers and editors
        )
        foreach(WARNING ${WARNINGS_THAT_SHOULD_BE_IGNORED})
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-${WARNING}")
        endforeach()
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
    endif()
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    if(WARNINGS)
        if(WARNINGS_AS_ERRORS)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
        endif()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
        set(WARNINGS_THAT_SHOULD_BE_IGNORED # WHY it is ok to ignore
        #TODO, see the Clang options
        )
        foreach(WARNING ${WARNINGS_THAT_SHOULD_BE_IGNORED})
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-${WARNING}")
        endforeach()
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
    endif()
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    if(WARNINGS)
        if(WARNINGS_AS_ERRORS)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /WX")
        endif()

        set(WARNINGS_THAT_SHOULD_BE_IGNORED # WHY it is ok to ignore
        #TODO, see the Clang options
        )
        foreach(WARNING ${WARNINGS_THAT_SHOULD_BE_IGNORED})
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd${WARNING}")
        endforeach()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Wall")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /experimental:external /external:anglebrackets /external:W0")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /w")
    endif()
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /ignore:4099")
else()
    message(WARNING "Unknown compiler, not able to set compiler options for ${CMAKE_CXX_COMPILER_ID}")
endif()
