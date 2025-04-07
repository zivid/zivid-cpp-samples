set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

# Warnings are disabled by default to ensure that the samples are as portable as possible across compiler versions. It
# is recommended to enable warnings if you plan to modify the samples or use this project as a basis for your
# development project.
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
            c++98-compat # Code base should be modern
            c++98-compat-pedantic # Code base should be modern
            newline-eof # Legacy warning, no benefit when using modern compilers and editors
            sign-conversion # Happens a lot in these samples, would complicate them too much to handle manually
            sign-compare # Happens a lot in these samples, would complicate them too much to handle manually
            shorten-64-to-32 # Narrowing conversions: Too strict and noisy for this code base
            padded # This indicates unused space: The compiler had to insert padding in a struct for alignment.
            # Implicit conversion loses integer precision (signed to unsigned).
            # Expected to happen a lot in these samples and would complicate them too much to handle manually
            conversion
            double-promotion # We are not concerned about the potential performance hit from this.
            # All values should be explicit handled AND the default case should throw an exception.
            covered-switch-default # We don't want this warning, because we want the default labels for safety.
        )

        if("${CMAKE_CXX_COMPILER_VERSION}" VERSION_LESS_EQUAL 12 AND "${CMAKE_CXX_STANDARD}" EQUAL 20)
            # Bug in Clang when translating operator<() into spaceship operator:
            # https://github.com/llvm/llvm-project/issues/43670
            list(APPEND WARNINGS_THAT_SHOULD_BE_IGNORED zero-as-null-pointer-constant)
        endif()

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
            # TODO, see the Clang options
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
            set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /WX")
        endif()

        set(WARNINGS_THAT_SHOULD_BE_IGNORED # WHY it is ok to ignore
            4244 # Narrowing conversions: Too strict and noisy for this code base.
            4267 # Conversion: Happens a lot in these samples, would complicate them too much to handle manually.
            4702 # Unreachable code: Ignoring because they happen in external headers despite `/external:W0`.
            4996 # Complains about use of `gmtime`. Ignoring since most samples use a single thread.
        )
        foreach(WARNING ${WARNINGS_THAT_SHOULD_BE_IGNORED})
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd${WARNING}")
        endforeach()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")

        if(CMAKE_VERSION VERSION_LESS 3.24 OR CMAKE_CXX_COMPILER_VERSION VERSION_LESS 19.29.30036.3)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /experimental:external /external:anglebrackets /external:W0")
        endif()
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W0")
    endif()
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /ignore:4099")
else()
    message(WARNING "Unknown compiler, not able to set compiler options for ${CMAKE_CXX_COMPILER_ID}")
endif()
