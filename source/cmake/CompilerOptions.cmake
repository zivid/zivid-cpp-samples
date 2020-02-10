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
            sign-conversion                 # Happens a lot in these samples, would complicate them too much to handle manually
            sign-compare                    # Happens a lot in these samples, would complicate them too much to handle manually
            shorten-64-to-32                # Narrowing conversions: Too strict and noisy for this code base
            padded                          # The type and order of elements caused the compiler to add padding to the end of a struct
            conversion                      # Implicit conversion loses integer precision (signed to unsigned). Expected to happen a lot in these samples, would complicate them too much to handle manually
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
            4702 # Got unreachable warnings from external, even though they are generallly ignored by /experimental:external
            4710 # If the compiler decides to not inline a function, that's their decision
            4711 # If the compiler decides to inline a function, that's their decision
            4571 # Just a non-interesting informational warning about msvc changing behaviour in 7.1
            4267 # Conversion: Happens a lot in these samples, would complicate them too much to handle manually
            4365 # Conversion: Happens a lot in these samples, would complicate them too much to handle manually
            4388 # Signed/unsigned comparison: Happens a lot in these samples, would complicate them too much to handle manually
            5045 # Compiler will insert Spectre mitigation for memory load if /Qspectre switch specified
                 # This warning is purely informational
            4244 # Narrowing conversions: Too strict and noisy for this code base
            4242 # Narrowing conversions: Too strict and noisy for this code base
            4820 # The type and order of elements caused the compiler to add padding to the end of a struct
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
