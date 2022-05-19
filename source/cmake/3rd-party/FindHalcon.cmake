if(NOT DEFINED HALCONARCH)
  if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
      set(HALCONARCH x64-macosx)
  elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
      set(HALCONARCH x64-linux)
  elseif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
      if("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
          set(HALCONARCH x64-win64)
      else()
          set(HALCONARCH x86sse2-win32)
      endif()
  else()
      message( FATAL_ERROR "Unsupported system." )
  endif()
endif()

if(HALCONROOT)

  if(EXISTS "${HALCONROOT}/lib")
    set(HALCON_EXT_LIB_DIR ${HALCONROOT}/lib/${HALCONARCH})
  else()
    if(EXISTS "${HALCONROOT}/libd")
      set(HALCON_EXT_LIB_DIR ${HALCONROOT}/libd/${HALCONARCH})
    else()
      if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
        message( FATAL_ERROR "HALCONROOT environment variable is not set or HALCON is not installed.")
      endif()
    endif()
  endif()

  if(EXISTS "${HALCONROOT}/include")
    set(HALCON_INC_DIRS ${HALCONROOT}/include)
  else()
    if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
      message( FATAL_ERROR "HALCONROOT environment variable is not set or HALCON is not installed.")
    endif()
  endif()

endif()
                    
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  if(HALCON_EXT_LIB_DIR)
    set(HALCON_LIBS "-F ${HALCON_EXT_LIB_DIR} -framework HALCON"
                    "-F ${HALCON_EXT_LIB_DIR} -framework HALCONC"
                    "-F ${HALCON_EXT_LIB_DIR} -framework HALCONCpp")

    set(HALCONXL_LIBS "-F ${HALCON_EXT_LIB_DIR} -framework HALCONxl"
                      "-F ${HALCON_EXT_LIB_DIR} -framework HALCONCxl"
                      "-F ${HALCON_EXT_LIB_DIR} -framework HALCONCppxl")
  else()
    set(HALCON_LIBS "-framework HALCON"
                    "-framework HALCONC"
                    "-framework HALCONCpp")

    set(HALCONXL_LIBS "-framework HALCONxl"
                      "-framework HALCONCxl"
                      "-framework HALCONCppxl")
  endif()
else()
  if(${WIN32})
    set(PREFIX ${CMAKE_IMPORT_LIBRARY_PREFIX})
    set(SUFFIX ${CMAKE_IMPORT_LIBRARY_SUFFIX})
  else()
    set(PREFIX ${CMAKE_SHARED_LIBRARY_PREFIX})
    set(SUFFIX ${CMAKE_SHARED_LIBRARY_SUFFIX})
  endif()
  if(HALCON_EXT_LIB_DIR)
    set(HALCON_LIBS ${HALCON_EXT_LIB_DIR}/${PREFIX}halcon${SUFFIX}
                    ${HALCON_EXT_LIB_DIR}/${PREFIX}halconc${SUFFIX}
                    ${HALCON_EXT_LIB_DIR}/${PREFIX}halconcpp${SUFFIX})

    set(HALCON_LIBS_XL ${HALCON_EXT_LIB_DIR}/${PREFIX}halconxl${SUFFIX}
                       ${HALCON_EXT_LIB_DIR}/${PREFIX}halconcxl${SUFFIX}
                       ${HALCON_EXT_LIB_DIR}/${PREFIX}halconcppxl${SUFFIX})
  else()
    set(HALCON_LIBS ${PREFIX}halcon${SUFFIX}
                    ${PREFIX}halconc${SUFFIX}
                    ${PREFIX}halconcpp${SUFFIX})

    set(HALCON_LIBS_XL ${PREFIX}halconxl${SUFFIX}
                       ${PREFIX}halconcxl${SUFFIX}
                       ${PREFIX}halconcppxl${SUFFIX})
  endif()
endif()