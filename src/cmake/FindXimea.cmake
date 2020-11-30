# FindXimea.cmake
#
# Find the Ximea camera library.
#
# This will define the following variables
#
#    Ximea_FOUND
#    Ximea_INCLUDE_DIRS
#    Ximea_LIBRARIES
#
# Author: Mason U'Ren

######################################
# Set package specific variables
######################################
set(PACKAGE_TARGET_NAME "Ximea")
set(TARGET_PATH ${CMAKE_PREFIX_PATH}/${PACKAGE_TARGET_NAME})

# Version Library
set(LIB_MAJOR_VERSION "1")
set(LIB_MINOR_VERSION "0")
set(LIB_PATCH_VERSION "2")
set(LIB_VERSION_STRING "${LIB_MAJOR_VERSION}.${LIB_MINOR_VERSION}.${LIB_PATCH_VERSION}")

#if(WIN32)
#    set(TARGET_PATH "C:/Metrology/lib/${PACKAGE_TARGET_NAME}")
#else()
#    set(TARGET_PATH "/opt/XIMEA")
#endif()

set(INCLUDE_HDRS
        xiApi.h
)
if(WIN32)
    set(LIB_PATH_SUFFIXES x86 x64)
    if (CMAKE_VS_PLATFORM_NAME STREQUAL "x64")
        set(LIBS
            xiapi64
        )
    elseif(CMAKE_VS_PLATFORM_NAME STREQUAL "Win32")
        set(LIBS
            xiapi32
        )
    else()
        message(ERROR "Unknown CMAKE_VS_PLATFORM_NAME: ${CMAKE_VS_PLATFORM_NAME}")
    endif()
else()
    set(LIBS m3api)
    set(LIB_PATH_SUFFIXES)
endif()

message(STATUS "Searching for package ${PACKAGE_TARGET_NAME} in TARGET_PATH=${TARGET_PATH}")
######################################
# Package creation
######################################
# Set package version...
set(${PACKAGE_TARGET_NAME}_VERSION ${LIB_VERSION_STRING})

# Set root _DIR...
set(${PACKAGE_TARGET_NAME}_DIR ${TARGET_PATH})

# Find includes...
find_path(
        ${PACKAGE_TARGET_NAME}_INCLUDE_DIRS
        NAMES
            ${INCLUDE_HDRS}
        PATHS
            ${TARGET_PATH}
        PATH_SUFFIXES
            include
         NO_DEFAULT_PATH
)

# Find libraries...s
foreach(library ${LIBS})
    find_library(
        ${library}_LIB
        NAMES
            ${LIBS}
        PATHS
            ${TARGET_PATH}/lib
            /usr/lib/
        PATH_SUFFIXES
            ${LIB_PATH_SUFFIXES}
    )

    # Verify library was found
    if(NOT ${library}_LIB)
        message("${library}.lib not found!")
    else()
        # Add element
        list(APPEND ${PACKAGE_TARGET_NAME}_LIBRARIES ${${library}_LIB})
    endif()
endforeach()


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
        ${PACKAGE_TARGET_NAME}
        REQUIRED_VARS
            ${PACKAGE_TARGET_NAME}_INCLUDE_DIRS
            ${PACKAGE_TARGET_NAME}_LIBRARIES
        VERSION_VAR
            ${PACKAGE_TARGET_NAME}_VERSION
)

mark_as_advanced(
        ${PACKAGE_TARGET_NAME}_FOUND
        ${PACKAGE_TARGET_NAME}_DIR
        ${PACKAGE_TARGET_NAME}_INCLUDE_DIRS
        ${PACKAGE_TARGET_NAME}_LIBRARIES
        ${PACKAGE_TARGET_NAME}_VERSION
)