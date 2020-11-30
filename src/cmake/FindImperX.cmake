# FindImperX.cmake
#
# Find the ImperX camera library.
#
# This will define the following variables
#
#    ImperX_FOUND
#    ImperX_INCLUDE_DIRS
#    ImperX_LIBRARIES
#
# Author: Mason U'Ren

######################################
# Set package specific variables
######################################
set(PACKAGE_TARGET_NAME "ImperX")
set(TARGET_PATH ${CMAKE_PREFIX_PATH}/${PACKAGE_TARGET_NAME})

# Version Library
set(LIB_MAJOR_VERSION "1")
set(LIB_MINOR_VERSION "0")
set(LIB_PATCH_VERSION "2")
set(LIB_VERSION_STRING "${LIB_MAJOR_VERSION}.${LIB_MINOR_VERSION}.${LIB_PATCH_VERSION}")


# TODO - old
if(WIN32)
#    set(TARGET_PATH "C:/Metrology/lib/${PACKAGE_TARGET_NAME}")
    set(LIBS 
		ipximageapi
		ipxdisplay
		IpxCameraApi
		IpxCameraGuiApi
		
	)
else() 
#    # Set-up for building the code only
#    set(TARGET_PATH "/home.shared/ychen/BoeingDR-ClosedLoopMetrology/ImperX-Driver/IpxCameraSDK-1.2.0.13")
    set(LIBS IpxDisplayQt;IpxCameraApi;IpxCameraGuiApi;IpxImageApi)
endif()

set(INCLUDE_HDRS
        IpxCameraApi.h
        IpxCameraErr.h
        IpxImage.h
        IpxPixelType.h
)

if(WIN32)
    if (CMAKE_VS_PLATFORM_NAME STREQUAL "x64")
        set(LIB_PATH_SUFFIX
            win64_x64
        )
    elseif(CMAKE_VS_PLATFORM_NAME STREQUAL "Win32")
        set(LIB_PATH_SUFFIX
            win32_i86
        )
    else()
        message(ERROR "Unknown CMAKE_VS_PLATFORM_NAME: ${CMAKE_VS_PLATFORM_NAME}")
    endif()
else()
    set(LIB_PATH_SUFFIX
            Linux64_x64
        )
endif()

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
            inc
)

# Find libraries...
foreach(library ${LIBS})
	find_library(
        #IPX_LIB_PATH
		${library}_LIB
		NAMES
			${library}
        PATHS
            ${TARGET_PATH}
        PATH_SUFFIXES
            lib/${LIB_PATH_SUFFIX}
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
