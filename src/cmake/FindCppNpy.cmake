# FindCppNpy.cmake
#
# Find the header-only, template file(s) resposible for interacting with python Numpy arrays.
# - Specifically imported to handle *.npy files
#
# This will define the following variables
#
#    CppNpy_FOUND
#    CppNpy_INCLUDE_DIRS
#
# Author: Mason U'Ren

######################################
# Set package specific variables
######################################
set(PACKAGE_TARGET_NAME "CppNpy")
set(TARGET_PATH ${CMAKE_INCLUDE_PATH}/${PACKAGE_TARGET_NAME})

# Version Library
set(LIB_MAJOR_VERSION "1")
set(LIB_MINOR_VERSION "0")
set(LIB_PATCH_VERSION "0")
set(LIB_VERSION_STRING "${LIB_MAJOR_VERSION}.${LIB_MINOR_VERSION}.${LIB_PATCH_VERSION}")

# Header files
set(INCLUDE_HDRS
        numpy.h
)

######################################
# Package creation
######################################
find_package(PkgConfig REQUIRED)
pkg_check_modules(PC_${PACKAGE_TARGET_NAME} QUIET ${PACKAGE_TARGET_NAME})

# Set package version...
set(${PACKAGE_TARGET_NAME}_VERSION ${LIB_VERSION_STRING})

# Set root _DIR
set(${PACKAGE_TARGET_NAME}_DIR ${TARGET_PATH})

# Find includes...
find_path(
        ${PACKAGE_TARGET_NAME}_INCLUDE_DIRS
        NAMES
            ${INCLUDE_HDRS}
        PATHS
            ${TARGET_PATH}/include
)

mark_as_advanced(
        ${PACKAGE_TARGET_NAME}_FOUND
        ${PACAKGE_TARGET_NAME}_DIR
        ${PACKAGE_TARGET_NAME}_INCLUDE_DIRS
        ${PACKAGE_TARGET_NAME}_LIBRARIES
        ${PACKAGE_TARGET_NAME}_VERSION
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
        ${PACKAGE_TARGET_NAME}
        REQUIRED_VARS
            ${PACAKGE_TARGET_NAME}_DIR
            ${PACKAGE_TARGET_NAME}_INCLUDE_DIRS
        VERSION_VAR
            ${PACKAGE_TARGET_NAME}_VERSION
)