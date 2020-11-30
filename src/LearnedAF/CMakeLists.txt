# This is a sample application that builds based on Workspace/LIME combined SDK.
# The SDK abstracts away Workspace, Qt and other third party dependencies and
# provides simple C++ interface for use.
cmake_minimum_required(VERSION 3.3)

# https://cmake.org/cmake/help/git-stage/policy/CMP0071.html
if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()

project(LENSCONTROLDEMO)

# This should point to where the Workspace/LIME combined SDK's root folder is.
# CSIRO_INSTALL_AREA is defined when building the sample application within the LIME repo
# but would be undefined if building based on a copy of extracted Workspace/LIME SDK.
if (WIN32)
    set(SDK_FILES 
        bin/GenICamPlugin.dll
        bin/workspace.dll
        include/LIME/GenICamSDK.h
    )
else()
    set(SDK_FILES 
        bin/libGenICamPlugin.so
        bin/libworkspace.so
        include/LIME/GenICamSDK.h
    )
endif()

find_path(LIME_SDK_DIR 
    ${SDK_FILES}
    HINTS ${CSIRO_INSTALL_AREA} ENV LIME_SDK_DIR
    DOC "Path to SDK")

if (NOT LIME_SDK_DIR OR NOT EXISTS ${LIME_SDK_DIR}/include/LIME/GenICamSDK.h)
    message(FATAL_ERROR "Set LIME_SDK_DIR to the location of the LIME SDK")
endif()

set(LENSCONTROLDEMO_VERSION 1.0.0)
string(REGEX MATCH "^[0-9]+" LENSCONTROLDEMO_SOVERSION ${LENSCONTROLDEMO_VERSION})

set(LENSCONTROLDEMO_SOURCES
  ${LENSCONTROLDEMO_SOURCE_DIR}/main.cpp
)

# specify using SDK exposed from GenICamPlugin
include_directories(${PROJECT_BINARY_DIR} ${LIME_SDK_DIR}/include)
add_definitions(-DLENSCONTROLDEMO_VERSION=${LENSCONTROLDEMO_VERSION})

link_directories(
    ${LIME_SDK_DIR}/lib/Plugins
    ${LIME_SDK_DIR}/lib
)

find_package(OpenCV REQUIRED)

add_executable(collectAFdata ${LENSCONTROLDEMO_SOURCES} ${LENSCONTROLDEMO_RESOURCES} )
#configure_file(${LENSCONTROLDEMO_SOURCE_DIR}/Sample/FocusingSample.png ${LIME_SDK_DIR}/appdata/FocusingSample.png COPYONLY)

# When building inside the Workspace LIME repo, we would expect the output file located in the same bin folder.
if (DEFINED CSIRO_INSTALL_AREA)
    set_target_properties(collectAFdata PROPERTIES
        VERSION ${LENSCONTROLDEMO_VERSION}
        SOVERSION ${LENSCONTROLDEMO_SOVERSION}
        RUNTIME_OUTPUT_DIRECTORY ${LIME_SDK_DIR}/bin
        PDB_OUTPUT_DIRECTORY     ${LIME_SDK_DIR}/bin/pdb
    )
    if (CMAKE_GENERATOR MATCHES "Visual Studio")
        foreach(CONF ${CMAKE_CONFIGURATION_TYPES})
            string(TOUPPER "${CONF}" CONF)
            set_target_properties(collectAFdata PROPERTIES
                RUNTIME_OUTPUT_DIRECTORY_${CONF} ${LIME_SDK_DIR}/bin
                PDB_OUTPUT_DIRECTORY_${CONF}     ${LIME_SDK_DIR}/bin/pdb)
        endforeach()
    endif()
endif()

if (DEFINED COMMON_SENSORS_DIR)
    set(XIMEA_DIR ${COMMON_SENSORS_DIR}/Cameras/GenICam/Ximea/SDK)
else()
    set(XIMEA_SDK_FILES 
        xiApi.h
        x64/xiapi64.dll
    )
	find_path(XIMEA_DIR 
        ${XIMEA_SDK_FILES}
        DOC "Path to Ximea SDK")

    if (NOT XIMEA_DIR OR NOT EXISTS ${XIMEA_DIR}/xiApi.h)
        message(FATAL_ERROR "Set XIMEA_DIR to the location of the XIMEA SDK")
    endif()
endif()

set(XIMEA_SDK_LIB ${XIMEA_DIR}/x64/xiapi64.lib)
target_include_directories(collectAFdata PUBLIC ${XIMEA_DIR})
target_link_libraries(collectAFdata genicamplugin ${OpenCV_LIBS} ${XIMEA_SDK_LIB})
