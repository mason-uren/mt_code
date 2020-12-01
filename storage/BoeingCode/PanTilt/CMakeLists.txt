cmake_minimum_required(VERSION 3.0)
if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()

project(PANTILTPLUGIN)

if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    find_path(WORKSPACE_CMAKE_ROOT Workspace_CMake_Root.txt
              HINTS ${CMAKE_SOURCE_DIR} ENV WORKSPACE_CMAKE_ROOT
              DOC "Directory containing the Workspace_CMake_Root.txt file")
    if (NOT WORKSPACE_CMAKE_ROOT)
        message(FATAL_ERROR "Please set WORKSPACE_CMAKE_ROOT to the directory containing the file Workspace_CMake_Root.txt")
    endif()
    include( ${WORKSPACE_CMAKE_ROOT}/CMakeLists.txt NO_POLICY_SCOPE )
endif()


include_directories(${PANTILTPLUGIN_SOURCE_DIR})
if (NOT ${PANTILTPLUGIN_SOURCE_DIR} STREQUAL ${PANTILTPLUGIN_BINARY_DIR})
    include_directories(${PANTILTPLUGIN_BINARY_DIR})
endif()


set(PANTILTPLUGIN_VERSION 0.1.0)
string(REGEX MATCH "^[0-9]+" PANTILTPLUGIN_SOVERSION ${PANTILTPLUGIN_VERSION})

# Add other Qt modules as required by your plugin - eg. QtNetwork, QtOpenGL, QtSql
find_package(Qt5Core)
find_package(Qt5Widgets)
set(QT_LIBRARIES Qt5::Core;Qt5::Widgets)

set(HEADERS
    ${PANTILTPLUGIN_SOURCE_DIR}/executecommand.h
    ${PANTILTPLUGIN_SOURCE_DIR}/getangles.h
    ${PANTILTPLUGIN_SOURCE_DIR}/movepantilt.h
    ${PANTILTPLUGIN_SOURCE_DIR}/pantiltplugin_api.h
    ${PANTILTPLUGIN_SOURCE_DIR}/pantiltplugin.h

)

set(INSTALL_HEADERS
    ${PANTILTPLUGIN_SOURCE_DIR}/executecommand.h
    ${PANTILTPLUGIN_SOURCE_DIR}/getangles.h
    ${PANTILTPLUGIN_SOURCE_DIR}/movepantilt.h
    ${PANTILTPLUGIN_SOURCE_DIR}/pantiltplugin_api.h
    ${PANTILTPLUGIN_SOURCE_DIR}/pantiltplugin.h
)

set(MOC_HEADERS
)

set(SOURCES
    ${PANTILTPLUGIN_SOURCE_DIR}/executecommand.cpp
    ${PANTILTPLUGIN_SOURCE_DIR}/getangles.cpp
    ${PANTILTPLUGIN_SOURCE_DIR}/movepantilt.cpp
    ${PANTILTPLUGIN_SOURCE_DIR}/pantiltplugin.cpp
)

set(UI_SOURCES
)

add_definitions(-DPANTILTPLUGIN_VERSION=${PANTILTPLUGIN_VERSION})

# The next line is used by the simple application generator wizard
# add_subdirectory(${PANTILTPLUGIN_SOURCE_DIR}/Application)

# The below line can be used to import sub-directories
# include( ${PANTILTPLUGIN_SOURCE_DIR}/SubDir/CMakeLists.txt )

# qtx macros are defined in the ${WORKSPACE_CMAKE_ROOT}/CMakeLists.txt included at the top of this file
# to support both Qt4 and Qt5 builds.
qtx_wrap_cpp(MOC_SOURCES ${MOC_HEADERS})
qtx_wrap_ui(UIC_SOURCES ${UI_SOURCES})
qtx_add_resources(RES_SOURCES ${RESOURCES})

add_library(pantiltplugin ${SOURCES} ${HEADERS} ${MOC_SOURCES} ${UIC_SOURCES} ${RES_SOURCES})
target_link_libraries(pantiltplugin  workspace ${QT_LIBRARIES} TelemetricClient opencvplugin)
add_dependencies(pantiltplugin opencvplugin)

set_target_properties(pantiltplugin PROPERTIES
    DEFINE_SYMBOL PANTILTPLUGIN_EXPORT
    VERSION ${PANTILTPLUGIN_VERSION}
    SOVERSION ${PANTILTPLUGIN_SOVERSION}
)

setTargetOutputDirectory(pantiltplugin ${CSIRO_INSTALL_AREA}/lib/Plugins)
configure_file(pkg-pantiltplugin.cmake ${CSIRO_INSTALL_AREA}/cmake/Exports/pkg-pantiltplugin.cmake @ONLY)

# Copy our install headers into the install directory so that others can build against our plugin.
foreach(inFile ${INSTALL_HEADERS})
    string(REGEX REPLACE "(${PANTILTPLUGIN_SOURCE_DIR}/)(.*)" "${CSIRO_INSTALL_AREA}/include/PanTiltPlugin/\\2" outFile "${inFile}")
    configure_file(${inFile} ${outFile} COPYONLY)
endforeach(inFile)

add_subdirectory(Designer)
add_subdirectory(TelemetricClient)

