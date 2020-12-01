cmake_minimum_required(VERSION 3.0)
if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()
project(TelemetricClient)

include_directories(${TelemetricClient_SOURCE_DIR})
if (NOT ${TelemetricClient_SOURCE_DIR} STREQUAL ${TelemetricClient_BINARY_DIR})
    include_directories(${TelemetricClient_BINARY_DIR})
endif()

if (NOT CSIRO_WORKSPACE_VERSION_MAJOR)

link_directories(${OpenCV_DIR}) 
    include_directories(${OpenCV_INCLUDE_DIRS} ) 

	
    INCLUDE_DIRECTORIES(
      ${TelemetricClient_SOURCE_DIR}
    )
	
	LINK_DIRECTORIES(${OPENCV_LIB}) 
endif()

set(HEADERS
    ${TelemetricClient_SOURCE_DIR}/PanTiltController.h
)

set(SOURCES
    ${TelemetricClient_SOURCE_DIR}/PanTiltController.cpp
)

add_library(TelemetricClient ${HEADERS} ${SOURCES})
target_link_libraries(TelemetricClient ${OpenCV_LIBS})

if (CSIRO_WORKSPACE_VERSION_MAJOR)
    # Part of a Workspace plugin build
    setTargetOutputDirectory(TelemetricClient ${CSIRO_INSTALL_AREA}/bin)
else()

endif()
    
