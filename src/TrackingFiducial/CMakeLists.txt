cmake_minimum_required(VERSION 3.5)

##################################################
# Setup up VERSION and CXX_STANDARD
# Project: TrackFiducial
##################################################
set(LIB_MAJOR_VERSION "1")
set(LIB_MINOR_VERSION "0")
set(LIB_PATCH_VERSION "0")
set(LIB_VERSION_STRING "${LIB_MAJOR_VERSION}.${LIB_MINOR_VERSION}.${LIB_PATCH_VERSION}")

project(TrackingFiducial  VERSION "${LIB_VERSION_STRING}" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(WIN32)
    add_compile_definitions(NOMINMAX) # solves problems with min/max macros in windows.h
endif()

###################################################
# Set PROJECT_PATH
###################################################
set(PROJECT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

message(STATUS)
message(STATUS ======================)
message(STATUS "${PROJECT_NAME} (v${${PROJECT_NAME}_VERSION}) - SETUP")
message(STATUS "CMAKE_VERSION ${CMAKE_VERSION}")
message(STATUS ======================)

###################################################
# Set CMAKE_MODULE_PATH		- source Find<*>.cmake
# Set CMAKE_PREFIX_PATH     - source libraries
# Set CMAKE_INCLUDE_PATH    - project specific shared objects
###################################################
# CMAKE_MODULE_PATH
get_filename_component(MODULE_PATH  ${PROJECT_SOURCE_DIR}/../cmake ABSOLUTE)
if (NOT ${MODULE_PATH} IN_LIST CMAKE_MODULE_PATH)
	list(APPEND CMAKE_MODULE_PATH ${MODULE_PATH})
endif()

# CMAKE_PREFIX_PATH
get_filename_component(PREFIX_PATH  ${PROJECT_SOURCE_DIR}/../../lib ABSOLUTE)
if(NOT ${PREFIX_PATH} IN_LIST CMAKE_PREFIX_PATH)
	list(APPEND CMAKE_PREFIX_PATH ${PREFIX_PATH})
endif()

# CMAKE_INCLUDE_PATH
get_filename_component(INCLUDE_PATH ${PROJECT_SOURCE_DIR}/../../include  ABSOLUTE)
if (NOT ${INCLUDE_PATH} IN_LIST CMAKE_INCLUDE_PATH)
	list(APPEND CMAKE_INCLUDE_PATH ${INCLUDE_PATH})
endif()

message(STATUS ======================)
message(STATUS "Environment:")
message(STATUS "PROJECT_PATH : ${PROJECT_PATH}")
message(STATUS "CMAKE_MODULE_PATH : ${CMAKE_MODULE_PATH}")
message(STATUS "CMAKE_PREFIX_PATH : ${CMAKE_PREFIX_PATH}")
message(STATUS "CMAKE_INCLUDE_PATH : ${CMAKE_INCLUDE_PATH}")
message(STATUS ======================)

###################################################
# Source helper functions
###################################################
include(${MODULE_PATH}/Functions.cmake)

###################################################
# Library and module inclusion
#   - If package is already sourced, don't resource
###################################################

# OpenCV 
if(NOT OpenCV_FOUND)
    find_package(OpenCV REQUIRED COMPONENTS core highgui aruco)
endif()
if (OpenCV_FOUND)
    message(STATUS "Found OpenCV: v${OpenCV_VERSION}")
    message(STATUS "\tRoot: ${OpenCV_DIR}")
    message(STATUS "\tIncludes: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "\tLibs: ${OpenCV_LIBS}")
endif()

# HRL
# CommonIncludes
get_property(INCLUDED_DIRS DIRECTORY ${CMAKE_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
if (NOT ${CommonIncludes_DIR} IN_LIST INCLUDE_DIRS)
	set(CommonIncludes_DIR ${CMAKE_SOURCE_DIR}/CommonIncludes)
	include_directories(${CommonIncludes_DIR})
	set(CommonIncludes_FOUND TRUE)
endif()
if (CommonIncludes_FOUND)
	message(STATUS "Found CommonIncludes Directory(s): ${CommonIncludes_DIR}")
endif()

###################################################
# Create executable and/or library target and set properties
###################################################
# Hardcoded default values used by the main.cpp, can be changed by command-line options:
add_compile_definitions(CAD_MODEL_JSON_PATH="${PROJECT_PATH}/../Config/DynamicExtrinsics/Ximea_id0_Imperx/2020_10_01_ptu0-cad-model.json")
add_compile_definitions(XIMEA_INTRINSICS_DIR="${PROJECT_PATH}/../Config/CameraIntrinsics/Ximea_id0/2020-07-15/")

set (
	PROJECT_HEADERS
	${CMAKE_CURRENT_SOURCE_DIR}/FiducialTracker.h
	${CMAKE_CURRENT_SOURCE_DIR}/FTVideoObject.h
)

set (
	PROJECT_SOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/ConfigParser/TrackingJSONParser.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/FiducialTracker.cpp 
	${CMAKE_CURRENT_SOURCE_DIR}/FTVideoObject.cpp
)

#
# Library
#
add_library(${PROJECT_NAME} STATIC ${PROJECT_SOURCES} ${PROJECT_HEADERS})

# Set target properties
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(
	# Target
	${PROJECT_NAME}
	PUBLIC
		${PROJECT_SOURCE_DIR}
	PRIVATE
		${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
	# Target
	${PROJECT_NAME}
	PRIVATE
		Models
		DynamicExtrinsics
		${OpenCV_LIBS}
	PUBLIC
		Utilities
		PoseEstimation
		LiMeInterface

)

#
# Executable
#
set(PROJECT_EXAMPLE_NAME ${PROJECT_NAME}_Example)
add_executable(${PROJECT_EXAMPLE_NAME} main.cpp)

# Set target properties
# Exclude by default on Windows
if(WIN32)
    exclude_target(${PROJECT_EXAMPLE_NAME})
endif()
set_target_properties(${PROJECT_EXAMPLE_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(
	# Target
	${PROJECT_EXAMPLE_NAME}
	PUBLIC
		${PROJECT_SOURCE_DIR}
		${OpenCV_INCLUDE_DIRS}
	PRIVATE
		
)

target_link_libraries(
	# Target
	${PROJECT_EXAMPLE_NAME}
	PRIVATE
		Models
		DynamicExtrinsics
		${PROJECT_NAME}
    PUBLIC
		Utilities
		PoseEstimation
		LiMeInterface
		${OpenCV_LIBS}

        
)

###################################################
# END
###################################################
message(STATUS ======================)
message(STATUS "${PROJECT_NAME} - FINISHED")
message(STATUS ======================)
message(STATUS)
