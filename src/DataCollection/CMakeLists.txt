cmake_minimum_required(VERSION 3.5)

##################################################
# Setup up VERSION and CXX_STANDARD
# Project: DataCollection
##################################################
set(LIB_MAJOR_VERSION "2")
set(LIB_MINOR_VERSION "0")
set(LIB_PATCH_VERSION "0")
set(LIB_VERSION_STRING "${LIB_MAJOR_VERSION}.${LIB_MINOR_VERSION}.${LIB_PATCH_VERSION}")

project(DataCollection VERSION "${LIB_VERSION_STRING}" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)

# FIXME - should probably exisit at top-level CMake
if(WIN32)
	add_compile_definitions(NOMINMAX) # solves problems with min/max macros in windows.h
endif()

###################################################
# Set TARGET_LIBRARY and PROJECT_PATH
###################################################
set(PROJECT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

message(STATUS)
message(STATUS ======================)
message(STATUS "${PROJECT_NAME} (v${${PROJECT_NAME}_VERSION}) - SETUP")
message(STATUS "CMAKE_VERSION ${CMAKE_VERSION}")
message(STATUS ======================)

###################################################
# Set Policy
###################################################
if( NOT ${CMAKE_VERSION} VERSION_LESS "3.12")
  cmake_policy(SET CMP0074 NEW) #use <PackageName>_ROOT var/env
  message(STATUS "Adopted CMake Policy CMP0074 ...")
endif()
# Requires CMake >=3.3 for IN_LIST
cmake_policy(SET CMP0057 NEW)

###################################################
# Set CMAKE_MODULE_PATH		- source Find<*>.cmake
# Set CMAKE_PREFIX_PATH     - source libraries
# Set CMAKE_LIBRARY_PATH    - external libs
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
get_filename_component(INCLUDE_PATH ${PROJECT_SOURCE_DIR}/../../include ABSOLUTE)
if (NOT ${INCLUDE_PATH} IN_LIST CMAKE_INCLUDE_PATH)
	list(APPEND CMAKE_INCLUDE_PATH ${INCLUDE_PATH})
endif()

message(STATUS ======================)
message(STATUS "Environment")
message(STATUS "PROJECT_PATH : ${PROJECT_PATH}")
message(STATUS "CMAKE_MODULE_PATH : ${CMAKE_MODULE_PATH}")
message(STATUS "CMAKE_PREFIX_PATH : ${CMAKE_PREFIX_PATH}")
message(STATUS "CMAKE_INCLUDE_PATH : ${CMAKE_INCLUDE_PATH}")
message(STATUS ======================)

###################################################
# Library and module inclusion
#   - If package is already sourced, don't resource
###################################################
# OpenCV
if (NOT OpenCV_FOUND)
	find_package(OpenCV 4.2 REQUIRED)
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
# Link subdirectory modules
###################################################
#message(STATUS ======================)
#message(STATUS "Linking Sub-Directories (of: ${PROJECT_NAME}) - BEGIN")
#message(STATUS ======================)

# Models 
#load_parallel_module(Models)

# Pose Estimation
#load_parallel_module(PoseEstimation)

# Utilities
#load_parallel_module(Utilities)

#message(STATUS ======================)
#message(STATUS "Linking Sub-Directories (of: ${PROJECT_NAME}) - FINISHED")
#message(STATUS ======================)

###################################################
# Create executable and/or library target and set properties
###################################################
set(PROJECT_SOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/DataCollector/DataCollector.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/CollectionGenerator/CollectionGenerator.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/ConfigParser/DataCollectionJSONParser.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/EstimatePoseCharucoBoard.cpp
)

# Libarry
add_library(${PROJECT_NAME} STATIC ${PROJECT_SOURCES})

# Set target properties
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

# Set target properties
target_include_directories(
        ${PROJECT_NAME}
		PUBLIC
			${PROJECT_SOURCE_DIR}
		PRIVATE
			${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
        ${PROJECT_NAME}
		PUBLIC
			Models
			PoseEstimation
			Utilities
		PRIVATE
			${OpenCV_LIBS}
#			nlohmann_json::nlohmann_json
			LiMeInterface
)

###################################################
# END
###################################################
message(STATUS ======================)
message(STATUS "${PROJECT_NAME} - FINISHED")
message(STATUS ======================)
message(STATUS)
