cmake_minimum_required(VERSION 3.5)

##################################################
# Setup up VERSION and CXX_STANDARD
# Project: Models
##################################################
set(LIB_MAJOR_VERSION "1")
set(LIB_MINOR_VERSION "0")
set(LIB_PATCH_VERSION "0")
set(LIB_VERSION_STRING "${LIB_MAJOR_VERSION}.${LIB_MINOR_VERSION}.${LIB_PATCH_VERSION}")

project(Models VERSION "${LIB_VERSION_STRING}" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)

###################################################
# Set PROJECT_PATH
###################################################
set(PROJECT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

message(STATUS)
message(STATUS ======================)
message(STATUS "${PROJECT_NAME} - SETUP")
message(STATUS "CMAKE_VERSION ${CMAKE_VERSION}")
message(STATUS ======================)

###################################################
# Set CMAKE_MODULE_PATH		- source Find<*>.cmake
# Set CMAKE_PREFIX_PATH     - source libraries
# Set CMAKE_LIBRARY_PATH    - external libs
# Set CMAKE_INCLUDE_PATH    - project specific shared objects
###################################################
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

# Eigen3
if (NOT Eigen3_FOUND)
	find_package(Eigen3 3.3 REQUIRED)
endif ()
if (Eigen3_FOUND)
	message(STATUS "Found Eigen: v${Eigen3_VERSION}")
	message(STATUS "\tRoot: ${EIGEN3_ROOT_DIR}")
	message(STATUS "\tIncludes: ${EIGEN3_INCLUDE_DIRS}")
endif()

# OpenCV
if (NOT OpenCV_FOUND)
	find_package(OpenCV 4.2 REQUIRED COMPONENTS core aruco)
endif ()
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
set (
        PROJECT_HEADERS
			${CMAKE_CURRENT_SOURCE_DIR}/Board
			${CMAKE_CURRENT_SOURCE_DIR}/Camera
			${CMAKE_CURRENT_SOURCE_DIR}/PT
)

set (
	PROJECT_SOURCES
		${CMAKE_CURRENT_SOURCE_DIR}/ConfigParser/ModelsJSONParser.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/Board/FiducialModel.cpp
)

# Library
add_library( ${PROJECT_NAME} STATIC ${PROJECT_SOURCES} ${PROJECT_HEADERS})

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
			Utilities
			${OpenCV_LIBS}
			Eigen3::Eigen
		PRIVATE
			
			
)

message(STATUS ======================)
message(STATUS "${PROJECT_NAME} - FINISHED")
message(STATUS ======================)
message(STATUS)