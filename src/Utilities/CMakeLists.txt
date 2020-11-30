cmake_minimum_required(VERSION 3.5)

##################################################
# Setup up VERSION and CXX_STANDARD
# Project: Utilities
##################################################
set(LIB_MAJOR_VERSION "1")
set(LIB_MINOR_VERSION "0")
set(LIB_PATCH_VERSION "0")
set(LIB_VERSION_STRING "${LIB_MAJOR_VERSION}.${LIB_MINOR_VERSION}.${LIB_PATCH_VERSION}")

project(Utilities VERSION "${LIB_VERSION_STRING}" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)

# Compiler definitions
if(WIN32)
	add_compile_definitions(NOMINMAX) # solves problems with min/max macros in windows.h
endif()

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
# HDF5
if (NOT HDF5_FOUND)
    set(ENV{HDF5_DIR} ${PREFIX_PATH}/HDF5/install/cmake)
	find_package(HDF5 1.12.0 REQUIRED COMPONENTS CXX) 
endif ()
if (HDF5_FOUND)
	message(STATUS "Found HDF5: v${HDF5_VERSION}")
	message(STATUS "\tRoot: ${HDF5_DIR}")
	message(STATUS "\tIncludes: ${HDF5_INCLUDE_DIRS}")
	message(STATUS "\tLibs: ${HDF5_LIBRARIES}")
endif()

# OpenCV 
if(NOT OpenCV_FOUND)
	find_package(OpenCV 4.2 REQUIRED COMPONENTS core highgui)
endif()
if (OpenCV_FOUND)
    message(STATUS "Found OpenCV: v${OpenCV_VERSION}")
    message(STATUS "\tRoot: ${OpenCV_DIR}")
    message(STATUS "\tIncludes: ${OpenCV_INCLUDE_DIRS}")
	message(STATUS "\tLibs: ${OpenCV_LIBS}")
endif()

# HRL
# Common Includes
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
set(
	PROJECT_SOURCES
		${CMAKE_CURRENT_LIST_DIR}/ErrorHandler/ErrorHandler.cpp
		${CMAKE_CURRENT_LIST_DIR}/JSONParser/JSONParser.cpp
		${CMAKE_CURRENT_LIST_DIR}/SystemConfigParser/SystemJSONParser.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Logger/Logger.cpp
		${CMAKE_CURRENT_LIST_DIR}/ThreadInterface/ThreadInterface.cpp
		${CMAKE_CURRENT_LIST_DIR}/OpenCVHelpers/Video/VideoObject.cpp
		${CMAKE_CURRENT_LIST_DIR}/OpenCVHelpers/Video/VideoRecorder.cpp
		${CMAKE_CURRENT_LIST_DIR}/OpenCVHelpers/CSV/MatRecorder.cpp
)

set(
	PROJECT_HEADERS
		${CMAKE_CURRENT_LIST_DIR}/H5
		${CMAKE_CURRENT_LIST_DIR}/Matrix
		${CMAKE_CURRENT_LIST_DIR}/OpenCVHelpers/Transformations

		# MSVS convience inclusion
		${CMAKE_CURRENT_LIST_DIR}/OpenCVHelpers/Video/VideoObject.h
        ${CMAKE_CURRENT_LIST_DIR}/OpenCVHelpers/Video/VideoRecorder.h
		${CMAKE_CURRENT_LIST_DIR}/OpenCVHelpers/CSV/MatRecorder.h
)

add_library(
        ${PROJECT_NAME} STATIC ${PROJECT_SOURCES} ${PROJECT_HEADERS}
)

# Set target properties
target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
			${PROJECT_SOURCE_DIR}
			${HDF5_INCLUDE_DIRS}
		PRIVATE
			${HDF5_SRC_DIR}
			${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
        ${PROJECT_NAME}
		PUBLIC
			${HDF5_LIBRARIES}
		PRIVATE 
			${HDF5_CPP_LIBSH_TARGET}
            ${HDF5_LIBSH_TARGET} 
			${OpenCV_LIBS}
			
)

message(STATUS ======================)
message(STATUS "${PROJECT_NAME} - FINISHED")
message(STATUS ======================)
message(STATUS)
