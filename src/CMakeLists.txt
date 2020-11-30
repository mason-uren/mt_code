cmake_minimum_required(VERSION 3.5)

##################################################
# Setup up VERSION and CXX_STANDARD
# Project: LiveMetrology
##################################################
set(LIB_MAJOR_VERSION "2")
set(LIB_MINOR_VERSION "0")
set(LIB_PATCH_VERSION "0")
set(LIB_VERSION_STRING "${LIB_MAJOR_VERSION}.${LIB_MINOR_VERSION}.${LIB_PATCH_VERSION}")

# Root CMake 
# - no CMake should exists in higher parent directory that is considered apart of the project
project(HRLDemoApp VERSION "${LIB_VERSION_STRING}" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler directives/Options
if(WIN32)
    add_compile_definitions(NOMINMAX) # solves problems with min/max macros in windows.h
	add_compile_definitions(WIN32_LEAN_AND_MEAN)
	add_compile_definitions(_WINSOCK_DEPRECATED_NO_WARNINGS)
	add_compile_definitions(_USE_MATH_DEFINES)
else()
    if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux" )
	add_compile_definitions(LINUX)
	# Threads
	set(CMAKE_CXX_FLAGS "-pthread")
    endif()
endif()

# The following places build products (executables, libraries (static & dynamic), etc) into the
#  lib and bin directories within the build directory.
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    CACHE STRING "Single directory for all Runtime files." FORCE)
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
    CACHE STRING "Single directory for all Library files." FORCE)
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
    CACHE STRING "Single directory for all Archive files." FORCE)

###################################################
# Set PROJECT_PATH
###################################################
set(PROJECT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

message(STATUS)
message(STATUS ======================)
message(STATUS "${PROJECT_NAME} - SETUP")
message(STATUS "CMAKE_VERSION ${CMAKE_VERSION}")
message(STATUS "CXX_VERSION ${CMAKE_CXX_STANDARD}")
message(STATUS ======================)

###################################################
# Set CMAKE_MODULE_PATH		- source Find<*>.cmake
# Set CMAKE_PREFIX_PATH     - source Find<*>Config.cmake
# Set CMAKE_LIBRARY_PATH    - external libs
# Set CMAKE_INCLUDE_PATH    - project specific shared objects
###################################################
# CMAKE_MODULE_PATH
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# CMAKE_PREFIX_PATH
get_filename_component(PREFIX_PATH ${CMAKE_SOURCE_DIR}/../lib ABSOLUTE)
list(APPEND CMAKE_PREFIX_PATH ${PREFIX_PATH})

# CMAKE_INCLUDE_PATH
get_filename_component(INCLUDE_PATH ${CMAKE_SOURCE_DIR}/../include ABSOLUTE)
list(APPEND CMAKE_INCLUDE_PATH ${INCLUDE_PATH})

message(STATUS "Environment")
message(STATUS "PROJECT_PATH : ${PROJECT_PATH}")
message(STATUS "CMAKE_MODULE_PATH : ${CMAKE_MODULE_PATH}")
message(STATUS "CMAKE_PREFIX_PATH : ${CMAKE_PREFIX_PATH}")
message(STATUS "CMAKE_INCLUDE_PATH : ${CMAKE_INCLUDE_PATH}")
message(STATUS ======================)

###################################################
# Link subdirectory modules
###################################################
message(STATUS ======================)
message(STATUS "Linking Sub-Directories (of: ${PROJECT_NAME}) - BEGIN")
message(STATUS ======================)

# CommonIncludes (Header Inclusions)
set(CommonIncludes_DIR ${PROJECT_SOURCE_DIR}/CommonIncludes)
include_directories(${CommonIncludes_DIR})
set(CommonIncludes_FOUND TRUE)

# External library inclusions...
include_directories(${INCLUDE_PATH})

# Utilities
add_subdirectory(Utilities)

# Models
add_subdirectory(Models)

# Dynamic Extrinsics
add_subdirectory(DynamicExtrinsics)

# Pose Estimation
add_subdirectory(PoseEstimation)

# LiMe Interface
add_subdirectory(LiMeInterface)

# DataCollection
add_subdirectory(DataCollection)

# Tracking
add_subdirectory(TrackingFiducial)

message(STATUS ======================)
message(STATUS "Linking Sub-Directories (of: ${PROJECT_NAME}) - FINISHED")
message(STATUS ======================)

###################################################
# Create executable and/or library target and set properties
###################################################
set(
        PROJECT_SOURCES
		# Should be left blank
		# - For quick testing add files here
)

# Intended to only operate on Windows OS
if (WIN32)
	add_executable(
		${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/main.cpp ${PROJECT_SOURCES}
	)

	# Set target properties
	target_include_directories(
			${PROJECT_NAME}
			PUBLIC

			PRIVATE
	)

	target_link_libraries(
			${PROJECT_NAME}
			PUBLIC
				Utilities
				Models
				DynamicExtrinsics
				PoseEstimation
				LiMeInterface
				DataCollection
				TrackingFiducial
	)
endif()

message(STATUS ======================)
message(STATUS "${PROJECT_NAME} - FINISHED")
message(STATUS ======================)
message(STATUS)
