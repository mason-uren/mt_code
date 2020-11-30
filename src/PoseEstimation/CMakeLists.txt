cmake_minimum_required(VERSION 3.5)

##################################################
# Setup up VERSION and CXX_STANDARD
# Project: PoseEstimation
##################################################
set(LIB_MAJOR_VERSION "2")
set(LIB_MINOR_VERSION "0")
set(LIB_PATCH_VERSION "0")
set(LIB_VERSION_STRING "${LIB_MAJOR_VERSION}.${LIB_MINOR_VERSION}.${LIB_PATCH_VERSION}")

project(PoseEstimation VERSION "${LIB_VERSION_STRING}" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)

###################################################
# Set PROJECT_PATH
###################################################
set(PROJECT_PATH ${CMAKE_CURRENT_SOURCE_DIR})

message(STATUS)
message(STATUS ======================)
message(STATUS "${PROJECT_NAME} - SETUP")
message(STATUS "CMAKE_VERSION ${CMAKE_VERSION}")
message(STATUS "CMAKE_INCLUDE_PATH : ${CMAKE_INCLUDE_PATH}")
message(STATUS ======================)

###################################################
# Set CMAKE_MODULE_PATH		- source Find<*>.cmake
# Set CMAKE_PREFIX_PATH     - source Find<*>Config.cmake
# Set CMAKE_LIBRARY_PATH    - external libs
# Set CMAKE_INCLUDE_PATH    - project specific shared objects
###################################################
message(STATUS "Environment")
message(STATUS "PROJECT_PATH : ${PROJECT_PATH}")
message(STATUS "CMAKE_MODULE_PATH : ${CMAKE_MODULE_PATH}")
message(STATUS "CMAKE_PREFIX_PATH : ${CMAKE_PREFIX_PATH}")
message(STATUS ======================)

###################################################
# Library and module inclusion
#   - If package is already sourced, don't resource
###################################################
# OpenCV
if(NOT OpenCV_FOUND)
    find_package(OpenCV REQUIRED COMPONENTS core aruco)
endif()

if (OpenCV_FOUND)
    message(STATUS "Found OpenCV: v${OpenCV_VERSION}")
    message(STATUS "\tRoot: ${OpenCV_DIR}")
    message(STATUS "\tIncludes: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "\tLibs: ${OpenCV_LIBS}")
endif()

# Eigen
if(NOT Eigen3_FOUND)
    find_package(Eigen3 3.3 REQUIRED)
endif()

if (Eigen3_FOUND)
    message(STATUS "Found Eigen: v${Eigen3_VERSION}")
    message(STATUS "\tRoot: ${EIGEN3_ROOT_DIR}")
    message(STATUS "\tIncludes: ${EIGEN3_INCLUDE_DIRS}")
endif()

###################################################
# Link subdirectory modules
###################################################
#message(STATUS ======================)
#message(STATUS "Linking Sub-Directories (of: ${PROJECT_NAME}) - BEGIN")
#message(STATUS ======================)

# Models
#load_parallel_module(Models)

#message(STATUS ======================)
#message(STATUS "Linking Sub-Directories (of: ${PROJECT_NAME}) - FINISHED")
#message(STATUS ======================)

###################################################
# Create executable and/or library target and set properties
###################################################
set(
	PROJECT_SOURCES 
                ${PROJECT_SOURCE_DIR}/PoseEstimator.cpp
                ${PROJECT_SOURCE_DIR}/PoseEstimator.h
                ${PROJECT_SOURCE_DIR}/SixDOF.cpp
                ${PROJECT_SOURCE_DIR}/SixDOF.h
                ${PROJECT_SOURCE_DIR}/FusedPoses/FusedPoses.cpp
                ${PROJECT_SOURCE_DIR}/FusedPoses/FusedPoses.h
                ${PROJECT_SOURCE_DIR}/HandOffSolver/HandOffSolver.cpp
                ${PROJECT_SOURCE_DIR}/HandOffSolver/HandOffSolver.h
)

add_library(${PROJECT_NAME} STATIC ${PROJECT_SOURCES})

target_include_directories(${PROJECT_NAME}
            PUBLIC
                ${PROJECT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME}
	PUBLIC
		${OpenCV_LIBS}
		Eigen3::Eigen
		# Should be Models
		DynamicExtrinsics
        Utilities         #need VideoObject
	PRIVATE

)

message(STATUS ======================)
message(STATUS "${PROJECT_NAME} - FINISHED")
message(STATUS ======================)
message(STATUS)

