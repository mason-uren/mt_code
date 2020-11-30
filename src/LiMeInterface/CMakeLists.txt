cmake_minimum_required(VERSION 3.5)

##################################################
# Setup up VERSION and CXX_STANDARD
# Project: LiMeInterface
##################################################
set(LIB_MAJOR_VERSION "2")
set(LIB_MINOR_VERSION "0")
set(LIB_PATCH_VERSION "0")
set(LIB_VERSION_STRING "${LIB_MAJOR_VERSION}.${LIB_MINOR_VERSION}.${LIB_PATCH_VERSION}")

project(LiMeInterface VERSION "${LIB_VERSION_STRING}" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
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
# Set CMAKE_PREFIX_PATH     - source Find<*>Config.cmake
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
if (NOT ${PREFIX_PATH} IN_LIST CMAKE_PREFIX_PATH)
	list(APPEND CMAKE_PREFIX_PATH ${PREFIX_PATH})
endif()

# CMAKE_INCLUDE_PATH
get_filename_component(INCLUDE_PATH ${PROJECT_SOURCE_DIR}/../../include ABSOLUTE)
if (NOT ${INCLUDE_PATH} IN_LIST CMAKE_INCLUDE_PATH)
	list(APPEND CMAKE_INCLUDE_PATH ${INCLUDE_PATH})
endif()

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

# Ximea
if (NOT Ximea_FOUND)
	find_package(Ximea QUIET)
endif()
if (Ximea_FOUND)
    message(STATUS "Found Ximea: v${Ximea_VERSION}")
    message(STATUS "\tRoot: ${Ximea_DIR}")
    message(STATUS "\tIncludes: ${Ximea_INCLUDE_DIRS}")
	message(STATUS "\tLibs: ${Ximea_LIBRARIES}")

	# Set source variable
	add_compile_definitions(USE_XIMEA_SDK=true)
else()
	# Or cmake would not pass when Ximea SDK is not found
	set(Ximea_INCLUDE_DIRS "")  
endif()
# ImperX
if (NOT ImperX_FOUND)
	find_package(ImperX QUIET)
endif()
if (ImperX_FOUND)
    message(STATUS "Found ImperX: v${ImperX_VERSION}")
    message(STATUS "\tRoot: ${ImperX_DIR}")
    message(STATUS "\tIncludes: ${ImperX_INCLUDE_DIRS}")
	message(STATUS "\tLibs: ${ImperX_LIBRARIES}")

	# Set source variable
	add_compile_definitions(USE_IMPERX_SDK=true)
else()
	# Or cmake would not pass when Imperx SDK is not found
	set(ImperX_INCLUDE_DIRS "")
endif()

# Check if camera SDKs were sourced
foreach(camera_sdk IN LISTS Ximea_FOUND ImperX_FOUND)
	if (NOT camera_sdk)
		set(CAMERA_SDKs_FOUND FALSE)

		# Unset any camera compile time defs
#		add_compile_definitions(USE_XIMEA_SDK=false)
#		add_compile_definitions(USE_IMPERX_SDK=false)

		break()
	else()
		set(CAMERA_SDKs_FOUND TRUE)
	endif()
endforeach()

# OpenCV 
if(NOT OpenCV_FOUND)
	find_package(OpenCV 4.2 REQUIRED COMPONENTS core)
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
if (CAMERA_SDKs_FOUND AND WIN32)
	set(
	PROJECT_SOURCES
		${CMAKE_CURRENT_SOURCE_DIR}/Adapters/GenICamInterface/GenICamAdapterInterface.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/Adapters/ImperX/ImperxAdapter.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/Adapters/Ximea/XimeaAdapter.cpp
		#
		${CMAKE_CURRENT_SOURCE_DIR}/ConfigParser/InterfaceJSONParser.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/Controllers/PanTilt/PanTiltController.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/Controllers/Camera/CameraInterface.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/Network/Network.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/Threads/DeviceInterface/DeviceInterface.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/Threads/ImperX/ImperxThread.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/Threads/PanTilt/PanTiltThread.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/Threads/Ximea/XimeaThread.cpp

		# FIXME - remove
		${CMAKE_CURRENT_SOURCE_DIR}/PhotoAlbum/PhotoAlbum.cpp
	)
else()
	if (WIN32)
		message(WARNING "WARNING: Camera SDKs were not found. GenICam will not be linked to project. Camera status: (DISABLED)")
	else()
		message(WARNING "WARNING: Target OS - ${CMAKE_SYSTEM_NAME} - not supported by GenICam. Camera status: (DISABLED)")
	endif()

	set(
	PROJECT_SOURCES
		${CMAKE_CURRENT_SOURCE_DIR}/ConfigParser/InterfaceJSONParser.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/Controllers/PanTilt/PanTiltController.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/Controllers/Camera/CameraInterface.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/Network/Network.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/Threads/DeviceInterface/DeviceInterface.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/Threads/ImperX/ImperxThread.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/Threads/PanTilt/PanTiltThread.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/Threads/Ximea/XimeaThread.cpp

		# FIXME - remove
		${CMAKE_CURRENT_SOURCE_DIR}/PhotoAlbum/PhotoAlbum.cpp
	)
   set(Ximea_INCLUDE_DIRS "")
   set(ImperX_INCLUDE_DIRS "")

endif()

add_library(
        ${PROJECT_NAME} STATIC ${PROJECT_SOURCES} 
)


# Set target properties
target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
			${PROJECT_SOURCE_DIR}
			${Ximea_INCLUDE_DIRS}
			${ImperX_INCLUDE_DIRS}
			${OpenCV_INCLUDE_DIRS}
		PRIVATE

)

target_link_libraries(
        ${PROJECT_NAME}
		PUBLIC
			${Ximea_LIBRARIES}
			${ImperX_LIBRARIES}
			 
		PRIVATE
			${OpenCV_LIBS}
			Models
			Utilities		
)

message(STATUS ======================)
message(STATUS "${PROJECT_NAME} - FINISHED")
message(STATUS ======================)
message(STATUS)
