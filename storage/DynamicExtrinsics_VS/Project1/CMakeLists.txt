cmake_minimum_required(VERSION 3.5)

project(Project1)

set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

###################################################
# Set CMAKE_MODULE_PATH so custom make files
#   can be found.
###################################################
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../Metrology2020_VS/Metrology2020/cmake)
message(STATUS "CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}")

find_package(OpenCV REQUIRED)

if(WIN32)
    find_package(Ximea REQUIRED)
    find_package(ImperX REQUIRED)
    # need to figure out how to do this:
    #find_package(Eigen3 REQUIRED)
    set(EIGEN3_INCLUDE_DIR C:/OpenCV/Eigen3/install/include/eigen3)
else()
    # this is only temporary: pointing to Mason's private installation of Eigen3 3.3.90
    set(EIGEN3_INCLUDE_DIR /home.shared/mruren/linux_libs/eigen/install/include/eigen3)
endif()
message(STATUS "EIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}")


set(METROLOGY_HOME ${PROJECT_SOURCE_DIR}/../../Metrology2020_VS/Metrology2020/)

if(WIN32)
set(METROLOGY_SRCS
        ${METROLOGY_HOME}/Adapters/GenICamInterface/GenICamAdapterInterface.cpp
        ${METROLOGY_HOME}/Adapters/ImperX/ImperxAdapter.cpp
        ${METROLOGY_HOME}/Adapters/Ximea/XimeaAdapter.cpp
        ${METROLOGY_HOME}/Controllers/PanTilt/PanTiltController.cpp
        ${METROLOGY_HOME}/Controllers/Camera/CameraInterface.cpp
        ${METROLOGY_HOME}/Threads/ImperX/ImperxThread.cpp
        ${METROLOGY_HOME}/Threads/Interface/ThreadInterface.cpp
        ${METROLOGY_HOME}/Threads/PanTilt/PanTiltThread.cpp
        ${METROLOGY_HOME}/Threads/Ximea/XimeaThread.cpp
        ${METROLOGY_HOME}/Utilities/ErrorHandler/ErrorHandler.cpp
        ${METROLOGY_HOME}/Utilities/Logger/Logger.cpp
        ${METROLOGY_HOME}/Utilities/Network/Network.cpp
        ${METROLOGY_HOME}/Utilities/PhotoAlbum/PhotoAlbum.cpp
)
endif()

include_directories(
                ${METROLOGY_HOME}/CommonIncludes  #need nlohmann/json.hpp
                ${METROLOGY_HOME}/Algorithms/     #Models/PanTilt/
                ${OpenCV_INCLUDE_DIRS} # Not needed for CMake >= 2.8.11
                ${EIGEN3_INCLUDE_DIR}
                # Camera/PTU interface required includes:
                ${METROLOGY_HOME}
                ${Ximea_INCLUDE_DIRS}
                ${ImperX_INCLUDE_DIRS}
                )

add_executable(${PROJECT_NAME} 
               main.cpp PoseEstimator.cpp
               ${METROLOGY_SRCS}
               )

target_link_libraries(${PROJECT_NAME} 
                      ${OpenCV_LIBS}
                      ${Ximea_LIBRARIES}
                      ${ImperX_LIBRARIES}
                      )
