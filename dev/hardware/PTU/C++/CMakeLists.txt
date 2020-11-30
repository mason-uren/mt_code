cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(PanTiltController VERSION 0.1 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS -pthread)

add_executable(PanTiltController main.cpp PanTiltController.cpp)

find_library(ZMQ required)
find_package(Boost COMPONENTS thread)

if(WIN32)
    target_link_libraries(PanTiltController Ws2_32 Mswsock AdvApi32)
endif()

target_link_libraries(${PROJECT_NAME}
        zmq
        ${Boost_LIBRARIES}
        )