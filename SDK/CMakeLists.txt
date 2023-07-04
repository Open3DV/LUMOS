cmake_minimum_required(VERSION 3.4.1)

add_compile_options(-fPIC)
add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "-Wl,-rpath=./")
 

project(laser_3d_cam_sdk CXX)
find_package(Threads)

add_library(laser_3d_cam_sdk SHARED laser_3d_cam.cpp ../test/triangulation.cpp socket_tcp.cpp utils.cpp ../firmware/easylogging++.cc utils.cpp) 
