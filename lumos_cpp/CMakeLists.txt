cmake_minimum_required(VERSION 3.4.1)

add_compile_options(-fPIC)
add_definitions(-std=c++11)


set(CMAKE_CXX_FLAGS "-Wl,-rpath=./")

 

project(lumos_sdk CXX)
find_package(Threads)

add_library(lumos_sdk SHARED lumos_camera.cpp ../test/triangulation.cpp ../SDK/socket_tcp.cpp ../firmware/easylogging++.cc) 
target_link_libraries(lumos_sdk PRIVATE
 Threads::Threads)
