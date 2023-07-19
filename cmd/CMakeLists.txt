cmake_minimum_required(VERSION 3.4.1)

add_compile_options(-fPIC)
add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "-Wl,-rpath=./")
project(lumos_cmd CXX)

include_directories(${OpenCV_INCLUDE_DIRS}  ../test ../sdk)

set(APP_SRC cmd.cpp getopt.c ../SDK/socket_tcp.cpp ../SDK/utils.cpp ../firmware/easylogging++.cc ../gui/PrecisionTest.cpp ../gui/calibrate_function.cpp)

link_libraries(${PROJECT_SOURCE_DIR}/../example/liblaser_3d_cam_sdk.so)
#print message
message(${PROJECT_SOURCE_DIR})   

 
find_package( OpenCV REQUIRED ) 
add_executable(${PROJECT_NAME} ${APP_SRC}) 

find_package(Threads)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} Threads::Threads)
