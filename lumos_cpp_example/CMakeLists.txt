cmake_minimum_required(VERSION 3.4.1)

add_compile_options(-fPIC)
#add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "-Wl,-rpath=./")

project(example CXX)

set(APP_SRC cpp_example.cpp)


#print message
message(${PROJECT_SOURCE_DIR})   


link_libraries(${PROJECT_SOURCE_DIR}/liblumos_sdk.so)  
link_libraries(${PROJECT_SOURCE_DIR}/libenumerate.so)  
add_executable(${PROJECT_NAME} ${APP_SRC}) 
