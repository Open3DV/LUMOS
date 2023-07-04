cmake_minimum_required(VERSION 3.1)

set(CMAKE_BUILD_TYPE "Release")
 
set(CMAKE_CXX_FLAGS "-Wl,-rpath=./")
#set(CMAKE_CUDA_ARCHITECTURES "50")

project(DF8 CXX CUDA)

find_package(Threads)
find_package(CUDA REQUIRED) 
set(OPENCV_DIR /home/ainstec/lumos/3rdparty)

message(${PROJECT_SOURCE_DIR})

include_directories(${OPENCV_DIR}/linux-opencv-4.4.0/include)
include_directories(${OPENCV_DIR}/linux-opencv-4.4.0/modules/core/include)
include_directories(${OPENCV_DIR}/linux-opencv-4.4.0/modules/highgui/include)
include_directories(${OPENCV_DIR}/linux-opencv-4.4.0/modules/imgproc/include)
include_directories(${OPENCV_DIR}/linux-opencv-4.4.0/modules/imgcodecs/include)
include_directories(${OPENCV_DIR}/linux-opencv-4.4.0/modules/calib3d/include)
include_directories(${OPENCV_DIR}/linux-opencv-4.4.0/modules/features2d/include)
include_directories(${OPENCV_DIR}/linux-opencv-4.4.0/modules/videoio/include)
include_directories(${OPENCV_DIR}/linux-opencv-4.4.0/modules/flann/include)
include_directories(${OPENCV_DIR}/linux-opencv-4.4.0/modules/dnn/include)
include_directories(${OPENCV_DIR}/linux-opencv-4.4.0/modules/ml/include)
include_directories(${OPENCV_DIR}/linux-opencv-4.4.0/modules/objdetect/include)
include_directories(${OPENCV_DIR}/linux-opencv-4.4.0/modules/photo/include)
include_directories(${OPENCV_DIR}/linux-opencv-4.4.0/modules/stitching/include)
include_directories(${OPENCV_DIR}/linux-opencv-4.4.0/modules/video/include)

link_directories(${OPENCV_DIR}/linux-opencv-4.4.0/lib)
link_libraries(${OPENCV_DIR}/linux-opencv-4.4.0/lib/libopencv_core.so.4.4)
link_libraries(${OPENCV_DIR}/linux-opencv-4.4.0/lib/libopencv_highgui.so.4.4)
link_libraries(${OPENCV_DIR}/linux-opencv-4.4.0/lib/libopencv_imgcodecs.so.4.4)
link_libraries(${OPENCV_DIR}/linux-opencv-4.4.0/lib/libopencv_imgproc.so.4.4)
link_libraries(${OPENCV_DIR}/linux-opencv-4.4.0/lib/libopencv_calib3d.so.4.4)
link_libraries(${OPENCV_DIR}/linux-opencv-4.4.0/lib/libopencv_features2d.so.4.4)
link_libraries(${OPENCV_DIR}/linux-opencv-4.4.0/lib/libopencv_videoio.so.4.4)
link_libraries(${OPENCV_DIR}/linux-opencv-4.4.0/lib/libopencv_flann.so.4.4)
link_libraries(${OPENCV_DIR}/linux-opencv-4.4.0/lib/libopencv_dnn.so.4.4)
link_libraries(${OPENCV_DIR}/linux-opencv-4.4.0/lib/libopencv_ml.so.4.4)
link_libraries(${OPENCV_DIR}/linux-opencv-4.4.0/lib/libopencv_objdetect.so.4.4)
link_libraries(${OPENCV_DIR}/linux-opencv-4.4.0/lib/libopencv_photo.so.4.4)
link_libraries(${OPENCV_DIR}/linux-opencv-4.4.0/lib/libopencv_stitching.so.4.4)
link_libraries(${OPENCV_DIR}/linux-opencv-4.4.0/lib/libopencv_video.so.4.4)

include_directories(${OPENCV_DIR}/JetsonGPIO/include)
link_directories(${OPENCV_DIR}/JetsonGPIO/lib)
link_libraries(${OPENCV_DIR}/JetsonGPIO/lib/libJetsonGPIO.so.1)

include_directories(/usr/local/cuda-10.2/targets/aarch64-linux/include/)

include_directories(/opt/MVS/include) 
link_libraries(/opt/MVS/lib/aarch64/libMvCameraControl.so)

add_executable(camera_server
                    camera_server.cpp 
                    encode_cuda.cu
                    management.cu
                    system_config_settings.cpp  
                    easylogging++.cc
                    socket_tcp.cpp
                    camera_mvs.cpp
                    scan3d.cpp
                    camera.cpp
                    merge_hdr.cu
                    projector_base.cpp
                    projector_ainstec.cpp
                    serial_function.cpp
                    )
target_link_libraries(camera_server PRIVATE Threads::Threads)

