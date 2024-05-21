cmake_minimum_required(VERSION 3.1)

set(CMAKE_BUILD_TYPE "Release")
 
set(CMAKE_CXX_FLAGS "-Wl,-rpath=./")
#set(CMAKE_CUDA_ARCHITECTURES "50")

project(LUMOS CXX CUDA)

find_package(Threads)
find_package(CUDA REQUIRED)
set(USER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../")
set(OPENCV_DIR ${USER_DIR}/lumos/3rdparty)

set(JETSON_MULTILIB_DIR /usr/src/jetson_multimedia_api/)
set(CLASS_DIR ${JETSON_MULTILIB_DIR}/samples/common/classes)
set(ALGO_CUDA_DIR ${JETSON_MULTILIB_DIR}/samples/common/algorithm/cuda)
set(ALGO_TRT_DIR ${JETSON_MULTILIB_DIR}/samples/common/algorithm/trt)
set(TEGRA_ARMABI aarch64-linux-gnu)
set(CUDA_PATH /usr/local/cuda)
set(ARGUS_UTILS_DIR ${JETSON_MULTILIB_DIR}/argus/samples/utils)


message(${PROJECT_SOURCE_DIR})

include_directories(${JETSON_MULTILIB_DIR}/include)
include_directories(${JETSON_MULTILIB_DIR}/include/libjpeg-8b)
include_directories(${ALGO_CUDA_DIR})
include_directories(${ALGO_TRT_DIR})
include_directories(${CUDA_PATH}/include)
include_directories(/usr/include/${TEGRA_ARMABI})
include_directories(/usr/include/libdrm)
include_directories(${ARGUS_UTILS_DIR})


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
include_directories(${OPENCV_DIR}/linux-opencv-4.4.0/include/opencv2)

link_directories(/lib/${TEGRA_ARMABI})
link_directories(/usr/lib/${TEGRA_ARMABI})
link_directories(/usr/lib/${TEGRA_ARMABI}/tegra)
link_directories(${CUDA_PATH}/lib64)

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
		            camera_mipi.cpp
                    merge_hdr.cu
                    projector_base.cpp
                    projector_ainstec.cpp
                    serial_function.cpp
                    camera_mipi_virtual.cpp
                    spi.cpp
                    devmem.cpp
                    projector_fpga.cpp
                    ${ARGUS_UTILS_DIR}/Thread.cpp
                    ${CLASS_DIR}/NvApplicationProfiler.cpp  ${CLASS_DIR}/NvElementProfiler.cpp  ${CLASS_DIR}/NvV4l2Element.cpp
                    ${CLASS_DIR}/NvBuffer.cpp               ${CLASS_DIR}/NvJpegDecoder.cpp      ${CLASS_DIR}/NvV4l2ElementPlane.cpp
                    ${CLASS_DIR}/NvDrmRenderer.cpp          ${CLASS_DIR}/NvJpegEncoder.cpp      ${CLASS_DIR}/NvVideoConverter.cpp
                    ${CLASS_DIR}/NvEglRenderer.cpp          ${CLASS_DIR}/NvLogging.cpp          ${CLASS_DIR}/NvVideoDecoder.cpp
                    ${CLASS_DIR}/NvElement.cpp              ${CLASS_DIR}/NvUtils.cpp            ${CLASS_DIR}/NvVideoEncoder.cpp
                    )
target_link_libraries(camera_server PRIVATE Threads::Threads
pthread v4l2 EGL GLESv2 X11 nvbuf_utils nvjpeg nvosd drm cuda cudart nveglstream_camconsumer nvargus_socketclient ${OpenCV_LIBS})

