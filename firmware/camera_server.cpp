#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <cassert>
#include "protocol.h"
#include <random>
#include <time.h>
#include <mutex>
#include <thread>
#include "easylogging++.h"
#include "system_config_settings.h"
#include "scan3d.h"
#include "socket_tcp.h"
#include <dirent.h>
#include "version.h"

INITIALIZE_EASYLOGGINGPP

Scan3D scan3d_;

std::random_device rd;
std::mt19937 rand_num(rd());
bool connected = false;
long long current_token = 0;

// heart beat
unsigned int heartbeat_timer = 0;
std::mutex mtx_heartbeat_timer;
std::thread heartbeat_thread;

struct CameraCalibParam param;
struct CameraCalibParam user_calib_param;// 极线矫正之后的相机标定参数

int brightness_current = 100;
float generate_brightness_exposure_time = 12000;
int generate_brightness_model = 1;

float max_camera_exposure_ = 100000;
float min_camera_exposure_ = 1000;

int camera_width_ = 0;
int camera_height_ = 0;
int rgb_camera_width_ = 0;
int rgb_camera_height_ = 0;

SystemConfigDataStruct system_config_settings_machine_;

bool readSystemConfig()
{
    return system_config_settings_machine_.loadFromSettings("../system_config.ini");
}

bool saveSystemConfig()
{
    return system_config_settings_machine_.saveToSettings("../system_config.ini");
}

int heartbeat_check()
{
    while (connected)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        mtx_heartbeat_timer.lock();
        heartbeat_timer ++;
        if (heartbeat_timer > 30)
        {
            LOG(INFO) << "HeartBeat stopped!";
            connected = false;
            current_token = 0;
        }
        mtx_heartbeat_timer.unlock();
    }

    return 0;
}

long long generate_token()
{
    long long token = rand_num();
    return token;
}




int handle_cmd_connect(int client_sock)
{
    int ret;
    if (connected)
    {
        LOG(INFO) << "new connection rejected" << std::endl;
        return send_command(client_sock, DF_CMD_REJECT);
    }
    else
    {
        ret = send_command(client_sock, DF_CMD_OK);
        if (ret == DF_FAILED)
        {

            LOG(INFO) << "send_command FAILED";
            return DF_FAILED;
        }
        long long token = generate_token();
        ret = send_buffer(client_sock, (char*)&token, sizeof(token));
        if (ret == DF_FAILED)
        {
            return DF_FAILED;
        }
        connected = true;
        current_token = token;

        mtx_heartbeat_timer.lock();
        heartbeat_timer = 0;
        mtx_heartbeat_timer.unlock();

        if (heartbeat_thread.joinable())
        {
            heartbeat_thread.join();
        }
        heartbeat_thread = std::thread(heartbeat_check);

        LOG(INFO) << "connection established, current token is: " << current_token;
        return DF_SUCCESS;
    }
}

int handle_cmd_unknown(int client_sock)
{
    long long token = 0;
    int ret = recv_buffer(client_sock, (char*)&token, sizeof(token));
    //std::cout<<"token ret = "<<ret<<std::endl;
    //std::cout<<"checking token:"<<token<<std::endl;
    if (ret == DF_FAILED)
    {
        return DF_FAILED;
    }

    if (token == current_token)
    {
        ret = send_command(client_sock, DF_CMD_UNKNOWN);

        if (ret == DF_FAILED)
        {
            LOG(INFO) << "send_command FAILED";
            return DF_FAILED;
        }
        else
        {
            return DF_SUCCESS;
        }
    }
    else
    {
        LOG(INFO) << "reject" << std::endl;
        ret = send_command(client_sock, DF_CMD_REJECT);
        if (ret == DF_FAILED)
        {
            LOG(INFO) << "send_command FAILED";
            return DF_FAILED;
        }
        else
        {
            return DF_SUCCESS;
        }
    }
}

int check_token(int client_sock)
{
    long long token = 0;
    int ret = recv_buffer(client_sock, (char*)&token, sizeof(token));
    // std::cout<<"token ret = "<<ret<<std::endl;
    // std::cout<<"checking token:"<<token<<std::endl;
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "recv_buffer token FAILED";
        return DF_FAILED;
    }

    if (token == current_token)
    {
        ret = send_command(client_sock, DF_CMD_OK);
        if (ret == DF_FAILED)
        {
            LOG(INFO) << "send_command FAILED";
            return DF_FAILED;
        }
        return DF_SUCCESS;
    }
    else
    {
        LOG(INFO) << "reject" << std::endl;
        ret = send_command(client_sock, DF_CMD_REJECT);
        if (ret == DF_FAILED)
        {
            LOG(INFO) << "send_command FAILED";
            return DF_FAILED;
        }
        return DF_FAILED;
    }
}

int handle_cmd_disconnect(int client_sock)
{
    LOG(INFO) << "handle_cmd_disconnect";
    long long token = 0;
    int ret = recv_buffer(client_sock, (char*)&token, sizeof(token));
    LOG(INFO) << "token " << token << " trying to disconnect";
    if (ret == DF_FAILED)
    {
        return DF_FAILED;
    }
    if (token == current_token)
    {
        connected = false;
        current_token = 0;
        LOG(INFO) << "client token=" << token << " disconnected";
        ret = send_command(client_sock, DF_CMD_OK);
        if (ret == DF_FAILED)
        {
            LOG(INFO) << "send_command FAILED";
            return DF_FAILED;
        }
    }
    else
    {
        LOG(INFO) << "disconnect rejected" << std::endl;
        ret = send_command(client_sock, DF_CMD_REJECT);
        if (ret == DF_FAILED)
        {
            LOG(INFO) << "send_command FAILED";
        }

        return DF_FAILED;
    }
    return DF_SUCCESS;
}


int handle_cmd_get_raw_01(int client_sock)
{

    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int image_num = 36;

    int width = 0;
    int height = 0;

    scan3d_.getCameraResolution(width, height);

    int buffer_size = height * width * image_num;
    unsigned char* buffer = new unsigned char[buffer_size];

    scan3d_.captureRaw01(buffer);

    LOG(INFO) << "start send image, buffer_size= " << buffer_size;
    int ret = send_buffer(client_sock, (char*)buffer, buffer_size);

    LOG(INFO) << "ret= " << ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        delete[] buffer;
        return DF_FAILED;
    }

    LOG(INFO) << "image sent!";

    delete[] buffer;
    return DF_SUCCESS;

}

int handle_cmd_get_raw_02_16bit(int client_sock)
{

    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int image_num = 36;

    int width = 0;
    int height = 0;

    scan3d_.getCameraResolution(width, height);

    int buffer_size = height * width * image_num * sizeof(unsigned short);
    unsigned short* buffer = new unsigned short[buffer_size];

    scan3d_.captureRaw01_16bit(buffer);

    LOG(INFO) << "start send image, buffer_size= " << buffer_size;
    int ret = send_buffer(client_sock, (char*)buffer, buffer_size);

    LOG(INFO) << "ret= " << ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        delete[] buffer;
        return DF_FAILED;
    }

    LOG(INFO) << "image sent!";

    delete[] buffer;
    return DF_SUCCESS;

}

int handle_cmd_get_raw_03(int client_sock)
{

    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int image_num = 36;

    int width = 0;
    int height = 0;
    int rgb_width = 0;
    int rgb_height = 0;
    scan3d_.getCameraResolution(width, height);
    scan3d_.getRGBCameraResolution(rgb_width, rgb_height);

    int buffer_size = height * width * image_num + rgb_width * rgb_height * 3;
    unsigned char* buffer = new unsigned char[buffer_size];

    scan3d_.captureRaw03(buffer);

    LOG(INFO) << "start send image, buffer_size= " << buffer_size;
    int ret = send_buffer(client_sock, (char*)buffer, buffer_size);

    LOG(INFO) << "ret= " << ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        delete[] buffer;
        return DF_FAILED;
    }

    LOG(INFO) << "image sent!";

    delete[] buffer;
    return DF_SUCCESS;

}

int handle_cmd_get_frame_01_parallel(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int ret = DF_SUCCESS;

    int depth_buf_size = camera_width_ * camera_height_ * 4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_ * camera_height_ * 1;
    unsigned char* brightness = new unsigned char[brightness_buf_size];


    LOG(INFO) << "captureFrame08";
    ret = scan3d_.captureFrame08();
    if (DF_SUCCESS != ret)
    {
        LOG(ERROR) << "captureFrame01 code: " << ret;
    }

    LOG(INFO) << "Reconstruct captureFrame01 Finished!";
    scan3d_.copyBrightnessData(brightness);
    scan3d_.copyDepthData(depth_map);

    LOG(INFO) << "copy depth";

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral"; 
    }

    LOG(INFO) << "start send depth, buffer_size= " << depth_buf_size;
    ret = send_buffer(client_sock, (const char*)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= " << ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }

    LOG(INFO) << "start send brightness, buffer_size= " << brightness_buf_size;
    ret = send_buffer(client_sock, (const char*)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= " << ret;

    LOG(INFO) << "Send Frame01";

    if (ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;


        return DF_FAILED;
    }
    LOG(INFO) << "frame sent!";
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;


    return DF_SUCCESS;
}

int handle_cmd_get_frame_04_parallel(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int ret = DF_SUCCESS;

    int depth_buf_size = camera_width_ * camera_height_ * 4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_ * camera_height_ * 1;
    unsigned char* brightness = new unsigned char[brightness_buf_size];

    int color_brightness_buf_size = rgb_camera_width_ * rgb_camera_height_ * 3;
    unsigned char* color_brightness = new unsigned char[color_brightness_buf_size];

    LOG(INFO) << "captureFrame04";
    ret = scan3d_.captureFrame04();
    if (DF_SUCCESS != ret)
    {
        LOG(ERROR) << "captureFrame04 code: " << ret;
    }

    LOG(INFO) << "Reconstruct captureFrame04 Finished!";
    scan3d_.copyBrightnessData(brightness);
    scan3d_.copyColorBrightnessData(color_brightness);
    scan3d_.copyDepthData(depth_map);

    LOG(INFO) << "copy depth";

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral"; 
    }

    LOG(INFO) << "start send depth, buffer_size= " << depth_buf_size;
    ret = send_buffer(client_sock, (const char*)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= " << ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;
        delete[] color_brightness;

        return DF_FAILED;
    }

    LOG(INFO) << "start send brightness, buffer_size= " << brightness_buf_size;
    ret = send_buffer(client_sock, (const char*)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= " << ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;
        delete[] color_brightness;

        return DF_FAILED;
    }

    LOG(INFO) << "start send color_brightness, buffer_size= " << color_brightness_buf_size;
    ret = send_buffer(client_sock, (const char*)color_brightness, color_brightness_buf_size);
    LOG(INFO) << "color_brightness ret= " << ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;
        delete[] color_brightness;

        return DF_FAILED;
    }
    
    LOG(INFO) << "frame sent!";
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;

    delete[] color_brightness;

    return DF_SUCCESS;
}

int handle_cmd_get_frame_03_parallel(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int ret = DF_SUCCESS;

    int depth_buf_size = camera_width_ * camera_height_ * 4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_ * camera_height_ * 1;
    unsigned char* brightness = new unsigned char[brightness_buf_size];

    int color_brightness_buf_size = rgb_camera_width_ * rgb_camera_height_ * 3;
    unsigned char* color_brightness = new unsigned char[color_brightness_buf_size];

    int depth2color_buf_size = camera_width_ * camera_height_ * sizeof(unsigned short) * 2;
    unsigned short* depth2color_map = (unsigned short*)(new unsigned char[depth2color_buf_size]);

    LOG(INFO) << "captureFrame03";
    ret = scan3d_.captureFrame03();
    if (DF_SUCCESS != ret)
    {
        LOG(ERROR) << "captureFrame03 code: " << ret;
    }

    LOG(INFO) << "Reconstruct captureFrame03 Finished!";
    scan3d_.copyBrightnessData(brightness);
    scan3d_.copyColorBrightnessData(color_brightness);
    scan3d_.copyDepthData(depth_map);
    scan3d_.copyDepth2ColorData(depth2color_map);

    LOG(INFO) << "copy depth";

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral"; 
    }

    LOG(INFO) << "start send depth, buffer_size= " << depth_buf_size;
    ret = send_buffer(client_sock, (const char*)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= " << ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;
        delete[] depth2color_map;
        delete[] color_brightness;

        return DF_FAILED;
    }

    LOG(INFO) << "start send brightness, buffer_size= " << brightness_buf_size;
    ret = send_buffer(client_sock, (const char*)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= " << ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;
        delete[] depth2color_map;
        delete[] color_brightness;

        return DF_FAILED;
    }

    LOG(INFO) << "start send color_brightness, buffer_size= " << color_brightness_buf_size;
    ret = send_buffer(client_sock, (const char*)color_brightness, color_brightness_buf_size);
    LOG(INFO) << "color_brightness ret= " << ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;
        delete[] depth2color_map;
        delete[] color_brightness;

        return DF_FAILED;
    }

    LOG(INFO) << "start send depth2color, buffer_size= " << depth2color_buf_size;
    ret = send_buffer(client_sock, (const char*)depth2color_map, depth2color_buf_size);
    LOG(INFO) << "depth2color ret= " << ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;
        delete[] depth2color_map;
        delete[] color_brightness;

        return DF_FAILED;
    }
    LOG(INFO) << "frame sent!";
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;
    delete[] depth2color_map;
    delete[] color_brightness;

    return DF_SUCCESS;
}

int handle_cmd_get_frame_01_hdr_parallel(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int ret = DF_SUCCESS;

    int depth_buf_size = camera_width_ * camera_height_ * 4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_ * camera_height_ * 1;
    unsigned char* brightness = new unsigned char[brightness_buf_size];


    LOG(INFO) << "captureFrame08HDR";
    ret = scan3d_.captureFrame08MixedHDR();
    if (DF_SUCCESS != ret)
    {
        LOG(ERROR) << "captureFrame08HDR code: " << ret;
    }

    LOG(INFO) << "Reconstruct captureFrame08HDR Finished!";
    scan3d_.copyBrightnessData(brightness);
    scan3d_.copyDepthData(depth_map);


    LOG(INFO) << "copy depth";

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral"; 
    }

    LOG(INFO) << "start send depth, buffer_size= " << depth_buf_size;
    ret = send_buffer(client_sock, (const char*)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= " << ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }

    LOG(INFO) << "start send brightness, buffer_size= " << brightness_buf_size;
    ret = send_buffer(client_sock, (const char*)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= " << ret;

    LOG(INFO) << "Send Frame01HDR";

    if (ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;


        return DF_FAILED;
    }
    LOG(INFO) << "frame sent!";
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;


    return DF_SUCCESS;
}

int handle_cmd_get_frame_04_hdr_parallel(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int ret = DF_SUCCESS;

    int depth_buf_size = camera_width_ * camera_height_ * 4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_ * camera_height_ * 1;
    unsigned char* brightness = new unsigned char[brightness_buf_size];

    int color_brightness_buf_size = rgb_camera_width_ * rgb_camera_height_ * 3;
    unsigned char* color_brightness = new unsigned char[color_brightness_buf_size];


    LOG(INFO) << "captureFrame04HDR";
    ret = scan3d_.captureFrame04MixedHDR();
    if (DF_SUCCESS != ret)
    {
        LOG(ERROR) << "captureFrame04HDR code: " << ret;
    }

    LOG(INFO) << "Reconstruct captureFrame04HDR Finished!";
    scan3d_.copyBrightnessData(brightness);
    scan3d_.copyColorBrightnessData(color_brightness);
    scan3d_.copyDepthData(depth_map);


    LOG(INFO) << "copy depth";

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral"; 
    }

    LOG(INFO) << "start send depth, buffer_size= " << depth_buf_size;
    ret = send_buffer(client_sock, (const char*)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= " << ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;
        delete[] color_brightness;

        return DF_FAILED;
    }

    LOG(INFO) << "start send brightness, buffer_size= " << brightness_buf_size;
    ret = send_buffer(client_sock, (const char*)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= " << ret;

    if (ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;
        delete[] color_brightness;

        return DF_FAILED;
    }

    LOG(INFO) << "start send color_brightness, buffer_size= " << color_brightness_buf_size;
    ret = send_buffer(client_sock, (const char*)color_brightness, color_brightness_buf_size);
    LOG(INFO) << "color_brightness ret= " << ret;

    LOG(INFO) << "Send Frame04HDR";

    if (ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;
        delete[] color_brightness;

        return DF_FAILED;
    }

    LOG(INFO) << "frame sent!";
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;
    delete[] color_brightness;

    return DF_SUCCESS;
}

int handle_cmd_get_frame_test_parallel(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int ret = DF_SUCCESS;

    int depth_buf_size = camera_width_ * camera_height_ * 4;
    float* depth_map = new float[depth_buf_size];

    int brightness_buf_size = camera_width_ * camera_height_ * 1;
    unsigned char* brightness = new unsigned char[brightness_buf_size];

    int patterns_buf_size = camera_width_ * camera_height_ * 28;
    unsigned char* patterns = new unsigned char[patterns_buf_size];


    LOG(INFO) << "captureFrameTest";
    ret = scan3d_.captureFrameTest(patterns);
    if (DF_SUCCESS != ret)
    {
        LOG(ERROR) << "captureFrameTest code: " << ret;
    }

    LOG(INFO) << "Reconstruct FrameTest Finished!";
    scan3d_.copyBrightnessData(brightness);
    scan3d_.copyDepthData(depth_map);


    LOG(INFO) << "copy depth";

    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter)
    { 
        cv::Mat depth_mat(camera_height_, camera_width_, CV_32FC1, depth_map);
        cv::Mat depth_bilateral_mat(camera_height_, camera_width_, CV_32FC1, cv::Scalar(0));
        cv::bilateralFilter(depth_mat, depth_bilateral_mat, system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d, 2.0, 10.0); 
        memcpy(depth_map,(float*)depth_bilateral_mat.data,depth_buf_size);
        LOG(INFO) << "Bilateral"; 
    }

    LOG(INFO) << "start send depth, buffer_size= " << depth_buf_size;
    ret = send_buffer(client_sock, (const char*)depth_map, depth_buf_size);
    LOG(INFO) << "depth ret= " << ret;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!";
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;

        return DF_FAILED;
    }

    LOG(INFO) << "start send brightness, buffer_size= " << brightness_buf_size;
    ret = send_buffer(client_sock, (const char*)brightness, brightness_buf_size);
    LOG(INFO) << "brightness ret= " << ret;

    if (ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;
        delete[] patterns;

        return DF_FAILED;
    }

    LOG(INFO) << "start send patterns, buffer_size= " << patterns_buf_size;
    ret = send_buffer(client_sock, (const char*)patterns, patterns_buf_size);
    LOG(INFO) << "patterns ret= " << ret;

    if (ret == DF_FAILED)
    {
        printf("send error, close this connection!\n");
        // delete [] buffer;
        delete[] depth_map;
        delete[] brightness;
        delete[] patterns;


        return DF_FAILED;
    }

    LOG(INFO) << "frame sent!";
    // delete [] buffer;
    delete[] depth_map;
    delete[] brightness;
    delete[] patterns;

    return DF_SUCCESS;
}

int handle_heartbeat(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    mtx_heartbeat_timer.lock();
    heartbeat_timer = 0;
    mtx_heartbeat_timer.unlock();

    return DF_SUCCESS;
}

int read_calib_param()
{
    std::ifstream ifile;

    ifile.open("calib_param.txt");

    if (!ifile.is_open())
    {
        return DF_FAILED;
    }

    int n_params = sizeof(param) / sizeof(float);
    for (int i = 0; i < n_params; i++)
    {
        ifile >> (((float*)(&param))[i]);
    }
    ifile.close();
    return DF_SUCCESS;
}

int read_calib_param_user()
{
    std::ifstream ifile;

    ifile.open("calib_param_user.txt");

    if (!ifile.is_open())
    {
        return DF_FAILED;
    }

    int n_params = sizeof(user_calib_param) / sizeof(float);
    for (int i = 0; i < n_params; i++)
    {
        ifile >> (((float*)(&user_calib_param))[i]);
    }
    ifile.close();
    return DF_SUCCESS;
}


int handle_get_camera_parameters(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    read_calib_param_user();

    int ret = send_buffer(client_sock, (char*)(&user_calib_param), sizeof(user_calib_param));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }
    return DF_SUCCESS;

}

/*****************************************************************************************/
//system config param 
int handle_get_system_config_parameters(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().config_param_), sizeof(system_config_settings_machine_.Instance().config_param_));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }
    return DF_SUCCESS;

}

bool set_system_config(SystemConfigParam& rect_config_param)
{

    //set led current
    if (rect_config_param.led_current != system_config_settings_machine_.Instance().config_param_.led_current)
    {
        if (0 <= rect_config_param.led_current && rect_config_param.led_current < 1024)
        {
            brightness_current = rect_config_param.led_current;

            system_config_settings_machine_.Instance().config_param_.led_current = brightness_current;

            scan3d_.setLaserBrightness(rect_config_param.led_current);
            scan3d_.setParamLaserCurrent(rect_config_param.led_current);
        }

    }

    //set many exposure param
    system_config_settings_machine_.Instance().config_param_.exposure_num = rect_config_param.exposure_num;
    std::memcpy(system_config_settings_machine_.Instance().config_param_.exposure_param, rect_config_param.exposure_param, sizeof(rect_config_param.exposure_param));


    //set external param

    std::memcpy(system_config_settings_machine_.Instance().config_param_.standard_plane_external_param, rect_config_param.standard_plane_external_param, sizeof(rect_config_param.standard_plane_external_param));

    return true;
}

int handle_set_system_config_parameters(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }



    SystemConfigParam rect_config_param;


    int ret = recv_buffer(client_sock, (char*)(&rect_config_param), sizeof(rect_config_param));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    bool ok = set_system_config(rect_config_param);

    if (!ok)
    {
        return DF_FAILED;
    }

    return DF_SUCCESS;

}

/**********************************************************************************************************************/

int handle_cmd_get_param_camera_gain(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    float send_gain;
    scan3d_.getParamGain(send_gain);

    int ret = send_buffer(client_sock, (char*)(&send_gain), sizeof(float));

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    return DF_SUCCESS;

}

int handle_cmd_get_param_camera_exposure(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    float camera_exposure;
    scan3d_.getParamExposure(camera_exposure);

    int ret = send_buffer(client_sock, (char*)&camera_exposure, sizeof(float));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    return DF_SUCCESS;

}

bool save_hdr_param_to_file(const char* param_path, int& hdr_count, int* exposure_time, int* brightness)
{
    std::string file_path(param_path);
    cv::FileStorage fs_out;

    cv::Mat brightness_list(5, 1, CV_32S, brightness);
    cv::Mat exposure_list(5, 1, CV_32S, exposure_time);

    fs_out.open(file_path, cv::FileStorage::WRITE);

    fs_out << "hdr_count" << hdr_count;
    fs_out << "hdr_exposure_list" << exposure_list;
    fs_out << "hdr_brightness_list" << brightness_list;
    fs_out.release();

    return true;
}

int handle_cmd_set_param_camera_gain(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }


    float gain = 0;

    int ret = recv_buffer(client_sock, (char*)(&gain), sizeof(float));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    if (gain < 0)
    {
        gain = 0;
    }


    if (scan3d_.setParamGain(gain))
    {
        system_config_settings_machine_.Instance().config_param_.camera_gain = gain;
        LOG(INFO) << "Set Camera Gain: " << gain;
    }
    else
    {
        LOG(INFO) << "Set Camera Gain Error!";
    }



    return DF_SUCCESS;
}

int handle_cmd_set_param_camera_gamma(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }


    float gamma = 0;

    int ret = recv_buffer(client_sock, (char*)(&gamma), sizeof(float));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    if (gamma < 0.1 || gamma > 2)
    {
        gamma = 1;
    }


    if (scan3d_.setParamCameraGamma(gamma))
    {
        system_config_settings_machine_.Instance().config_param_.camera_gamma = gamma;
        LOG(INFO) << "Set Camera Gamma: " << gamma;
    }
    else
    {
        LOG(INFO) << "Set Camera Gamma Error!";
    }



    return DF_SUCCESS;
}

int handle_cmd_get_param_camera_gamma(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    float send_gamma;
    scan3d_.getParamCameraGamma(send_gamma);

    int ret = send_buffer(client_sock, (char*)(&send_gamma), sizeof(float));

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    return DF_SUCCESS;
}

int handle_cmd_set_param_hdr(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }


    int hdr_count = 0;
    int exposure_list[5];
    int brightness_list[5];

    int ret = recv_buffer(client_sock, (char*)(&hdr_count), sizeof(int));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }
    ret = recv_buffer(client_sock, (char*)(&exposure_list), 5 * sizeof(int));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }
    ret = recv_buffer(client_sock, (char*)(&brightness_list), 5 * sizeof(int));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    const char HDR_param_path[100] = "./HDR_params.xml";

    save_hdr_param_to_file(HDR_param_path, hdr_count, exposure_list, brightness_list);

    return DF_SUCCESS;
}

//设置双边滤波参数
int handle_cmd_set_param_bilateral_filter(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }


    int param[2];

    int ret = recv_buffer(client_sock, (char*)(&param[0]), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
 

    ret = recv_buffer(client_sock, (char*)(&param[1]), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

    system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter = param[0];


    if (1 == param[0])
    {   
        if (3 == param[1] || 5 == param[1] || 7 == param[1] || 9 == param[1] || 11 == param[1])
        { 
        system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d = param[1];
        }
    }

    
    LOG(INFO)<<"Use Bilateral Filter: "<<system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter;
    LOG(INFO)<<"Bilateral Filter param: "<<system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d;
         

    return DF_SUCCESS;
}

//获取双边滤波参数
int handle_cmd_get_param_bilateral_filter(int client_sock)
{
   if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
     
    int ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().firwmare_param_.use_bilateral_filter), sizeof(int) );
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
 
    ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().firwmare_param_.bilateral_filter_param_d), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
 

    return DF_SUCCESS;
}

int handle_cmd_set_param_depth_filter(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }


    int switch_val = 0;
    float depth_throshold = 2;

    int ret = recv_buffer(client_sock, (char*)(&switch_val), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
 

    ret = recv_buffer(client_sock, (char*)(&depth_throshold), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }


    system_config_settings_machine_.Instance().firwmare_param_.depth_filter_threshold = depth_throshold;
    system_config_settings_machine_.Instance().firwmare_param_.use_depth_filter = switch_val;

 
    LOG(INFO)<<"use_depth_filter: "<<system_config_settings_machine_.Instance().firwmare_param_.use_depth_filter;
    LOG(INFO)<<"depth_filter_threshold: "<<system_config_settings_machine_.Instance().firwmare_param_.depth_filter_threshold;
         

    return DF_SUCCESS;
}

int handle_cmd_get_param_depth_filter(int client_sock)
{
   if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
     
    int ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().firwmare_param_.use_depth_filter), sizeof(int) );
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
 
    ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().firwmare_param_.depth_filter_threshold), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    LOG(INFO)<<"use_depth_filter: "<<system_config_settings_machine_.Instance().firwmare_param_.use_depth_filter;
    LOG(INFO)<<"depth_filter_threshold: "<<system_config_settings_machine_.Instance().firwmare_param_.depth_filter_threshold;
         

    return DF_SUCCESS;
}

int handle_cmd_set_param_camera_exposure(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }


    float exposure = 0;

    int ret = recv_buffer(client_sock, (char*)(&exposure), sizeof(float));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }



    if (scan3d_.setParamExposure(exposure))
    {
        LOG(INFO) << "Set Camera Exposure Time: " << exposure;

        system_config_settings_machine_.Instance().config_param_.camera_exposure_time = exposure;

    }
    else
    {
        LOG(INFO) << "Set Camera Exposure Time Error!";
    }



    return DF_SUCCESS;
}

int handle_cmd_get_param_generate_brightness(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }


    int ret = send_buffer(client_sock, (char*)(&scan3d_.generate_brightness_model_), sizeof(int));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    float exposure_temp = scan3d_.generate_brightness_exposure_;

    ret = send_buffer(client_sock, (char*)(&exposure_temp), sizeof(float));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    return DF_SUCCESS;
}

//�����������Ȳ���
int handle_cmd_set_param_generate_brightness(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int flag = 1;

    int ret = recv_buffer(client_sock, (char*)(&flag), sizeof(int));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    float exposure = 0;

    ret = recv_buffer(client_sock, (char*)(&exposure), sizeof(float));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }


    if (scan3d_.setParamGenerateBrightness(flag, exposure))
    {
        generate_brightness_model = flag;
        generate_brightness_exposure_time = exposure;

        LOG(INFO) << "generate_brightness_model: " << generate_brightness_model << "\n";
        LOG(INFO) << "generate_brightness_exposure_time: " << generate_brightness_exposure_time << "\n";
    }

    // camera.setGenerateBrightnessParam(generate_brightness_model,generate_brightness_exposure_time);


    return DF_SUCCESS;
}

int handle_cmd_get_param_camera_resolution(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int width = 0;
    int height = 0;

    scan3d_.getCameraResolution(width, height);

    // lc3010.read_dmd_device_id(version); 

    int ret = send_buffer(client_sock, (char*)(&width), sizeof(int) * 1);
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    ret = send_buffer(client_sock, (char*)(&height), sizeof(int) * 1);
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    LOG(INFO) << "camera width: " << width;
    LOG(INFO) << "camera height: " << height;

    return DF_SUCCESS;

}

int handle_cmd_get_param_rgb_camera_resolution(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int width = 0;
    int height = 0;

    scan3d_.getRGBCameraResolution(width, height);

    // lc3010.read_dmd_device_id(version); 

    int ret = send_buffer(client_sock, (char*)(&width), sizeof(int) * 1);
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    ret = send_buffer(client_sock, (char*)(&height), sizeof(int) * 1);
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    LOG(INFO) << "camera width: " << width;
    LOG(INFO) << "camera height: " << height;

    return DF_SUCCESS;

}

//设置置信度参数
int handle_cmd_set_param_confidence(int client_sock)
{

    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	  
    float val = 0; 
    int ret = recv_buffer(client_sock, (char*)(&val), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
    LOG(INFO) << "Set Confidence: "<<val;
    system_config_settings_machine_.Instance().firwmare_param_.confidence = val;
    // cuda_set_config(system_config_settings_machine_);

    if(!scan3d_.setParamConfidence(val))
    { 
        LOG(INFO)<<"Set Param Confidence Failed!";
    }

    return DF_SUCCESS;
}

//获取置信度参数
int handle_cmd_get_param_confidence(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }
 
    float confidence = system_config_settings_machine_.Instance().firwmare_param_.confidence;
 

    int ret = send_buffer(client_sock, (char *)(&confidence), sizeof(float) * 1);
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }
    return DF_SUCCESS;
}

/*************************************************************************************************************************/

int write_calib_param()
{
    std::ofstream ofile;
    ofile.open("calib_param.txt");
    int n_params = sizeof(param) / sizeof(float);
    for (int i = 0; i < n_params; i++)
    {
        ofile << (((float*)(&param))[i]) << std::endl;
    }
    ofile.close();
    return DF_SUCCESS;
}

int write_calib_param_user()
{
    std::ofstream ofile;
    ofile.open("calib_param_user.txt");
    int n_params = sizeof(user_calib_param) / sizeof(float);
    for (int i = 0; i < n_params; i++)
    {
        ofile << (((float*)(&user_calib_param))[i]) << std::endl;
    }
    ofile.close();
    return DF_SUCCESS;
}

int handle_set_camera_parameters(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int ret = recv_buffer(client_sock, (char*)(&param), sizeof(param));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }
    write_calib_param();

    return DF_SUCCESS;

}


void load_txt(std::string filename, char* info, int length)
{
    FILE* fr = fopen(filename.c_str(), "rb");

    if (fr != NULL) {
        fread(info, 1, length, fr);
        fclose(fr);
    }
    else {
        std::cout << "open file error" << std::endl;
    }
}

//设置光机投影亮度
int handle_cmd_set_param_led_current(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	  

    int led= -1;

    int ret = recv_buffer(client_sock, (char*)(&led), sizeof(led));
    if(ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    LOG(INFO) << "LED: " << led;

    // set led current

    if (0 <= led && led <= 1024)
    {

        brightness_current = led;

        if (true != scan3d_.setLaserBrightness(brightness_current))
        {
                LOG(ERROR) << "Set Led Current";

                return DF_FAILED;
        }
        system_config_settings_machine_.Instance().config_param_.led_current = brightness_current;

        scan3d_.setParamLaserCurrent(led);
        return DF_SUCCESS;
    }

    return DF_FAILED; 
}

//获取光机投影亮度
int handle_cmd_get_param_led_current(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
  
	
    int ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().config_param_.led_current), sizeof(system_config_settings_machine_.Instance().config_param_.led_current));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	return DF_FAILED;
    }
    return DF_SUCCESS;
 
       
}

//设置基准平面外参
int handle_cmd_set_param_standard_param_external(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	  
    float plane_param[12]; 

    int ret = recv_buffer(client_sock, (char*)(plane_param), sizeof(float)*12);
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }


	memcpy(system_config_settings_machine_.Instance().config_param_.standard_plane_external_param, plane_param, sizeof(float)*12);
 
 
    return DF_SUCCESS;
 
}


//获取基准平面外参
int handle_cmd_get_param_standard_param_external(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
   
	
    int ret = send_buffer(client_sock, (char*)(system_config_settings_machine_.Instance().config_param_.standard_plane_external_param), sizeof(float)*12);
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	return DF_FAILED;
    }
    return DF_SUCCESS;
 
       
}

int handle_cmd_get_param_projector_version(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int version = 0;

    int ret = send_buffer(client_sock, (char *)(&version), sizeof(int) * 1);
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }

    LOG(INFO)<<"laser version: "<<version << "\n";

    return DF_SUCCESS;

}

//设置混合多曝光参数
int handle_cmd_set_param_mixed_hdr(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
	LOG(INFO)<<"check_token finished!";
    int param[13]; 

    int ret = recv_buffer(client_sock, (char*)(&param), sizeof(int)*13);
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

    
	LOG(INFO)<<"recv_buffer param finished!";

    int num = param[0];  
      //set led current

    for (int i = 0; i < num; i++)
    {
        int exposure = param[1 + i];

        if (exposure > max_camera_exposure_)
        {
            exposure = max_camera_exposure_;
        }
        else if (exposure < min_camera_exposure_)
        {
            exposure = min_camera_exposure_;
        }
        param[1 + i] = exposure;
    }

    if(0< num && num<= 5)
    {

        system_config_settings_machine_.Instance().firwmare_param_.mixed_exposure_num = num;
        memcpy(system_config_settings_machine_.Instance().firwmare_param_.mixed_exposure_param_list, param + 1, sizeof(int) * 6);
        memcpy(system_config_settings_machine_.Instance().firwmare_param_.mixed_led_param_list, param + 7, sizeof(int) * 6);
        system_config_settings_machine_.Instance().firwmare_param_.hdr_model = 2;

        std::vector<int> led_current_list;
        std::vector<int> camera_exposure_list;

        for (int i = 0; i < 6; i++)
        {
            led_current_list.push_back(system_config_settings_machine_.Instance().firwmare_param_.mixed_led_param_list[i]);
            camera_exposure_list.push_back(system_config_settings_machine_.Instance().firwmare_param_.mixed_exposure_param_list[i]);
        }
        // 设置到成员变量
        scan3d_.setParamHdr(num, led_current_list, camera_exposure_list);
        // 保存到本地文件
        save_hdr_param_to_file("./HDR_params.xml", num, param + 1, param + 7);

        return DF_SUCCESS;
    }

    return DF_FAILED;
}

//获取混合多曝光参数
int handle_cmd_get_param_mixed_hdr(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }

    int param[13];
    param[0] = system_config_settings_machine_.Instance().firwmare_param_.mixed_exposure_num;

    memcpy(param + 1, system_config_settings_machine_.Instance().firwmare_param_.mixed_exposure_param_list, sizeof(int) * 6);
    memcpy(param + 7, system_config_settings_machine_.Instance().firwmare_param_.mixed_led_param_list, sizeof(int) * 6);

    int ret = send_buffer(client_sock, (char *)(&param), sizeof(int) * 13);
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }
    return DF_SUCCESS;
}

//设置半径滤波参数
int handle_cmd_set_param_radius_filter(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }


    int switch_val = 0;
    float radius = 2;
    int num = 3;

    int ret = recv_buffer(client_sock, (char*)(&switch_val), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
 

    ret = recv_buffer(client_sock, (char*)(&radius), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }
     
    ret = recv_buffer(client_sock, (char*)(&num), sizeof(int));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
    	return DF_FAILED;
    }

    system_config_settings_machine_.Instance().firwmare_param_.use_radius_filter = switch_val;
    system_config_settings_machine_.Instance().firwmare_param_.radius_filter_r = radius;
    system_config_settings_machine_.Instance().firwmare_param_.radius_filter_threshold_num = num;

 
    LOG(INFO)<<"use_radius_filter: "<<system_config_settings_machine_.Instance().firwmare_param_.use_radius_filter;
    LOG(INFO)<<"radius_filter_r: "<<system_config_settings_machine_.Instance().firwmare_param_.radius_filter_r;
    LOG(INFO)<<"radius_filter_threshold_num: "<<system_config_settings_machine_.Instance().firwmare_param_.radius_filter_threshold_num;
         

    return DF_SUCCESS;
}

int handle_cmd_get_param_radius_filter(int client_sock)
{
   if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }
     
    int ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().firwmare_param_.use_radius_filter), sizeof(int) );
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
 
    ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().firwmare_param_.radius_filter_r), sizeof(float));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }

    ret = send_buffer(client_sock, (char*)(&system_config_settings_machine_.Instance().firwmare_param_.radius_filter_threshold_num), sizeof(int) );
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }

    
    LOG(INFO)<<"use_radius_filter: "<<system_config_settings_machine_.Instance().firwmare_param_.use_radius_filter;
    LOG(INFO)<<"radius_filter_r: "<<system_config_settings_machine_.Instance().firwmare_param_.radius_filter_r;
    LOG(INFO)<<"radius_filter_threshold_num: "<<system_config_settings_machine_.Instance().firwmare_param_.radius_filter_threshold_num;
         

    return DF_SUCCESS;
}

float read_temperature(int flag)
{
    float val = -1.0;

    switch(flag)
    {
        case 0:
        {
            char data[100];
            std::ifstream infile;
            infile.open("/sys/class/thermal/thermal_zone0/temp");
            infile >> data;
            // std::cout << "first read data from file1.dat == " << data << std::endl;
 
            val = (float)std::atoi(data) / 1000.0; 

        }
        break;

        case 1:
        {
            char data[100];
            std::ifstream infile;
            infile.open("/sys/class/thermal/thermal_zone1/temp");
            infile >> data;
            
            val = (float)std::atoi(data) / 1000.0; 
        }
        break;

        case 2:
        {
            char data[100];
            std::ifstream infile;
            infile.open("/sys/class/thermal/thermal_zone2/temp");
            infile >> data;
            
            val =(float)std::atoi(data) / 1000.0; 
        }
        break;

        default:
        break;
    }

    return val;
}

int handle_get_temperature(int client_sock)
{
    if (check_token(client_sock) == DF_FAILED)
    {
        return DF_FAILED;
    }
    float temperature = read_temperature(0);
    LOG(INFO) << "CPU temperature:" << temperature;
    int ret = send_buffer(client_sock, (char *)(&temperature), sizeof(temperature));
    if (ret == DF_FAILED)
    {
        LOG(INFO) << "send error, close this connection!\n";
        return DF_FAILED;
    }
    return DF_SUCCESS;
}

int handle_get_firmware_version(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }

    LOG(INFO)<<"get firmware version!";

    char version[_VERSION_LENGTH_] = _VERSION_;
    int ret = send_buffer(client_sock, version, _VERSION_LENGTH_);
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    return DF_SUCCESS;
}

int handle_cmd_get_product_info(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED) {
	    return DF_FAILED;
    }

    LOG(INFO)<<"get product info!";

    char *info = new char[INFO_SIZE];
    memset(info, 0, INFO_SIZE);
    load_txt("../product_info.txt", info, INFO_SIZE);
	std::cout << "INFO:\n" << info << std::endl;

    int ret = send_buffer(client_sock, info, INFO_SIZE);    
    delete [] info;
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    return DF_SUCCESS;
}

int read_bandwidth()
{
    int val = 0;
    char data[100];

    std::ifstream infile;
    infile.open("/sys/class/net/eth0/speed");
    infile >> data;
    val = (int)std::atoi(data);
    infile.close();

    return val;
}

int handle_get_network_bandwidth(int client_sock)
{
    if(check_token(client_sock) == DF_FAILED)
    {
	    return DF_FAILED;
    }

    LOG(INFO)<<"get network bandwidth!";

    int speed = read_bandwidth();
    int ret = send_buffer(client_sock, (char*)(&speed), sizeof(speed));
    if(ret == DF_FAILED)
    {
        LOG(INFO)<<"send error, close this connection!\n";
	    return DF_FAILED;
    }
    
    return DF_SUCCESS;
}

int handle_commands(int client_sock)
{
    int command;
    int ret = recv_command(client_sock, &command);
    LOG(INFO) << "command:" << command;

    if (ret == DF_FAILED)
    {
        LOG(INFO) << "connection command not received";
        close(client_sock);
        return DF_FAILED;
    }


    switch (command)
    {
    case DF_CMD_CONNECT:
        LOG(INFO) << "DF_CMD_CONNECT";
        ret = handle_cmd_connect(client_sock);
        break;
    case DF_CMD_DISCONNECT:
        LOG(INFO) << "DF_CMD_DISCONNECT";
        ret = handle_cmd_disconnect(client_sock);
        break;
    case DF_CMD_GET_RAW_01:
        LOG(INFO) << "DF_CMD_GET_RAW_01";
        ret = handle_cmd_get_raw_01(client_sock);
        break;
    case DF_CMD_GET_RAW_02:
        LOG(INFO) << "DF_CMD_GET_RAW_02";
        ret = handle_cmd_get_raw_02_16bit(client_sock);
        break;
    case DF_CMD_GET_RAW_03:
        LOG(INFO) << "DF_CMD_GET_RAW_03";
        ret = handle_cmd_get_raw_03(client_sock);
        break;
    case DF_CMD_GET_FRAME_01:
        LOG(INFO) << "DF_CMD_GET_FRAME_01";
        ret = handle_cmd_get_frame_01_parallel(client_sock);
        break;
    case DF_CMD_GET_FRAME_01_HDR:
        LOG(INFO) << "DF_CMD_GET_FRAME_01_HDR";
        ret = handle_cmd_get_frame_01_hdr_parallel(client_sock);
        break;
    case DF_CMD_GET_FRAME_TEST:
        LOG(INFO) << "DF_CMD_GET_FRAME_TEST";
        ret = handle_cmd_get_frame_test_parallel(client_sock);
        break;
    case DF_CMD_GET_FRAME_03:
        LOG(INFO) << "DF_CMD_GET_FRAME_03";
        ret = handle_cmd_get_frame_03_parallel(client_sock);
        break;
    case DF_CMD_GET_FRAME_04:
    LOG(INFO) << "DF_CMD_GET_FRAME_04";
    ret = handle_cmd_get_frame_04_parallel(client_sock);
    break;
    case DF_CMD_GET_FRAME_04_HDR:
    LOG(INFO) << "DF_CMD_GET_FRAME_04_HDR";
    ret = handle_cmd_get_frame_04_hdr_parallel(client_sock);
    break;
    case DF_CMD_HEARTBEAT:
        LOG(INFO) << "DF_CMD_HEARTBEAT";
        ret = handle_heartbeat(client_sock);
        break;
    case DF_CMD_GET_CAMERA_PARAMETERS:
        LOG(INFO) << "DF_CMD_GET_CAMERA_PARAMETERS";
        ret = handle_get_camera_parameters(client_sock);
        break;
    case DF_CMD_SET_CAMERA_PARAMETERS:
        LOG(INFO) << "DF_CMD_SET_CAMERA_PARAMETERS";
        ret = handle_set_camera_parameters(client_sock);
        read_calib_param();
        break;
    case DF_CMD_GET_SYSTEM_CONFIG_PARAMETERS:
        LOG(INFO) << "DF_CMD_GET_SYSTEM_CONFIG_PARAMETERS";
        ret = handle_get_system_config_parameters(client_sock);
        break;
    case DF_CMD_SET_PARAM_GENERATE_BRIGHTNESS:
        LOG(INFO) << "DF_CMD_SET_PARAM_GENERATE_BRIGHTNESS";
        ret = handle_cmd_set_param_generate_brightness(client_sock);
        break;
    case DF_CMD_GET_PARAM_GENERATE_BRIGHTNESS:
        LOG(INFO) << "DF_CMD_GET_PARAM_GENERATE_BRIGHTNESS";
        ret = handle_cmd_get_param_generate_brightness(client_sock);
        break;
    case DF_CMD_SET_PARAM_CAMERA_EXPOSURE_TIME:
        LOG(INFO) << "DF_CMD_SET_PARAM_CAMERA_EXPOSURE_TIME";
        ret = handle_cmd_set_param_camera_exposure(client_sock);
        break;
    case DF_CMD_GET_PARAM_CAMERA_EXPOSURE_TIME:
        LOG(INFO) << "DF_CMD_GET_PARAM_CAMERA_EXPOSURE_TIME";
        ret = handle_cmd_get_param_camera_exposure(client_sock);
        break;
    case DF_CMD_SET_PARAM_CAMERA_GAIN:
        LOG(INFO) << "DF_CMD_SET_PARAM_CAMERA_GAIN";
        ret = handle_cmd_set_param_camera_gain(client_sock);
        break;
    case DF_CMD_GET_PARAM_CAMERA_GAIN:
        LOG(INFO) << "DF_CMD_GET_PARAM_CAMERA_GAIN";
        ret = handle_cmd_get_param_camera_gain(client_sock);
        break;
    case DF_CMD_SET_PARAM_CAMERA_GAMMA:
        LOG(INFO) << "DF_CMD_SET_PARAM_CAMERA_GAMMA";
        ret = handle_cmd_set_param_camera_gamma(client_sock);
        break;
    case DF_CMD_GET_PARAM_CAMERA_GAMMA:
        LOG(INFO) << "DF_CMD_GET_PARAM_CAMERA_GAMMA";
        ret = handle_cmd_get_param_camera_gamma(client_sock);
        break;
    case DF_CMD_GET_CAMERA_RESOLUTION:
        LOG(INFO) << "DF_CMD_GET_CAMERA_RESOLUTION";
        ret = handle_cmd_get_param_camera_resolution(client_sock);
        break;
    case DF_CMD_GET_RGB_CAMERA_RESOLUTION:
        LOG(INFO) << "DF_CMD_GET_RGB_CAMERA_RESOLUTION";
        ret = handle_cmd_get_param_rgb_camera_resolution(client_sock);
        break;
    case DF_CMD_SET_PARAM_HDR:
        LOG(INFO) << "DF_CMD_SET_PARAM_HDR";
        ret = handle_cmd_set_param_hdr(client_sock);
        break;
    case DF_CMD_SET_PARAM_BILATERAL_FILTER:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_BILATERAL_FILTER";   
    	handle_cmd_set_param_bilateral_filter(client_sock);
	    break;
	case DF_CMD_GET_PARAM_BILATERAL_FILTER:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_BILATERAL_FILTER";   
    	ret = handle_cmd_get_param_bilateral_filter(client_sock);
	    break;
    case DF_CMD_SET_PARAM_DEPTH_FILTER:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_DEPTH_FILTER";   
    	ret = handle_cmd_set_param_depth_filter(client_sock);
	    break;
	case DF_CMD_GET_PARAM_DEPTH_FILTER:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_DEPTH_FILTER";   
    	ret = handle_cmd_get_param_depth_filter(client_sock);
	    break;
    case DF_CMD_GET_PARAM_CAMERA_CONFIDENCE:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_CAMERA_CONFIDENCE";   
    	ret = handle_cmd_get_param_confidence(client_sock); 
	    break;
    case DF_CMD_SET_PARAM_CAMERA_CONFIDENCE:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_CAMERA_CONFIDENCE";   
    	ret = handle_cmd_set_param_confidence(client_sock); 
	    break;
	case DF_CMD_GET_PARAM_LED_CURRENT:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_LED_CURRENT";   
    	ret = handle_cmd_get_param_led_current(client_sock);  
	    break;
	case DF_CMD_SET_PARAM_LED_CURRENT:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_LED_CURRENT";   
    	ret = handle_cmd_set_param_led_current(client_sock);  
	    break;
    case DF_CMD_GET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM";   
    	ret = handle_cmd_get_param_standard_param_external(client_sock);  
	    break;
	case DF_CMD_SET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM";   
    	ret = handle_cmd_set_param_standard_param_external(client_sock);  
	    break;
	case DF_CMD_GET_PARAM_PROJECTOR_VERSION:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_PROJECTOR_VERSION";   
    	ret = handle_cmd_get_param_projector_version(client_sock);
	    break;
	case DF_CMD_SET_PARAM_MIXED_HDR:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_MIXED_HDR";   
    	ret = handle_cmd_set_param_mixed_hdr(client_sock);
	    break;
	case DF_CMD_GET_PARAM_MIXED_HDR:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_MIXED_HDR";   
    	ret = handle_cmd_get_param_mixed_hdr(client_sock);
	    break;
	case DF_CMD_SET_PARAM_RADIUS_FILTER:
	    LOG(INFO)<<"DF_CMD_SET_PARAM_RADIUS_FILTER";
    	ret = handle_cmd_set_param_radius_filter(client_sock);
	    break;
	case DF_CMD_GET_PARAM_RADIUS_FILTER:
	    LOG(INFO)<<"DF_CMD_GET_PARAM_RADIUS_FILTER";   
    	ret = handle_cmd_get_param_radius_filter(client_sock);
	    break;
	case DF_CMD_GET_TEMPERATURE:
	    LOG(INFO)<<"DF_CMD_GET_TEMPERATURE";
	    ret = handle_get_temperature(client_sock);
	    break;
	case DF_CMD_GET_FIRMWARE_VERSION:
	    LOG(INFO)<<"DF_CMD_GET_FIRMWARE_VERSION";
	    ret = handle_get_firmware_version(client_sock);
	    break;
    case DF_CMD_GET_PRODUCT_INFO:
        LOG(INFO)<<"DF_CMD_GET_PRODUCT_INFO"; 
        ret = handle_cmd_get_product_info(client_sock);
        break;
	case DF_CMD_GET_NETWORK_BANDWIDTH:
	    LOG(INFO)<<"DF_CMD_GET_NETWORK_BANDWIDTH";
	    ret = handle_get_network_bandwidth(client_sock);
	    break;
	case DF_CMD_SET_SYSTEM_CONFIG_PARAMETERS:
	    LOG(INFO)<<"DF_CMD_SET_SYSTEM_CONFIG_PARAMETERS";
	    ret = handle_set_system_config_parameters(client_sock);
        saveSystemConfig();
	    break;
    default:
        LOG(INFO) << "DF_CMD_UNKNOWN";
        LOG(INFO) << "COMMAND: " << command;
        ret = handle_cmd_unknown(client_sock);
        break;
    }

    close(client_sock);

    LOG(INFO) << "handle_commands ret: " << ret;
    return DF_SUCCESS;
}

int init()
{
    if (!scan3d_.init())
    {
        LOG(INFO) << "init Failed!";
        // return DF_FAILED;
    }

    max_camera_exposure_ = scan3d_.getParamMaxCameraExposure();
    
    scan3d_.getCameraResolution(camera_width_, camera_height_);
    scan3d_.getRGBCameraResolution(rgb_camera_width_, rgb_camera_height_);

    if(!scan3d_.setParamConfidence(system_config_settings_machine_.Instance().firwmare_param_.confidence))
    { 
        LOG(INFO)<<"Set Param Confidence Failed!";
    }

    if (!scan3d_.setParamExposure(system_config_settings_machine_.Instance().config_param_.camera_exposure_time))
    {
       LOG(INFO) << "Set Param Exposure Failed!";
    }

    if (!scan3d_.setParamGain(system_config_settings_machine_.Instance().config_param_.camera_gain))
    {
       LOG(INFO) << "Set Param Gain Failed!";
    }

    scan3d_.setParamSystemConfig(system_config_settings_machine_);

    return DF_SUCCESS;
}

void rolloutHandler(const char* filename, std::size_t size)
{
    if (access("lumosLog", F_OK) != 0)
    {
        system("mkdir lumosLog");
    }

    std::vector<std::string> name_list;
    std::string suffix = "log";
    DIR* dir;
    struct dirent* ent;
    if ((dir = opendir("lumosLog")) != NULL)
    {
        while ((ent = readdir(dir)) != NULL)
        {
            /* print all the files and directories within directory */
            // printf("%s\n", ent->d_name);

            std::string name = ent->d_name;

            if (name.size() < 3)
            {
                continue;
            }

            std::string curSuffix = name.substr(name.size() - 3);

            if (suffix == curSuffix)
            {
                name_list.push_back(name);
            }
        }
        closedir(dir);
    }

    sort(name_list.begin(), name_list.end());

    int num = name_list.size();
    if (num < 10)
    {
        num++;
    }
    else
    {
        num = 10;
        name_list.pop_back();
    }


    for (int i = num; i > 0 && !name_list.empty(); i--)
    {
        std::stringstream ss;
        std::string path = "./lumosLog/" + name_list.back();
        name_list.pop_back();
        ss << "mv " << path << " lumosLog/log_" << i - 1 << ".log";
        std::cout << ss.str() << std::endl;
        system(ss.str().c_str());
    }

    std::stringstream ss;
    ss << "mv " << filename << " lumosLog/log_0" << ".log";
    system(ss.str().c_str());


}



int main()
{
    el::Configurations conf;
    conf.setToDefault();
    conf.setGlobally(el::ConfigurationType::Filename, "lumos_log.log");
    conf.setGlobally(el::ConfigurationType::Enabled, "true");
    conf.setGlobally(el::ConfigurationType::ToFile, "true");
    el::Loggers::reconfigureAllLoggers(conf);
    //log configure
    el::Loggers::addFlag(el::LoggingFlag::StrictLogFileSizeCheck);
    el::Loggers::reconfigureAllLoggers(el::ConfigurationType::MaxLogFileSize, "10485760");//10MB 10485760 
    el::Helpers::installPreRollOutCallback(rolloutHandler);

    /***************************************************************************/

    LOG(INFO) << "server started";
    int ret = init();

    LOG(INFO) << "inited";

    int server_sock;
    do
    {
        server_sock = setup_socket(DF_PORT);
        sleep(1);
    } while (server_sock == DF_FAILED);

    LOG(INFO) << "listening";
    scan3d_.captureFrame01();
    scan3d_.captureFrame01();
    while (true)
    {
        int client_sock = accept_new_connection(server_sock);
        if (client_sock != -1)
        {
            handle_commands(client_sock);
        }
    }

    close(server_sock);

    return 0;
}
