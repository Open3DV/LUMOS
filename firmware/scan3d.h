#ifndef SCAN_3D_H
#define SCAN_3D_H

#pragma once
#include "easylogging++.h"
#include "camera_mvs.h"
#include "camera_mipi.h"
#include "camera_mipi_virtual.h"
#include "camera_param.h"
#include "vector"
#include "system_config_settings.h"
#include "projector_base.h"
#include "projector_ainstec.h"
#include "projector_fpga.h"
#include <opencv2/opencv.hpp>

//#define VIRTUAL_CAMERA


class Scan3D
{
public:
	Scan3D();
	~Scan3D();

	int init();

    bool grayCodeToBinCode(bool* gray_code, bool* bin_code);

    bool cameraIsValid();

    bool reboot_flag_enable(std::string file_path, int reboot_falg);

    bool setParamExposure(float exposure);

    bool getParamExposure(float& exposure);

    bool setParamGain(float gain);

    bool getParamGain(float& gain);

    bool setParamGenerateBrightness(int model,int exposure);
    
    void setParamSystemConfig(SystemConfigDataStruct param);
    
    bool captureTextureImage(int model,float exposure,unsigned char* buff);

    bool captureRaw01(unsigned char* buff);

    bool captureRaw01_16bit(unsigned short* buff);

    bool captureRaw03(unsigned char* buff);
    
    int captureFrame01();

    int captureFrame04();

    int captureFrame05();

    int captureFrame04MixedHDR();

    int captureFrame04Repetition(int repetition_count);

    int captureFrame08();

    int captureFrame08MixedHDR();

    int captureFrame08Repetition(int repetition_count);

    int captureFrame03();

    int captureFrame01HDR();

    int captureFrame01MixedHDR();

    int captureFrameTest(unsigned char* patterns_buf);

    int captureColorBrightness();

    bool readCalibParam();

    bool writeCalibParamUser();

    bool loadCalibData();

    void copyBrightnessData(unsigned char* &ptr);
    
    void copyDepthData(float* &ptr);
    
    void copyPointcloudData(float* &ptr);

    void copyDepth2ColorData(unsigned short* &ptr);

    void copyColorBrightnessData(unsigned char* &ptr);

    void copyResizeColorBrightnessData(unsigned char* &ptr);

    void getCameraResolution(int &width, int &height);

    void getRGBCameraResolution(int &width, int &height);
    
    int captureFrame02();

    int captureFrame02_base();

    void removeOutlierBaseDepthFilter();

    void removeOutlierBaseRadiusFilter();

    bool read_hdr_param_from_file(const char* param_path, int& hdr_count, int* exposure_time, int* brightness);

    bool read_calib_param_from_file(const char* param_path, double* intrinsic_l, double* intrinsic_r, double* dist_l, double* dist_r, double* R_ptr, double* T_ptr);

    bool read_calib_param_from_file(const char* param_path, double* intrinsic_l, double* intrinsic_r, double* intrinsic_rgb, double* dist_l, double* dist_r, double* dist_rgb, double* R_ptr, double* R_rgb_ptr, double* T_ptr, double* T_rgb_ptr);

    bool setParamConfidence(float confidence);

    bool setLaserBrightness(int brightness);

    bool setParamLaserCurrent(int val);

    bool setParamHdr(int num,std::vector<int> led_list,std::vector<int> exposure_list);

    int getParamMaxCameraExposure();

    bool setParamCameraGamma(float gamma);

    bool getParamCameraGamma(float& gamma);

    int generate_brightness_model_;
    int generate_brightness_exposure_;

    int laser_current_;
    int hdr_num_ = 1;
    std::vector<int> laser_current_list_;
    std::vector<int> camera_exposure_list_;

private:
 
    Camera* camera_left_;
    Camera* camera_right_;
    Camera* camera_rgb_;

    bool camera_opened_;
    bool camera_left_opened_;
    bool camera_right_opened_;

    BaseProjector* projector_;

    bool camera_opened_flag_;
    
    struct CameraCalibParam calib_param_;

    struct CameraCalibParam calib_param_user_;

    int max_camera_exposure_;
    int min_camera_exposure_;

    int camera_exposure_;
    float camera_gain_;
    float camera_gamma_;

    int image_width_;
    int image_height_;

    int rgb_image_width_;
    int rgb_image_height_;

    unsigned char* buff_brightness_;
    unsigned char* buff_color_brightness_;
    float* buff_depth_;
    float* buff_pointcloud_;
    unsigned short* buff_depth_color_map_;

    SystemConfigDataStruct system_config_settings_machine_;

    cv::Mat Q_mat_;

    float Q_matrix_[16];



    unsigned short* host_img_left_[18];
    unsigned short* host_img_right_[18];

};


#endif