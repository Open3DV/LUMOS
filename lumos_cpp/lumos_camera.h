#pragma once 
#ifndef __XEMA_CAMERA_H__
#define __XEMA_CAMERA_H__

#include "lcamera.h"
#include <mutex>
#include <thread>
#include "../SDK/socket_tcp.h" 
#include "../firmware/camera_param.h" 
#include "../firmware/system_config_settings.h"

extern "C" {
	namespace LUMOS {

		class LumosCamera : public LCamera
		{
		public:
			LumosCamera();
			~LumosCamera();
			LumosCamera(const LumosCamera&) = delete;
			LumosCamera& operator=(const LumosCamera&) = delete;

			//功能： 连接相机
			//输入参数： camera_id（相机ip地址）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
			int connect(const char* camera_id)override;

			//功能： 设置拍摄所用的相机，采集时，会根据选择的相机返回对于应拍照数据（分辨率、内参、通道数、亮度图、深度图）
			//输入参数：camera_select(GrayCamera：左目灰色相机、RGBCamera：中间彩色相机)
			//输出参数：  
			//返回值： 类型（int）:返回0表示设置参数成功;返回-1表示设置参数失败。
			int selectCamera(LumosCameraSelect camera_select)override;

			//功能： 获取选择的拍摄相机
			//输入参数：
			//输出参数：camera_select(GrayCamera：左目灰色相机、RGBCamera：中间彩色相机)
			//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败。
			int getSelectedCamera(LumosCameraSelect& camera_select)override;

			//功能： 获取相机图像通道数
			//输入参数： 无
			//输出参数： channels(通道数)
			//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
			int getCameraChannels(int* channels)override;

			//功能： 获取相机分辨率
			//输入参数： 无
			//输出参数： width(图像宽)、height(图像高)
			//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
			int getCameraResolution(int* width, int* height)override;

			//功能： 转换左目黑白相机手眼标定参数到右目
			//输入参数： gray_cam_2_base_r（左目相机坐标系到base坐标系的旋转矩阵）、gray_cam_2_base_t（左目相机坐标系到base坐标系的平移）
			//输出参数： output_rgb_cam_2_base_r（彩色相机坐标系到base坐标系的旋转矩阵）、output_rgb_cam_2_base_t（彩色相机坐标系到base坐标系的平移）
			//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
			int transGrayHandEeyCalibToRGB(float* gray_cam_2_base_r, float* gray_cam_2_base_t, float* output_rgb_cam_2_base_r, float* output_rgb_cam_2_base_t)override;

			//功能： 获取相机分辨率
			//输入参数： 无
			//输出参数： width(图像宽)、height(图像高)
			//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
			int getGrayCameraResolution(int* width, int* height)override;

			//功能： 获取彩色相机分辨率
			//输入参数： 无
			//输出参数： width(图像宽)、height(图像高)
			//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
			int getRGBCameraResolution(int* width, int* height)override;

			//功能： 断开相机连接
			//输入参数： camera_id（相机ip地址）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示断开成功;返回-1表示断开失败.
			int disconnect(const char* camera_id)override;

			//功能： 获取相机标定参数
			//输入参数： 无
			//输出参数： calibration_param（相机标定参数结构体）
			//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
			int getCalibrationParam(struct CalibrationParam* calibration_param)override;

			//功能： 设置相机曝光时间
			//输入参数：exposure(相机曝光时间)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamCameraExposure(float exposure)override;

			//功能： 获取相机曝光时间
			//输入参数： 无
			//输出参数：exposure(相机曝光时间)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			int getParamCameraExposure(float& exposure)override;

			//功能： 设置相机增益
			//输入参数：gain(相机增益)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamCameraGain(float gain)override;

			//功能： 获取相机增益
			//输入参数： 无
			//输出参数：gain(相机增益)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			int getParamCameraGain(float& gain)override;

			//功能： 设置相机Gamma矫正
			//输入参数：gamma(Gamma参数)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamCameraGamma(float gamma)override;

			//功能： 获取相机Gamma矫正
			//输入参数：无
			//输出参数：gamma(Gamma参数)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			int getParamCameraGamma(float& gamma)override;

			//功能： 设置HDR曝光参数
			//输入参数： 曝光组数，曝光时间，亮度
			//输出参数：无
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			int setParamHDR(int hdr_count, int* exposure_list, int* brightness_list)override;

			//功能： 单独拍照获取一个彩色亮度图数据
			//输入参数：brightness_buf_size（亮度图尺寸sizeof(unsigned char) * width * height）
			//输出参数：brightness
			//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
			int getBrightness(unsigned char* resize_color_brightness, int resize_color_brightness_buf_size)override;

			//功能： 获取点云平滑参数
			//输入参数：无
			//输出参数：smoothing(0:关、1-5:平滑程度由低到高)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			int getParamSmoothing(int& smoothing)override;

			//功能： 设置点云平滑参数
			//输入参数：smoothing(0:关、1-5:平滑程度由低到高)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamSmoothing(int smoothing)override;

			//功能： 设置深度图滤波参数
			//输入参数：use(开关：1开、0关)、depth_filterthreshold(阈值0-100)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamDepthFilter(int use, float depth_filter_threshold)override;

			//功能： 设置深度图滤波参数
			//输入参数：use(开关：1开、0关)、depth_filterthreshold(阈值0-100)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			int getParamDepthFilter(int& use, float& depth_filter_threshold)override;

			//功能： 设置相机噪声过滤置信度
			//输入参数：confidence(相机置信度)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamCameraConfidence(float confidence)override;

			//功能： 获取相机噪声过滤置信度
			//输入参数： 无
			//输出参数：confidence(相机置信度)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			int getParamCameraConfidence(float& confidence)override;

			//功能： 设置LED电流
			//输入参数： led（电流值）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamLedCurrent(int led)override;

			//功能： 设置LED电流
			//输入参数： 无
			//输出参数： led（电流值）
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			int getParamLedCurrent(int& led)override;

			//功能： 设置混合多曝光参数（最大曝光次数为5次）
			//输入参数： num（曝光次数）、exposure_param（曝光参数）、led_param[5]（5个led亮度参数、前num个有效）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
			int setParamMixedHdr(int num, int exposure_param[5], int led_param[5])override;

			//功能： 获取混合多曝光参数（最大曝光次数为5次）
			//输入参数： 无
			//输出参数： num（曝光次数）、exposure_param（曝光参数）、led_param[5]（5个led亮度参数、前num个有效）
			//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
			int getParamMixedHdr(int& num, int exposure_param[5], int led_param[5])override;

			//功能： 设置基准平面的外参
			//输入参数：R(旋转矩阵：3*3)、T(平移矩阵：3*1)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamStandardPlaneExternal(float* R, float* T)override;

			//功能： 获取基准平面的外参
			//输入参数：无
			//输出参数： R(旋转矩阵：3*3)、T(平移矩阵：3*1)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			int getParamStandardPlaneExternal(float* R, float* T)override;

			//功能： 设置点云半径滤波参数
			//输入参数：use(开关：1开、0关)、radius(半径）、num（有效点）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamRadiusFilter(int use, float radius, int num)override;

			//功能： 获取点云半径滤波参数
			//输入参数：无
			//输出参数：use(开关：1开、0关)、radius(半径）、num（有效点）
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			int getParamRadiusFilter(int& use, float& radius, int& num)override;

			//功能： 设置多曝光模式
			//输入参数： model(1：HDR(默认值)、2：重复曝光)
			//输出参数：无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamMultipleExposureModel(int model)override;

			//功能： 设置重复曝光数
			//输入参数： num(2-10)
			//输出参数：无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			int setParamRepetitionExposureNum(int num)override;

			//功能： 采集一帧数据并阻塞至返回状态
			//输入参数： exposure_num（曝光次数）：设置值为1为单曝光，大于1为多曝光模式（具体参数在相机gui中设置）.
			//输出参数： timestamp(时间戳)
			//返回值： 类型（int）:返回0表示获取采集数据成功;返回-1表示采集数据失败.
			int captureData(int exposure_num, char* timestamp)override;

			//功能： 获取亮度图
			//输入参数：无
			//输出参数： brightness(亮度图)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			int getBrightnessData(unsigned char* brightness)override;

			//功能： 获取深度图
			//输入参数：无
			//输出参数： depth(深度图)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			int getDepthDataFloat(float* depth)override;

			//功能： 获取校正到基准平面的高度映射图
			//输入参数：无  
			//输出参数： height_map(高度映射图)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			int getHeightMapData(float* height_map)override;

			//功能： 获取基准平面参数
			//输入参数：无
			//输出参数： R(旋转矩阵：3*3)、T(平移矩阵：3*1)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			int getStandardPlaneParam(float* R, float* T)override;

			//功能： 获取校正到基准平面的高度映射图
			//输入参数：R(旋转矩阵)、T(平移矩阵)
			//输出参数： height_map(高度映射图)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			int getHeightMapDataBaseParam(float* R, float* T, float* height_map)override;

			//功能： 获取点云
			//输入参数：无
			//输出参数： point_cloud(点云)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			int getPointcloudData(float* point_cloud)override;

		public:

			int getFrame01(float* depth, int depth_buf_size,
				unsigned char* brightness, int brightness_buf_size);

			int getFrame01HDR(float* depth, int depth_buf_size,
				unsigned char* brightness, int brightness_buf_size);

			int getRepetitionFrame01(int count, float* depth, int depth_buf_size,
				unsigned char* brightness, int brightness_buf_size);

			int getFrame03(float* depth, int depth_buf_size,
				unsigned char* brightness, int brightness_buf_size, unsigned char* color_brightness, int color_brightness_buf_size, unsigned short* depth2color, int depth2color_buf_size);

			int getFrame04(float* depth, int depth_buf_size,
				unsigned char* brightness, int brightness_buf_size, unsigned char* color_brightness, int color_brightness_buf_size, unsigned char* resize_color_brightness, int resize_color_brightness_buf_size, float* resize_color_depth, int resize_color_depth_buf_size);

			int getFrame04HDR(float* depth, int depth_buf_size,
				unsigned char* brightness, int brightness_buf_size, unsigned char* color_brightness, int color_brightness_buf_size, unsigned char* resize_color_brightness, int resize_color_brightness_buf_size, float* resize_color_depth, int resize_color_depth_buf_size);

			int getRepetitionFrame04(int count, float* depth, int depth_buf_size,
				unsigned char* brightness, int brightness_buf_size, unsigned char* color_brightness, int color_brightness_buf_size, unsigned char* resize_color_brightness, int resize_color_brightness_buf_size, float* resize_color_depth, int resize_color_depth_buf_size);

			int getFrame05(float* depth, int depth_buf_size,
				unsigned char* brightness, int brightness_buf_size, unsigned char* color_brightness, int color_brightness_buf_size, unsigned char* resize_color_brightness, int resize_color_brightness_buf_size, float* resize_color_depth, int resize_color_depth_buf_size);

			int getFrameTest(float* depth, int depth_buf_size,
				unsigned char* brightness, int brightness_buf_size, unsigned char* patterns, int patterns_buf_size);

			int getCameraRawData01(unsigned char* raw, int raw_buf_size);

			int getCameraRawData02(unsigned short* raw, int raw_buf_size);

			int getCameraRawData03(unsigned char* raw, int raw_buf_size);

			int setParamGenerateBrightness(int model, float exposure);

			int getParamGenerateBrightness(int& model, float& exposure);

			int depthTransformPointcloud(float* depth_map, float* point_cloud_map);

			int getCalibrationParam(struct CameraCalibParam& calibration_param);

			int setCalibrationParam(const struct CameraCalibParam& calibration_param);

			int getSystemConfigParam(struct SystemConfigParam& config_param);

			int setSystemConfigParam(const struct SystemConfigParam& config_param);

			bool transformPointcloud(float* org_point_cloud_map, float* transform_point_cloud_map, float* rotate, float* translation);

			int getDeviceTemperature(float& temperature);

			int setParamBilateralFilter(int use, int param_d);

			int getParamBilateralFilter(int& use, int& param_d);

			int getProjectorVersion(int& version);

			int getFirmwareVersion(char* pVersion, int length);

			int firmwareVersionIsOlder(long long version_num, bool& firmware_is_older);

			int getProductInfo(char* info, int length);

			int getNetworkBandwidth(int& speed);

			int getFrameStatus(int& status);

			int undistortRGBBrightnessMap(unsigned char* brightness_map);

			int undistortResizeRGBBrightnessMap(unsigned char* brightness_map);

			int initCaptureData();

			std::string get_timestamp();

			std::time_t getTimeStamp(long long& msec);

			std::tm* gettm(long long timestamp);
		public:

			int registerOnDropped(int (*p_function)(void*));

			int connectNet(const char* ip);

			int disconnectNet();

			int HeartBeat();

			int HeartBeat_loop();

		private:
			bool connected = false;
			long long token = 0;
			std::string camera_id_;
			std::string firmware_version_;
			std::thread heartbeat_thread;
			int heartbeat_error_count_ = 0;

			SOCKET g_sock_heartbeat;
			SOCKET g_sock;

			int (*p_OnDropped)(void*) = 0;

			int multiple_exposure_model_ = 1;
			int repetition_exposure_model_ = 2;

			std::timed_mutex command_mutex_;

			struct CameraCalibParam calibration_param_;
			bool connected_flag_ = false;

			int camera_width_ = 1920;
			int camera_height_ = 1200;
			int rgb_camera_width_ = 1920;
			int rgb_camera_height_ = 1200;
			int resize_rgb_camera_width_ = rgb_camera_width_ / 2;
			int resize_rgb_camera_height_ = rgb_camera_height_ / 2;

			int image_size_ = camera_width_ * camera_height_;
			int rgb_image_size_ = rgb_camera_width_ * rgb_camera_height_ * 3;

			const char* camera_ip_ = "";


			int depth_buf_size_ = 0;
			int resize_color_depth_buf_size_ = 0;
			int pointcloud_buf_size_ = 0;
			int resize_color_pointcloud_buf_size_ = 0;
			int brightness_buf_size_ = 0;
			int rgb_brightness_buf_size_ = 0;
			int resize_color_brightness_buf_size_ = 0;
			int depth2texture_buf_size_ = 0;
			float* point_cloud_buf_ = NULL;
			float* resize_color_point_cloud_buf_ = NULL;
			float* trans_point_cloud_buf_ = NULL;
			float* resize_color_trans_point_cloud_buf_ = NULL;
			bool transform_pointcloud_flag_ = false;
			float* depth_buf_ = NULL;
			float* resize_color_depth_buf_ = NULL;
			unsigned char* brightness_buf_ = NULL;
			unsigned char* rgb_brightness_buf_ = NULL;
			unsigned char* resize_color_brightness_buf_ = NULL;
			unsigned short* depth2texture_buf_ = NULL;
			float* undistort_map_x_ = NULL;
			float* undistort_map_y_ = NULL;
			float* distorted_map_x_ = NULL;
			float* distorted_map_y_ = NULL;
			unsigned char* brightness_map_temp = NULL;

			LumosCameraSelect lumos_camera_select_ = LumosCameraSelect::GrayCamera;
		};


	}
}
#endif
