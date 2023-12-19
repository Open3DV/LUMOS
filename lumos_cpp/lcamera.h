#pragma once
#ifndef __LUMOS_LCAMERA_H__
#define __LUMOS_LCAMERA_H__
#include<vector>
#include<string>

extern "C" {
	namespace LUMOS {

		using namespace std;
#ifdef _WIN32 
#define LUMOS_API __declspec(dllexport)

#elif __linux
#define LUMOS_API 
#endif


		//相机标定参数结构体
		typedef struct CalibrationParam
		{
			//相机内参
			float intrinsic[3 * 3];
			//相机外参
			float extrinsic[4 * 4];
			//相机畸变，只用前5个
			float distortion[1 * 12];//<k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4>

		}CalibrationParam;

		enum class LumosCameraSelect
		{
			GrayCamera = 0,
			RGBCamera = 1,
		};

		class LCamera
		{
		public:
			LCamera() = default;
			virtual ~LCamera() = default;
			LCamera(const LCamera&) = delete;
			LCamera& operator=(const LCamera&) = delete;

			//功能： 连接相机
			//输入参数： camera_id（相机ip地址）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
			virtual int connect(const char* camera_id) = 0;

			//功能： 设置拍摄所用的相机，采集时，会根据选择的相机返回对于应拍照数据（分辨率、内参、通道数、亮度图、深度图）
			//输入参数：camera_select(GrayCamera：左目灰色相机、RGBCamera：中间彩色相机)
			//输出参数：  
			//返回值： 类型（int）:返回0表示设置参数成功;返回-1表示设置参数失败。
			virtual int selectCamera(LumosCameraSelect camera_select) = 0;

			//功能： 获取选择的拍摄相机
			//输入参数：
			//输出参数：camera_select(GrayCamera：左目灰色相机、RGBCamera：中间彩色相机)
			//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败。
			virtual int getSelectedCamera(LumosCameraSelect& camera_select) = 0;

			//功能： 获取相机图像通道数
			//输入参数： 无
			//输出参数： channels(通道数)
			//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
			virtual int getCameraChannels(int* channels) = 0;

			//功能： 获取相机分辨率
			//输入参数： 无
			//输出参数： width(图像宽)、height(图像高)
			//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
			virtual int getCameraResolution(int* width, int* height) = 0;

			//功能： 转换左目黑白相机手眼标定参数到右目
			//输入参数： gray_cam_2_base_r（左目相机坐标系到base坐标系的旋转矩阵）、gray_cam_2_base_t（左目相机坐标系到base坐标系的平移）
			//输出参数： output_rgb_cam_2_base_r（彩色相机坐标系到base坐标系的旋转矩阵）、output_rgb_cam_2_base_t（彩色相机坐标系到base坐标系的平移）
			//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
			virtual int transGrayHandEeyCalibToRGB(float* gray_cam_2_base_r, float* gray_cam_2_base_t, float* output_rgb_cam_2_base_r, float* output_rgb_cam_2_base_t) = 0;

			//功能： 获取相机分辨率
			//输入参数： 无
			//输出参数： width(图像宽)、height(图像高)
			//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
			virtual int getGrayCameraResolution(int* width, int* height) = 0;

			//功能： 获取彩色相机分辨率
			//输入参数： 无
			//输出参数： width(图像宽)、height(图像高)
			//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
			virtual int getRGBCameraResolution(int* width, int* height) = 0;

			//功能： 断开相机连接
			//输入参数： camera_id（相机ip地址）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示断开成功;返回-1表示断开失败.
			virtual int disconnect(const char* camera_id) = 0;

			//功能： 获取相机标定参数
			//输入参数： 无
			//输出参数： calibration_param（相机标定参数结构体）
			//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
			virtual int getCalibrationParam(struct CalibrationParam* calibration_param) = 0;

			//功能： 设置相机曝光时间
			//输入参数：exposure(相机曝光时间)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual int setParamCameraExposure(float exposure) = 0;

			//功能： 获取相机曝光时间
			//输入参数： 无
			//输出参数：exposure(相机曝光时间)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			virtual int getParamCameraExposure(float& exposure) = 0;

			//功能： 设置相机增益
			//输入参数：gain(相机增益)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual int setParamCameraGain(float gain) = 0;

			//功能： 获取相机增益
			//输入参数： 无
			//输出参数：gain(相机增益)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			virtual int getParamCameraGain(float& gain) = 0;

			//功能： 设置相机Gamma矫正
			//输入参数：gamma(Gamma参数)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual int setParamCameraGamma(float gamma) = 0;

			//功能： 获取相机Gamma矫正
			//输入参数：无
			//输出参数：gamma(Gamma参数)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			virtual int getParamCameraGamma(float& gamma) = 0;

			//功能： 设置HDR曝光参数
			//输入参数： 曝光组数，曝光时间，亮度
			//输出参数：无
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			virtual int setParamHDR(int hdr_count, int* exposure_list, int* brightness_list) = 0;

			//功能： 单独拍照获取一个彩色亮度图数据
			//输入参数：brightness_buf_size（亮度图尺寸sizeof(unsigned char) * width * height）
			//输出参数：brightness
			//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
			virtual int getBrightness(unsigned char* resize_color_brightness, int resize_color_brightness_buf_size) = 0;

			//功能： 获取点云平滑参数
			//输入参数：无
			//输出参数：smoothing(0:关、1-5:平滑程度由低到高)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			virtual int getParamSmoothing(int& smoothing) = 0;

			//功能： 设置点云平滑参数
			//输入参数：smoothing(0:关、1-5:平滑程度由低到高)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual int setParamSmoothing(int smoothing) = 0;

			//功能： 设置深度图滤波参数
			//输入参数：use(开关：1开、0关)、depth_filterthreshold(阈值0-100)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual int setParamDepthFilter(int use, float depth_filter_threshold) = 0;

			//功能： 设置深度图滤波参数
			//输入参数：use(开关：1开、0关)、depth_filterthreshold(阈值0-100)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			virtual int getParamDepthFilter(int& use, float& depth_filter_threshold) = 0;

			//功能： 设置相机噪声过滤置信度
			//输入参数：confidence(相机置信度)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual int setParamCameraConfidence(float confidence) = 0;

			//功能： 获取相机噪声过滤置信度
			//输入参数： 无
			//输出参数：confidence(相机置信度)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			virtual int getParamCameraConfidence(float& confidence) = 0;

			//功能： 设置LED电流
			//输入参数： led（电流值）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual int setParamLedCurrent(int led) = 0;


			//功能： 设置LED电流
			//输入参数： 无
			//输出参数： led（电流值）
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			virtual int getParamLedCurrent(int& led) = 0;

			//功能： 设置混合多曝光参数（最大曝光次数为5次）
			//输入参数： num（曝光次数）、exposure_param（曝光参数）、led_param[5]（5个led亮度参数、前num个有效）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
			virtual int setParamMixedHdr(int num, int exposure_param[5], int led_param[5]) = 0;

			//功能： 获取混合多曝光参数（最大曝光次数为5次）
			//输入参数： 无
			//输出参数： num（曝光次数）、exposure_param（曝光参数）、led_param[5]（5个led亮度参数、前num个有效）
			//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
			virtual int getParamMixedHdr(int& num, int exposure_param[5], int led_param[5]) = 0;

			//功能： 设置基准平面的外参
			//输入参数：R(旋转矩阵：3*3)、T(平移矩阵：3*1)
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual int setParamStandardPlaneExternal(float* R, float* T) = 0;

			//功能： 获取基准平面的外参
			//输入参数：无
			//输出参数： R(旋转矩阵：3*3)、T(平移矩阵：3*1)
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			virtual int getParamStandardPlaneExternal(float* R, float* T) = 0;

			//功能： 设置点云半径滤波参数
			//输入参数：use(开关：1开、0关)、radius(半径）、num（有效点）
			//输出参数： 无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual int setParamRadiusFilter(int use, float radius, int num) = 0;

			//功能： 获取点云半径滤波参数
			//输入参数：无
			//输出参数：use(开关：1开、0关)、radius(半径）、num（有效点）
			//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
			virtual int getParamRadiusFilter(int& use, float& radius, int& num) = 0;

			//功能： 设置多曝光模式
			//输入参数： model(1：HDR(默认值)、2：重复曝光)
			//输出参数：无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual int setParamMultipleExposureModel(int model) = 0;

			//功能： 设置重复曝光数
			//输入参数： num(2-10)
			//输出参数：无
			//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
			virtual int setParamRepetitionExposureNum(int num) = 0;

			//功能： 采集一帧数据并阻塞至返回状态
			//输入参数： exposure_num（曝光次数）：设置值为1为单曝光，大于1为多曝光模式（具体参数在相机gui中设置）.
			//输出参数： timestamp(时间戳)
			//返回值： 类型（int）:返回0表示获取采集数据成功;返回-1表示采集数据失败.
			virtual int captureData(int exposure_num, char* timestamp) = 0;

			//功能： 获取亮度图
			//输入参数：无
			//输出参数： brightness(亮度图)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			virtual int getBrightnessData(unsigned char* brightness) = 0;

			//功能： 获取深度图
			//输入参数：无
			//输出参数： depth(深度图)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			virtual int getDepthDataFloat(float* depth) = 0;

			//功能： 获取校正到基准平面的高度映射图
			//输入参数：无  
			//输出参数： height_map(高度映射图)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			virtual int getHeightMapData(float* height_map) = 0;

			//功能： 获取基准平面参数
			//输入参数：无
			//输出参数： R(旋转矩阵：3*3)、T(平移矩阵：3*1)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			virtual int getStandardPlaneParam(float* R, float* T) = 0;

			//功能： 获取校正到基准平面的高度映射图
			//输入参数：R(旋转矩阵)、T(平移矩阵)
			//输出参数： height_map(高度映射图)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			virtual int getHeightMapDataBaseParam(float* R, float* T, float* height_map) = 0;

			//功能： 获取点云
			//输入参数：无
			//输出参数： point_cloud(点云)
			//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
			virtual int getPointcloudData(float* point_cloud) = 0;
		};

		LUMOS_API void* createLCamera();
		LUMOS_API void destroyLCamera(void*);

	}
}
#endif