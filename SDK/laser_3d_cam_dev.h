#pragma once

#ifdef _WIN32 
#define DF_SDK_API __declspec(dllexport)

#elif __linux
#define DF_SDK_API 
#endif

/***************************************************************************************/

extern "C"
{

	//返回码
	//0: 成功; -1:失败; -2:未获取相机分辨率分配内存

	//相机标定参数结构体
	struct CalibrationParam
	{
		//相机内参
		float intrinsic[3 * 3];
		//相机外参
		float extrinsic[4 * 4];
		//相机畸变，只用前5个
		float distortion[1 * 12];//<k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4>

	};

	enum class LumosCameraSelect
	{
		GrayCamera = 0,
		RGBCamera = 1,
	};

	//函数名： DfConnect
	//功能： 连接相机
	//输入参数： camera_id（相机ip地址）
	//输出参数： 无
	//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
	DF_SDK_API int DfConnect(const char* camera_id);

	//函数名： DfSelectCamera
	//功能： 设置拍摄所用的相机，采集时，会根据选择的相机返回对于应拍照数据（分辨率、内参、通道数、亮度图、深度图）
	//输入参数：camera_select(GrayCamera：左目灰色相机、RGBCamera：中间彩色相机)
	//输出参数：  
	//返回值： 类型（int）:返回0表示设置参数成功;返回-1表示设置参数失败。
	DF_SDK_API int DfSelectCamera(LumosCameraSelect camera_select);

	//函数名： DfGetSelectedCamera
	//功能： 获取选择的拍摄相机
	//输入参数：
	//输出参数：camera_select(GrayCamera：左目灰色相机、RGBCamera：中间彩色相机)
	//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败。
	DF_SDK_API int DfGetSelectedCamera(LumosCameraSelect& camera_select);

	//函数名： DfGetCameraChannels
	//功能： 获取相机图像通道数
	//输入参数： 无
	//输出参数： channels(通道数)
	//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
	DF_SDK_API int  DfGetCameraChannels(int* channels);

	//函数名： DfGetCameraResolution
	//功能： 获取相机分辨率
	//输入参数： 无
	//输出参数： width(图像宽)、height(图像高)
	//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
	DF_SDK_API int  DfGetCameraResolution(int* width, int* height);

	//函数名： DfTransGrayHandEeyCalibToRGB
	//功能： 转换左目黑白相机手眼标定参数到右目
	//输入参数： gray_cam_2_base_r（左目相机坐标系到base坐标系的旋转矩阵）、gray_cam_2_base_t（左目相机坐标系到base坐标系的平移）
	//输出参数： output_rgb_cam_2_base_r（彩色相机坐标系到base坐标系的旋转矩阵）、output_rgb_cam_2_base_t（彩色相机坐标系到base坐标系的平移）
	//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
	DF_SDK_API int  DfTransGrayHandEeyCalibToRGB(float* gray_cam_2_base_r, float* gray_cam_2_base_t, float* output_rgb_cam_2_base_r, float* output_rgb_cam_2_base_t);

	//函数名： DfGetGrayCameraResolution
	//功能： 获取相机分辨率
	//输入参数： 无
	//输出参数： width(图像宽)、height(图像高)
	//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
	DF_SDK_API int  DfGetGrayCameraResolution(int* width, int* height);

	//函数名： DfGetRGBCameraResolution
	//功能： 获取彩色相机分辨率
	//输入参数： 无
	//输出参数： width(图像宽)、height(图像高)
	//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
	DF_SDK_API int  DfGetRGBCameraResolution(int* width, int* height);
	
	//函数名： DfConnect
	//功能： 断开相机连接
	//输入参数： camera_id（相机ip地址）
	//输出参数： 无
	//返回值： 类型（int）:返回0表示断开成功;返回-1表示断开失败.
	DF_SDK_API int DfDisconnect(const char* camera_id);

	//函数名： DfGetCalibrationParam
	//功能： 获取相机标定参数
	//输入参数： 无
	//输出参数： calibration_param（相机标定参数结构体）
	//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
	DF_SDK_API int DfGetCalibrationParam(struct CalibrationParam* calibration_param);

	//函数名： DfSetParamCameraExposure
	//功能： 设置相机曝光时间
	//输入参数：exposure(相机曝光时间)
	//输出参数： 无
	//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
	DF_SDK_API int DfSetParamCameraExposure(float exposure);

	//函数名： DfGetParamCameraExposure
	//功能： 获取相机曝光时间
	//输入参数： 无
	//输出参数：exposure(相机曝光时间)
	//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
	DF_SDK_API int DfGetParamCameraExposure(float& exposure);

	//函数名： DfSetParamCameraGain
	//功能： 设置相机增益
	//输入参数：gain(相机增益)
	//输出参数： 无
	//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
	DF_SDK_API int DfSetParamCameraGain(float gain);

	//函数名： DfGetParamCameraGain
	//功能： 获取相机增益
	//输入参数： 无
	//输出参数：gain(相机增益)
	//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
	DF_SDK_API int DfGetParamCameraGain(float& gain);

	//函数名： DfSetParamCameraGamma
	//功能： 设置相机Gamma矫正
	//输入参数：gamma(Gamma参数)
	//输出参数： 无
	//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
	DF_SDK_API int DfSetParamCameraGamma(float gamma);

	//函数名： DfSetParamCameraGamma
	//功能： 获取相机Gamma矫正
	//输入参数：无
	//输出参数：gamma(Gamma参数)
	//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
	DF_SDK_API int DfGetParamCameraGamma(float& gamma);

	//函数名： DfSetParamHDR
	//功能： 设置HDR曝光参数
	//输入参数： 曝光组数，曝光时间，亮度
	//输出参数：无
	//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
	DF_SDK_API int DfSetParamHDR(int hdr_count, int* exposure_list, int* brightness_list);

	//函数名： DfGetFrame01
	//功能： 获取一帧数据（亮度图+深度图），基于Raw01的相移图
	//输入参数：depth_buf_size（深度图尺寸）、brightness_buf_size（亮度图尺寸）
	//输出参数：depth（深度图）、brightness（亮度图）
	//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
	DF_SDK_API int DfGetFrame01(float* depth, int depth_buf_size,
		unsigned char* brightness, int brightness_buf_size);

	//函数名： DfGetFrame01HDR
	//功能： 获取一帧高动态数据（亮度图+深度图），基于Raw01的相移图
	//输入参数：depth_buf_size（深度图尺寸）、brightness_buf_size（亮度图尺寸）
	//输出参数：depth（深度图）、brightness（亮度图）
	//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
	DF_SDK_API int DfGetFrame01HDR(float* depth, int depth_buf_size,
		unsigned char* brightness, int brightness_buf_size);

	//函数名： DfGetRepetitionFrame01
	//功能： 获取一帧数据（亮度图+深度图），基于Raw01重复count次
	//输入参数：count（重复次数）、depth_buf_size（深度图尺寸）、brightness_buf_size（亮度图尺寸）
	//输出参数：depth（深度图）、brightness（亮度图）
	//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
	DF_SDK_API int DfGetRepetitionFrame01(int count, float* depth, int depth_buf_size,
		unsigned char* brightness, int brightness_buf_size);

	//函数名： DfGetFrame03
	//功能： 获取一帧彩色深度数据（黑白亮度图+彩色亮度图+深度图+深度图转彩色表），基于Raw01的相移图
	//输入参数：depth_buf_size（深度图尺寸）、brightness_buf_size（亮度图尺寸）
	//输出参数：depth（深度图）、brightness（亮度图）、color_brightness（彩色亮度图）、depth2color（深度图转彩色表）
	//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
	DF_SDK_API int DfGetFrame03(float* depth, int depth_buf_size,
		unsigned char* brightness, int brightness_buf_size, unsigned char* color_brightness, int color_brightness_buf_size, unsigned short* depth2color, int depth2color_buf_size);

	//函数名： DfGetFrame04
	//功能： 获取一帧彩色深度数据（黑白亮度图+彩色亮度图+深度图+深度图转彩色表），基于Raw01的相移图
	//输入参数：depth_buf_size（深度图尺寸）、brightness_buf_size（亮度图尺寸）
	//输出参数：depth（深度图）、brightness（亮度图）、color_brightness（彩色亮度图）、depth2color（深度图转彩色表）
	//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
	DF_SDK_API int DfGetFrame04(float* depth, int depth_buf_size,
		unsigned char* brightness, int brightness_buf_size, unsigned char* color_brightness, int color_brightness_buf_size, unsigned char* resize_color_brightness, int resize_color_brightness_buf_size, float* resize_color_depth, int resize_color_depth_buf_size);

	//函数名： DfGetFrame04HDR
	//功能： 获取一帧HDR彩色深度数据（黑白亮度图+彩色亮度图+深度图+深度图转彩色表），基于Raw01的相移图
	//输入参数：depth_buf_size（深度图尺寸）、brightness_buf_size（亮度图尺寸）
	//输出参数：depth（深度图）、brightness（亮度图）、color_brightness（彩色亮度图）、depth2color（深度图转彩色表）
	//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
	DF_SDK_API int DfGetFrame04HDR(float* depth, int depth_buf_size,
		unsigned char* brightness, int brightness_buf_size, unsigned char* color_brightness, int color_brightness_buf_size, unsigned char* resize_color_brightness, int resize_color_brightness_buf_size, float* resize_color_depth, int resize_color_depth_buf_size);

	//函数名： DfGetRepetitionFrame01
	//功能： 获取一帧数据（彩色亮度图+深度图），基于Raw03重复count次
	//输入参数：count（重复次数）、depth_buf_size（深度图尺寸）、brightness_buf_size（亮度图尺寸）
	//输出参数：depth（深度图）、brightness（亮度图）
	//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
	DF_SDK_API int DfGetRepetitionFrame04(int count, float* depth, int depth_buf_size,
		unsigned char* brightness, int brightness_buf_size, unsigned char* color_brightness, int color_brightness_buf_size, unsigned char* resize_color_brightness, int resize_color_brightness_buf_size, float* resize_color_depth, int resize_color_depth_buf_size);

	//函数名： DfGetFrameTest
	//功能： 获取一帧数据（亮度图+深度图），以及28张基于Raw01的相移图
	//输入参数：depth_buf_size（深度图尺寸）、brightness_buf_size（亮度图尺寸）、patterns_buf_size（Raw01图片组尺寸）
	//输出参数：depth（深度图）、brightness（亮度图）
	//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
	DF_SDK_API int DfGetFrameTest(float* depth, int depth_buf_size,
		unsigned char* brightness, int brightness_buf_size, unsigned char* patterns, int patterns_buf_size);

	//函数名： DfGetCameraRawData01
	//功能： 采集一组8bit条纹图，一共28张，4张相移条纹图 + 8张格雷码图 + 2张黑白图
	//输入参数：raw_buf_size（28张8位图的尺寸）
	//输出参数：raw
	//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
	DF_SDK_API int DfGetCameraRawData01(unsigned char* raw, int raw_buf_size);

	//函数名： DfGetCameraRawData02
	//功能： 采集一组16bit条纹图，一共28张，4张相移条纹图 + 8张格雷码图 + 2张黑白图
	//输入参数：raw_buf_size（28张16位图的尺寸）
	//输出参数：raw
	//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
	DF_SDK_API int DfGetCameraRawData02(unsigned short* raw, int raw_buf_size);

	//函数名： DfGetCameraRawData03
	//功能： 采集一组8bit条纹图和一张彩色图，一共29张，4张相移条纹图 + 8张格雷码图 + 2张黑白图 + 1张彩色图
	//输入参数：raw_buf_size（28张8位图的尺寸）
	//输出参数：raw
	//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
	DF_SDK_API int DfGetCameraRawData03(unsigned char* raw, int raw_buf_size);

	//函数名： DfGetParamSmoothing
	//功能： 获取点云平滑参数
	//输入参数：无
	//输出参数：smoothing(0:关、1-5:平滑程度由低到高)
	//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
	DF_SDK_API int DfGetParamSmoothing(int& smoothing);

	//函数名： DfSetParamSmoothing
	//功能： 设置点云平滑参数
	//输入参数：smoothing(0:关、1-5:平滑程度由低到高)
	//输出参数： 无
	//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
	DF_SDK_API int DfSetParamSmoothing(int smoothing);

	//函数名： DfSetParamDepthFilter
	//功能： 设置深度图滤波参数
	//输入参数：use(开关：1开、0关)、depth_filterthreshold(阈值0-100)
	//输出参数： 无
	//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
	DF_SDK_API int DfSetParamDepthFilter(int use, float depth_filter_threshold);

	//函数名： DfGetParamDepthFilter
	//功能： 设置深度图滤波参数
	//输入参数：use(开关：1开、0关)、depth_filterthreshold(阈值0-100)
	//输出参数： 无
	//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
	DF_SDK_API int DfGetParamDepthFilter(int& use, float& depth_filter_threshold);

	//函数名： DfSetParamCameraConfidence
	//功能： 设置相机噪声过滤置信度
	//输入参数：confidence(相机置信度)
	//输出参数： 无
	//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
	DF_SDK_API int DfSetParamCameraConfidence(float confidence);

	//函数名： DfGetParamCameraConfidence
	//功能： 获取相机噪声过滤置信度
	//输入参数： 无
	//输出参数：confidence(相机置信度)
	//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
	DF_SDK_API int DfGetParamCameraConfidence(float& confidence);

	//函数名： DfSetParamLedCurrent
	//功能： 设置LED电流
	//输入参数： led（电流值）
	//输出参数： 无
	//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
	DF_SDK_API int DfSetParamLedCurrent(int led);


	//函数名： DfGetParamLedCurrent
	//功能： 设置LED电流
	//输入参数： 无
	//输出参数： led（电流值）
	//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
	DF_SDK_API int DfGetParamLedCurrent(int& led);

	//函数名： DfSetParamMixedHdr
	//功能： 设置混合多曝光参数（最大曝光次数为5次）
	//输入参数： num（曝光次数）、exposure_param（曝光参数）、led_param[5]（5个led亮度参数、前num个有效）
	//输出参数： 无
	//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
	DF_SDK_API int DfSetParamMixedHdr(int num, int exposure_param[5], int led_param[5]);

	//函数名： DfGetParamMixedHdr
	//功能： 获取混合多曝光参数（最大曝光次数为5次）
	//输入参数： 无
	//输出参数： num（曝光次数）、exposure_param（曝光参数）、led_param[5]（5个led亮度参数、前num个有效）
	//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
	DF_SDK_API int DfGetParamMixedHdr(int& num, int exposure_param[5], int led_param[5]);

	//函数名： DfSetParamStandardPlaneExternal
	//功能： 设置基准平面的外参
	//输入参数：R(旋转矩阵：3*3)、T(平移矩阵：3*1)
	//输出参数： 无
	//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
	DF_SDK_API int DfSetParamStandardPlaneExternal(float* R, float* T);

	//函数名： DfGetParamStandardPlaneExternal
	//功能： 获取基准平面的外参
	//输入参数：无
	//输出参数： R(旋转矩阵：3*3)、T(平移矩阵：3*1)
	//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
	DF_SDK_API int DfGetParamStandardPlaneExternal(float* R, float* T);

	//函数名： DfSetParamRadiusFilter
	//功能： 设置点云半径滤波参数
	//输入参数：use(开关：1开、0关)、radius(半径）、num（有效点）
	//输出参数： 无
	//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
	DF_SDK_API int DfSetParamRadiusFilter(int use, float radius, int num);

	//函数名： DfGetParamRadiusFilter
	//功能： 获取点云半径滤波参数
	//输入参数：无
	//输出参数：use(开关：1开、0关)、radius(半径）、num（有效点）
	//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
	DF_SDK_API int DfGetParamRadiusFilter(int& use, float& radius, int& num);

	//函数名： DfSetParamMultipleExposureModel
	//功能： 设置多曝光模式
	//输入参数： model(1：HDR(默认值)、2：重复曝光)
	//输出参数：无
	//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
	DF_SDK_API int DfSetParamMultipleExposureModel(int model);

	//函数名： DfSetParamRepetitionExposureNum
	//功能： 设置重复曝光数
	//输入参数： num(2-10)
	//输出参数：无
	//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
	DF_SDK_API int DfSetParamRepetitionExposureNum(int num);

	//函数名： DfCaptureData
	//功能： 采集一帧数据并阻塞至返回状态
	//输入参数： exposure_num（曝光次数）：设置值为1为单曝光，大于1为多曝光模式（具体参数在相机gui中设置）.
	//输出参数： timestamp(时间戳)
	//返回值： 类型（int）:返回0表示获取采集数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfCaptureData(int exposure_num, char* timestamp);

	//函数名： DfGetBrightnessData
	//功能： 获取亮度图
	//输入参数：无
	//输出参数： brightness(亮度图)
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfGetBrightnessData(unsigned char* brightness);

	//函数名： DfGetDepthDataFloat
	//功能： 获取深度图
	//输入参数：无
	//输出参数： depth(深度图)
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfGetDepthDataFloat(float* depth);

	//函数名： DfGetHeightMapData
	//功能： 获取校正到基准平面的高度映射图
	//输入参数：无  
	//输出参数： height_map(高度映射图)
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfGetHeightMapData(float* height_map);

	//函数名： DfGetStandardPlaneParam
	//功能： 获取基准平面参数
	//输入参数：无
	//输出参数： R(旋转矩阵：3*3)、T(平移矩阵：3*1)
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfGetStandardPlaneParam(float* R, float* T);

	//函数名： DfGetHeightMapDataBaseParam
	//功能： 获取校正到基准平面的高度映射图
	//输入参数：R(旋转矩阵)、T(平移矩阵)
	//输出参数： height_map(高度映射图)
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfGetHeightMapDataBaseParam(float* R, float* T, float* height_map);

	//函数名： DfGetPointcloudData
	//功能： 获取点云
	//输入参数：无
	//输出参数： point_cloud(点云)
	//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
	DF_SDK_API int DfGetPointcloudData(float* point_cloud);


}

//函数名： DfConnectNet（区别于DfConnect:带注册的连接，DfConnectNet：不带注册直接连接）
//功能： 连接相机
//输入参数： ip
//输出参数： 无
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfConnectNet(const char* ip);

//函数名： DfDisconnectNet
//功能： 断开相机
//输入参数：无
//输出参数： 无
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfDisconnectNet();

//函数名： DfSetParamGenerateBrightness
//功能： 设置生成亮度图参数
//输入参数：model(1:与条纹图同步连续曝光、2：单独发光曝光、3：不发光单独曝光)、exposure(亮度图曝光时间)
//输出参数： 无
//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
DF_SDK_API int DfSetParamGenerateBrightness(int model, float exposure);

//函数名： DfGetParamGenerateBrightness
//功能： 获取生成亮度图参数
//输入参数： 无
//输出参数：model(1:与条纹图同步连续曝光、2：单独发光曝光、3：不发光单独曝光)、exposure(亮度图曝光时间)
//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
DF_SDK_API int DfGetParamGenerateBrightness(int& model, float& exposure);

//函数名： depthTransformPointcloud
//功能： 深度图转点云接口
//输入参数：depth_map（深度图）
//输出参数：point_cloud_map（点云）
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int depthTransformPointcloud(float* depth_map, float* point_cloud_map);

//函数名： DfGetCalibrationParam
//功能：获取标定参数接口
//输入参数：无
//输出参数：calibration_param（标定参数）
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfGetCalibrationParam(struct CameraCalibParam& calibration_param);

//函数名： DfSetCalibrationParam
//功能：设置标定参数接口
//输入参数：calibration_param（标定参数）
//输出参数：无
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfSetCalibrationParam(const struct CameraCalibParam& calibration_param);

//函数名： DfRegisterOnDropped
//功能：注册断连回调函数
//输入参数：无
//输出参数：p_function（回调函数）
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfRegisterOnDropped(int (*p_function)(void*));

//函数名： DfGetCalibrationParam
//功能：获取相机配置参数接口
//输入参数：config_param（配置参数）
//输出参数：无
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfGetSystemConfigParam(struct SystemConfigParam& config_param);

//函数名： DfGetCalibrationParam
//功能：设置相机配置参数接口
//输入参数：config_param（配置参数）
//输出参数：无
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfSetSystemConfigParam(const struct SystemConfigParam& config_param);

//函数名： transformPointcloud
//功能： 点云坐标系转换接口
//输入参数：rotate（旋转矩阵）、translation（平移矩阵）
//输出参数：point_cloud_map（点云）
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API bool transformPointcloud(float* org_point_cloud_map, float* transform_point_cloud_map, float* rotate, float* translation);

//函数名： DfGetDeviceTemperature
//功能：获取设备温度
//输入参数：无
//输出参数：temperature（摄氏度）
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfGetDeviceTemperature(float& temperature);

//函数名： DfSetParamBilateralFilter
//功能： 设置双边滤波参数
//输入参数： use（开关：1为开、0为关）、param_d（平滑系数：3、5、7、9、11）
//输出参数： 无
//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
DF_SDK_API int DfSetParamBilateralFilter(int use, int param_d);

//函数名： DfGetParamBilateralFilter
//功能： 获取混合多曝光参数（最大曝光次数为6次）
//输入参数： 无
//输出参数： use（开关：1为开、0为关）、param_d（平滑系数：3、5、7、9、11）
//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
DF_SDK_API int DfGetParamBilateralFilter(int& use, int& param_d);

//函数名：  DfGetProjectVersion
//功能：    获取相机型号
//输入参数：无
//输出参数：型号（3010、4710）
//返回值：  类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfGetProjectorVersion(int& version);

//函数名：  DfGetFirmwareVersion
//功能：    获取固件版本
//输入参数：版本号缓冲区地址，缓冲区长度
//输出参数：版本号
//返回值：  类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfGetFirmwareVersion(char* pVersion, int length);

//函数名： DfGetProductInfo
//功能： 设置标定板检测
//输入参数：info(信息)，lenth(信息长度) 
//输出参数： 无
//返回值： 类型（int）:返回0表示获取数据成功;否则表示获取数据失败.
DF_SDK_API int DfGetProductInfo(char* info, int length);

//函数名：  DfGetNetworkBandwidth
//功能：    获取链路的网络带宽
//输入参数：无
//输出参数：网络带宽大小
//返回值：  类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfGetNetworkBandwidth(int& speed);

//函数名： DfGetFrameStatus
//功能： 获取当前帧数据状态
//输入参数：无
//输出参数： status（状态码）
//返回值： 类型（int）:返回0表示获取数据成功;否则表示获取数据失败.
DF_SDK_API int DfGetFrameStatus(int& status);

//函数名： DfSetParamMixedHdr
//功能： 设置混合多曝光参数（最大曝光次数为5次）
//输入参数： num（曝光次数）、exposure_param[5]（5个曝光参数、前num个有效）、led_param[5]（5个led亮度参数、前num个有效）
//输出参数： 无
//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
DF_SDK_API int DfSetParamMixedHdr(int num, int exposure_param[5], int led_param[5]);

//函数名： DfGetParamMixedHdr
//功能： 获取混合多曝光参数（最大曝光次数为5次）
//输入参数： 无
//输出参数： num（曝光次数）、exposure_param[5]（5个曝光参数、前num个有效）、led_param[5]（5个led亮度参数、前num个有效）
//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
DF_SDK_API int DfGetParamMixedHdr(int& num, int exposure_param[5], int led_param[5]);
