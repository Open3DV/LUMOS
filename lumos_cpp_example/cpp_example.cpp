// example.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <string.h>
#include "lcamera.h"
#include "enumerate.h"

using namespace LUMOS;

int main()
{
	/*****************************************************************************************************/
	int ret_code = 0;

	//更新相机设备列表
	int camera_num = 0;
	ret_code = DfUpdateDeviceList(1, camera_num);
	if (0 != ret_code || 0 == camera_num)
	{
		return -1;
	}

	DeviceBaseInfo* pBaseinfo = (DeviceBaseInfo*)malloc(sizeof(DeviceBaseInfo) * camera_num);
	int n_size = camera_num * sizeof(DeviceBaseInfo);
	//获取设备信息
	ret_code = DfGetAllDeviceBaseInfo(pBaseinfo, &n_size);
	for (int i = 0; i < camera_num; i++)
	{
		std::cout << "mac: " << pBaseinfo[i].mac << "  ip: " << pBaseinfo[i].ip << std::endl;
	}


	//创建相机
	LCamera* p_camera = (LCamera*)createLCamera();

	char version[64] = "";


	//连接相机 
	ret_code = p_camera->connect(pBaseinfo[0].ip);
	//ret_code = p_camera->connect("192.168.4.221");

	int width = 0, height = 0;

	int channels = 0;

	if (0 == ret_code)
	{
		//连接相机后要调用 DfSelectCamera() 先行选择所需的相机坐标系
		p_camera->selectCamera(LumosCameraSelect::RGBCamera);

		//必须连接相机成功后，才可获取相机分辨率
		ret_code = p_camera->getCameraResolution(&width, &height);
		std::cout << "Width: " << width << "    Height: " << height << std::endl;

		// 获取图像通道数

		ret_code = p_camera->getCameraChannels(&channels);
	}
	else
	{
		std::cout << "Connect Camera Error!";
		return -1;
	}

	//获取相机的标定参数
	CalibrationParam calib_param;
	ret_code = p_camera->getCalibrationParam(&calib_param);

	if (0 == ret_code)
	{
		std::cout << "intrinsic: " << std::endl;
		for (int r = 0; r < 3; r++)
		{
			for (int c = 0; c < 3; c++)
			{
				std::cout << calib_param.intrinsic[3 * r + c] << "\t";
			}
			std::cout << std::endl;
		}

		std::cout << "extrinsic: " << std::endl;
		for (int r = 0; r < 4; r++)
		{
			for (int c = 0; c < 4; c++)
			{
				std::cout << calib_param.extrinsic[4 * r + c] << "\t";
			}
			std::cout << std::endl;
		}

		std::cout << "distortion: " << std::endl;
		for (int r = 0; r < 1; r++)
		{
			for (int c = 0; c < 12; c++)
			{
				std::cout << calib_param.distortion[1 * r + c] << "\t";
			}
			std::cout << std::endl;
		}
	}
	else
	{
		std::cout << "Get Calibration Data Error!" << std::endl;
		return -1;
	}

	//分配内存保存采集结果
	float* point_cloud_data = (float*)malloc(sizeof(float) * width * height * 3);
	memset(point_cloud_data, 0, sizeof(float) * width * height * 3);

	float* height_map_data = (float*)malloc(sizeof(float) * width * height);
	memset(height_map_data, 0, sizeof(float) * width * height);

	float* depth_data = (float*)malloc(sizeof(float) * width * height);
	memset(depth_data, 0, sizeof(float) * width * height);

	char* timestamp_data = (char*)malloc(sizeof(char) * 30);
	memset(timestamp_data, 0, sizeof(char) * 30);

	unsigned char* brightness_data = (unsigned char*)malloc(sizeof(unsigned char) * width * height * channels);
	memset(brightness_data, 0, sizeof(unsigned char) * width * height * channels);

	int capture_num = 0;

	if (0 == ret_code)
	{
		ret_code = p_camera->setParamCameraConfidence(10);
		if (0 != ret_code)
		{
			std::cout << "Set Camera Confidence Error!" << std::endl;
		}

		ret_code = p_camera->setParamCameraGain(0.);
		if (0 != ret_code)
		{
			std::cout << "Set Camera Gain Error!" << std::endl;
		}

		ret_code = p_camera->setParamSmoothing(1);
		if (0 != ret_code)
		{
			std::cout << "Set Pointcloud Smoothing Error!" << std::endl;
		}

		//采集单曝光数据
		if (false)
		{
			//设置投影亮度参数
			ret_code = p_camera->setParamLedCurrent(1023);
			if (0 != ret_code)
			{
				std::cout << "Set LED Current Error!" << std::endl;
			}

			//设置相机曝光时间（us）
			ret_code = p_camera->setParamCameraExposure(30000);
			if (0 != ret_code)
			{
				std::cout << "Set Camera Exposure Error!" << std::endl;
			}

			//采集一帧单次曝光的数据
			ret_code = p_camera->captureData(1, timestamp_data);
			std::cout << "Capture Single Exposure Data" << std::endl;
			std::cout << "timestamp: " << timestamp_data << std::endl;
		}
		else
		{

			//采集HDR模式数据 
			int num = 2;
			int led_param[5] = { 1023,1023,1023,1023,1023 };
			int exposure_param[5] = { 10000, 20000, 40000, 60000, 80000 };

			//设置多曝光参数
			ret_code = p_camera->setParamMixedHdr(num, exposure_param, led_param);

			if (0 != ret_code)
			{
				std::cout << "Set HDR Param Error;" << std::endl;
			}

			ret_code = p_camera->captureData(num, timestamp_data);
			std::cout << "Capture HDR Data" << std::endl;
			std::cout << "timestamp: " << timestamp_data << std::endl;


		}


		if (0 == ret_code)
		{
			//获取亮度图数据
			ret_code = p_camera->getBrightnessData(brightness_data);
			if (0 == ret_code)
			{
				std::cout << "Get Brightness!" << std::endl;
			}

			//获取深度图数据
			ret_code = p_camera->getDepthDataFloat(depth_data);

			if (0 == ret_code)
			{
				std::cout << "Get Depth!" << std::endl;
			}

			//获取高度映射图数据
			ret_code = p_camera->getHeightMapData(height_map_data);

			if (0 == ret_code)
			{
				std::cout << "Get Height Map!" << std::endl;
			}

			//获取点云数据
			ret_code = p_camera->getPointcloudData(point_cloud_data);
			if (0 == ret_code)
			{
				std::cout << "Get Pointcloud!" << std::endl;
			}

			//动态获取基准平面高度映射图
			float plane_R[9] = { 1,0,0,0,1,0,0,0,1 };
			float plane_T[3] = { 0,0,0 };

			//动态获取高度映射图数据
			ret_code = p_camera->getHeightMapDataBaseParam(plane_R, plane_T, height_map_data);
			if (0 == ret_code)
			{
				std::cout << "Get Height Map Base Param!" << std::endl;
			}

			capture_num++;
			std::cout << "Capture num: " << capture_num << std::endl;

		}
		else
		{

			std::cout << "Capture Data Error!" << std::endl;

		}

	}

	free(brightness_data);
	free(depth_data);
	free(point_cloud_data);
	free(height_map_data);
	free(timestamp_data);
	free(pBaseinfo);

	p_camera->disconnect(pBaseinfo[0].ip);
	//p_camera->disconnect("192.168.4.221");

	destroyLCamera(p_camera);

}


