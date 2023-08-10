#pragma once
#ifdef _WIN32 
#include "../SDK/laser_3d_cam_dev.h" 
#include <windows.h>
#elif __linux
#include "../SDK/laser_3d_cam_dev.h" 
#include <cstring>
#include <stdio.h> 
#define fopen_s(pFile,filename,mode) ((*(pFile))=fopen((filename),  (mode)))==NULL
#endif 
#include "../firmware/system_config_settings.h"
#include "../firmware/protocol.h"
//#include "../firmware/version.h"
//#include "../test/solution.h"
#include "opencv2/opencv.hpp"
#include <assert.h>
#include <fstream>
#include <string.h>
#include "getopt.h" 
#include <iomanip>
//#include "../test/LookupTableFunction.h"
#include "../test/triangulation.h"

using namespace std;

const char* help_info1 =
"Examples:\n\
\n\
1.Get Frame 01: \n\
laser_3d_cam.exe --get-frame-01 --ip 192.168.x.x --path ./frame_01\n\
\n\
2.Get calibration parameters: \n\
laser_3d_cam.exe --get-calib-param --ip 192.168.x.x --path ./param.txt\n\
\n\
3.Set calibration parameters: \n\
laser_3d_cam.exe --set-calib-param --ip 192.168.x.x --path ./param.txt\n\
\n\
4.Get raw images (Mode 01): \n\
laser_3d_cam.exe --get-raw-01 --ip 192.168.x.x --path ./raw01_image_dir\n\
\n\
5.Get raw images (Mode 02): \n\
laser_3d_cam.exe --get-raw-02 --ip 192.168.x.x --path ./raw02_image_dir\n\
\n\
6.Get raw images (Mode 03): \n\
laser_3d_cam.exe --get-raw-03 --ip 192.168.x.x --path ./raw03_image_dir\n\
\n\
7.Set Brightness Param: \n\
laser_3d_cam.exe --set-brightness-param --ip 192.168.x.x --model 2 --exposure 12000\n\
\n\
8.Get Brightness Param: \n\
laser_3d_cam.exe --get-brightness-param --ip 192.168.x.x\n\
\n\
9.Set Camera Exposure Param: \n\
laser_3d_cam.exe --set-camera-exposure-param --ip 192.168.x.x --exposure 12000\n\
\n\
10.Get Camera Exposure Param: \n\
laser_3d_cam.exe --get-camera-exposure-param --ip 192.168.x.x\n\
\n\
11.Set Camera Gain: \n\
laser_3d_cam.exe --set-camera-gain --ip 192.168.x.x --gain 1023\n\
\n\
12.Get Camera Gain: \n\
laser_3d_cam.exe --get-camera-gain --ip 192.168.x.x\n\
\n\
13.Get Frame test:\n\
laser_3d_cam.exe --get-frame-test --ip 192.168.x.x --path ./frame_test\n\
\n\
14.Set HDR Param: \n\
laser_3d_cam.exe --set-hdr-param --ip 192.168.x.x --path ./hdr_params.xml\n\
\n\
15.Get Frame 01 HDR:\n\
laser_3d_cam.exe --get-frame-01-hdr --ip 192.168.x.x --path ./frame_01_hdr\n\
\n\
16.Get Frame 03: \n\
laser_3d_cam.exe --get-frame-03 --ip 192.168.x.x --path ./frame_03\n\
\n\
17.Get Frame 04: \n\
laser_3d_cam.exe --get-frame-04 --ip 192.168.x.x --path ./frame_04\n\
18.Get Frame 04 hdr: \n\
laser_3d_cam.exe --get-frame-04-hdr --ip 192.168.x.x --path ./frame_04_hdr\n\
\n\
\n\
";

const char* help_info =
"Examples:\n\
\n\
1.Get Frame 01: 采集并保存一帧亮度图、深度图、点云\n\
laser_3d_cam.exe --get-frame-01 --ip 192.168.x.x --path ./frame_01\n\
\n\
2.Get raw images (Mode 01): 采集并保存所有的条纹图片\n\
laser_3d_cam.exe --get-raw-01 --ip 192.168.x.x --path ./raw01_image_dir\n\
\n\
3.Get raw images (Mode 02): 采集并保存所有的条纹图片\n\
laser_3d_cam.exe --get-raw-02 --ip 192.168.x.x --path ./raw02_image_dir\n\
\n\
4.Get raw images (Mode 03): 采集并保存所有的条纹图片\n\
laser_3d_cam.exe --get-raw-03 --ip 192.168.x.x --path ./raw03_image_dir\n\
\n\
5.Set Camera Exposure Param: 设置相机曝光时间（10000us - 100000us）\n\
laser_3d_cam.exe --set-camera-exposure-param --ip 192.168.x.x --exposure 12000\n\
\n\
6.Get Camera Exposure Param: 获取相机曝光时间\n\
laser_3d_cam.exe --get-camera-exposure-param --ip 192.168.x.x\n\
\n\
7.Set Camera Gain: 设置相机增益\n\
laser_3d_cam.exe --set-camera-gain --ip 192.168.x.x --gain 5\n\
\n\
8.Get Camera Gain: 获取相机增益\n\
laser_3d_cam.exe --get-camera-gain --ip 192.168.x.x\n\
\n\
9.Get Frame test: 获取相机的一帧图像亮度图、深度图、点云以及条纹图\n\
laser_3d_cam.exe --get-frame-test --ip 192.168.x.x --path ./frame_test\n\
\n\
10.Set HDR Param: 设置激光的动态曝光亮度参数（运行得到默认的文件：hdr_params.xml；修改文件：hdr_params.xml；亮度：0-1023，组数：2-5，目前不支持曝光时间设置，曝光时间统一为“Set Camera Exposure Param”所指定；要求：1.HDR最后一组的亮度应为1023最亮；2.HDR的亮度应为递增）\n\
laser_3d_cam.exe --set-hdr-param --ip 192.168.x.x --path ./hdr_params.xml\n\
\n\
11.Get Frame 01 HDR: 获取一帧的亮度数据\n\
laser_3d_cam.exe --get-frame-01-hdr --ip 192.168.x.x --path ./frame_01_hdr\n\
\n\
12.Set Camera Gamma: 设置相机Gamma矫正\n\
laser_3d_cam.exe --set-camera-gamma --ip 192.168.x.x --gamma 0.5\n\
\n\
13.Get Camera Gamma: 获取相机Gamma矫正\n\
laser_3d_cam.exe --get-camera-gamma --ip 192.168.x.x\n\
\n\
14.Get Frame 03: 获取一帧彩色点云\n\
laser_3d_cam.exe --get-frame-03 --ip 192.168.x.x --path ./frame_03\n\
\n\
15.Get Frame 04: 获取一帧彩色点云（基于RGB相机坐标系）\n\
laser_3d_cam.exe --get-frame-04 --ip 192.168.x.x --path ./frame_04\n\
\n\
16.Get Frame 04 hdr: 获取一帧彩色点云HDR（基于RGB相机坐标系）\n\
laser_3d_cam.exe --get-frame-04-hdr --ip 192.168.x.x --path ./frame_04_hdr\n\
\n\
";

void help_with_version(const char* help);
int get_frame_01(const char* ip, const char* frame_path);
int get_frame_01_hdr(const char* ip, const char* frame_path);
int get_frame_03(const char* ip, const char* frame_path);
int get_frame_04(const char* ip, const char* frame_path);
int get_frame_04_hdr(const char* ip, const char* frame_path);
int get_frame_test(const char* ip, const char* frame_path);
void save_frame(int image_height, int image_width, float* depth_buffer, unsigned char* bright_buffer, const char* frame_path);
void save_images(const char* raw_image_dir, unsigned char* buffer, int width, int height, int image_num);
void save_images_16bit(const char* raw_image_dir, unsigned short* buffer, int width, int height, int image_num);
void save_color_point_cloud(float* point_cloud_buffer, unsigned char* brightness_buffer, const char* pointcloud_path);
void write_fbin(std::ofstream& out, float val);
void write_fbin(std::ofstream& out, unsigned char val);
bool SaveBinPointsToPly(cv::Mat deep_mat, string path, cv::Mat texture_map);
bool convertDepthToColor(cv::Mat& deep_mat, cv::Mat& color_texture_map, cv::Mat& depth2color_map, cv::Mat& output_color_depth);
int on_dropped(void* param);
int get_raw_01(const char* ip, const char* raw_image_dir);
int get_raw_02(const char* ip, const char* raw_image_dir);
int get_raw_03(const char* ip, const char* raw_image_dir);
int get_calib_param(const char* ip, const char* calib_param_path);
int set_calib_param(const char* ip, const char* calib_param_path);
int set_generate_brightness_param(const char* ip, int model, float exposure);
int get_generate_brightness_param(const char* ip, int& model, float& exposure);
int set_camera_exposure_param(const char* ip, float exposure);
int get_camera_exposure_param(const char* ip, float& exposure);
int set_camera_gain(const char* ip, float exposure);
int get_camera_gain(const char* ip, float& exposure);
int percentile(cv::Mat& image, int percent);
bool depthToDepthColor(cv::Mat depth_map, cv::Mat& color_map, cv::Mat& grey_map, float low_z, float high_z);
bool maskZMap(cv::Mat& z_map, cv::Mat mask);
int set_hdr_param(const char* ip, const char* param_path);
bool read_hdr_param_from_file(const char* param_path, int& hdr_count, int* exposure_time, int* brightness);
int set_camera_gamma(const char* ip, float gamma);
int get_camera_gamma(const char* ip, float& gamma);

extern int optind, opterr, optopt;
extern char* optarg;

enum opt_set
{
	IP,
	PATH,
	GET_CALIB_PARAM,
	SET_CALIB_PARAM,
	GET_RAW_01,
	GET_RAW_02,
	GET_RAW_03,
	GET_FRAME_01,
	GET_FRAME_01_HDR,
	GET_FRAME_03,
	GET_FRAME_04,
	GET_FRAME_04_HDR,
	GET_FRAME_TEST,
	HELP,
	SET_GENERATE_BRIGHTNESS,
	GET_GENERATE_BRIGHTNESS,
	MODEL,
	EXPOSURE,
	SET_CAMERA_EXPOSURE,
	GET_CAMERA_EXPOSURE,
	SET_CAMERA_GAIN,
	GET_CAMERA_GAIN,
	OFFSET,
	GAIN,
	LINE_INDEX,
	SET_HDR_PARAM,
	SET_CAMERA_GAMMA,
	GET_CAMERA_GAMMA,
	GAMMA
};

static struct option long_options[] =
{
	{"ip",required_argument,NULL,IP},
	{"path", required_argument, NULL, PATH},
	{"model", required_argument, NULL, MODEL},
	{"exposure", required_argument, NULL, EXPOSURE},
	{"offset", required_argument, NULL, OFFSET},
	{"gain", required_argument, NULL, GAIN},
	{"line-index", required_argument, NULL, LINE_INDEX},
	{"gamma", required_argument, NULL, GAMMA},
	{"get-calib-param",no_argument,NULL,GET_CALIB_PARAM},
	{"set-calib-param",no_argument,NULL,SET_CALIB_PARAM},
	{"get-raw-01",no_argument,NULL,GET_RAW_01},
	{"get-raw-02",no_argument,NULL,GET_RAW_02},
	{"get-raw-03",no_argument,NULL,GET_RAW_03},
	{"get-frame-01",no_argument,NULL,GET_FRAME_01},
	{"get-frame-01-hdr",no_argument,NULL,GET_FRAME_01_HDR},
	{"get-frame-03",no_argument,NULL,GET_FRAME_03},
	{"get-frame-04",no_argument,NULL,GET_FRAME_04},
	{"get-frame-04-hdr",no_argument,NULL,GET_FRAME_04_HDR},
	{"get-frame-test",no_argument,NULL,GET_FRAME_TEST},
	{"help",no_argument,NULL,HELP},
	{"set-brightness-param",no_argument,NULL,SET_GENERATE_BRIGHTNESS},
	{"get-brightness-param",no_argument,NULL,GET_GENERATE_BRIGHTNESS},
	{"set-camera-exposure-param",no_argument,NULL,SET_CAMERA_EXPOSURE},
	{"get-camera-exposure-param",no_argument,NULL,GET_CAMERA_EXPOSURE},
	{"set-camera-gain",no_argument,NULL,SET_CAMERA_GAIN},
	{"get-camera-gain",no_argument,NULL,GET_CAMERA_GAIN},
	{"set-hdr-param",no_argument,NULL,SET_HDR_PARAM},
	{"set-camera-gamma",no_argument,NULL,SET_CAMERA_GAMMA},
	{"get-camera-gamma",no_argument,NULL,GET_CAMERA_GAMMA},
};

const char* camera_id = NULL;
const char* path = NULL;
const char* c_model = NULL;
const char* c_exposure = NULL;
const char* c_offset = NULL;
const char* c_gain = NULL;
const char* c_line_index = NULL;
const char* c_gamma = NULL;

int command = HELP;

struct CameraCalibParam calibration_param_;

int main(int argc, char* argv[])
{


	int c = 0;

	while (EOF != (c = getopt_long(argc, argv, "i:h", long_options, NULL)))
	{
		switch (c)
		{
		case IP:
			camera_id = optarg;
			break;
		case PATH:
			path = optarg;
			break;
		case MODEL:
			c_model = optarg;
			break;
		case EXPOSURE:
			c_exposure = optarg;
			break;
		case OFFSET:
			c_offset = optarg;
			break;
		case GAIN:
			c_gain = optarg;
			break;
		case LINE_INDEX:
			c_line_index = optarg;
			break;
		case GAMMA:
			c_gamma = optarg;
			break;
		case '?':
			printf("unknow option:%c\n", optopt);
			break;
		default:
			command = c;
			break;
		}
	}

	switch (command)
	{
	case HELP:
		help_with_version(help_info);
		break;
	case GET_CALIB_PARAM:
		get_calib_param(camera_id, path);
		break;
	case SET_CALIB_PARAM:
		set_calib_param(camera_id, path);
		break;
	case GET_RAW_01:
		get_raw_01(camera_id, path);
		break;
	case GET_RAW_02:
		get_raw_02(camera_id, path);
		break;
	case GET_RAW_03:
		get_raw_03(camera_id, path);
		break;
	case GET_FRAME_01:
		get_frame_01(camera_id, path);
		break;
	case GET_FRAME_01_HDR:
		get_frame_01_hdr(camera_id, path);
		break;
	case GET_FRAME_03:
		get_frame_03(camera_id, path);
		break;
	case GET_FRAME_04:
		get_frame_04(camera_id, path);
		break;
	case GET_FRAME_04_HDR:
		get_frame_04_hdr(camera_id, path);
		break;
	case GET_FRAME_TEST:
		get_frame_test(camera_id, path);
		break;
	case SET_GENERATE_BRIGHTNESS:
	{
		int model = std::atoi(c_model);
		float exposure = std::atof(c_exposure);
		set_generate_brightness_param(camera_id, model, exposure);
	}
	break;
	case GET_GENERATE_BRIGHTNESS:
	{
		int model = 0;
		float exposure = 0;
		get_generate_brightness_param(camera_id, model, exposure);
	}
	break;
	case SET_CAMERA_EXPOSURE:
	{
		float exposure = std::atof(c_exposure);
		set_camera_exposure_param(camera_id, exposure);
	}
	break;
	case GET_CAMERA_EXPOSURE:
	{
		float exposure = 0;
		get_camera_exposure_param(camera_id, exposure);
	}
	break;
	case SET_CAMERA_GAIN:
	{
		float gain = std::atof(c_gain);
		set_camera_gain(camera_id, gain);
	}
	break;
	case GET_CAMERA_GAIN:
	{
		float gain = 0;
		get_camera_gain(camera_id, gain);
	}
	break;
	case SET_HDR_PARAM:
	{
		set_hdr_param(camera_id, path);
	}
	break;
	case SET_CAMERA_GAMMA:
	{
		float gamma = std::atof(c_gamma);
		set_camera_gamma(camera_id, gamma);
	}
	break;
	case GET_CAMERA_GAMMA:
	{
		float gamma = 0;
		get_camera_gamma(camera_id, gamma);
	}
	break;
	default:
		break;
	}

	return 0;
}

void help_with_version(const char* help)
{
	char info[100 * 1024] = { '\0' };
	char version[] = "_VERSION_";
	char enter[] = "\n";

#ifdef _WIN32 
	strcpy_s(info, sizeof(enter), enter);
	strcat_s(info, sizeof(info), version);
	strcat_s(info, sizeof(info), enter);
	strcat_s(info, sizeof(info), enter);
	strcat_s(info, sizeof(info), help);


#elif __linux
	strncpy(info, enter, sizeof(enter));
	strncat(info, version, sizeof(info));
	strncat(info, enter, sizeof(info));
	strncat(info, enter, sizeof(info));
	strncat(info, help, sizeof(info));

#endif 


	printf(info);

}

int on_dropped(void* param)
{
	std::cout << "Network dropped!" << std::endl;
	return 0;
}

void save_frame(int image_height, int image_width, float* depth_buffer, unsigned char* bright_buffer, const char* frame_path)
{
	std::string folderPath = frame_path;

	cv::Mat depth_map(image_height, image_width, CV_32F, depth_buffer);
	cv::Mat bright_map(image_height, image_width, CV_8U, bright_buffer);


	std::string depth_path = folderPath + "_depth.tiff";
	cv::imwrite(depth_path, depth_map);
	std::cout << "save depth: " << depth_path << "\n";

	std::string bright_path = folderPath + "_brightness.bmp";
	cv::imwrite(bright_path, bright_map);
	std::cout << "save brightness: " << bright_path << "\n";

}

void save_frame(int image_height, int image_width, int image_channels, float* depth_buffer, unsigned char* bright_buffer, const char* frame_path)
{
	std::string folderPath = frame_path;

	cv::Mat depth_map(image_height, image_width, CV_32F, depth_buffer);
	cv::Mat bright_map;

	if (image_channels == 1)
	{
		bright_map = cv::Mat(image_height, image_width, CV_8U, bright_buffer);
	}
	else if (image_channels == 3)
	{
		bright_map = cv::Mat(image_height, image_width, CV_8UC3, bright_buffer);
	}


	std::string depth_path = folderPath + "_depth.tiff";
	cv::imwrite(depth_path, depth_map);
	std::cout << "save depth: " << depth_path << "\n";

	std::string bright_path = folderPath + "_brightness.bmp";
	cv::imwrite(bright_path, bright_map);
	std::cout << "save brightness: " << bright_path << "\n";

}

bool depth_to_xyz(cv::Mat& depth_input, cv::Mat& bright_map, cv::Mat& Q_map, const char* frame_path)
{
	cv::Mat camera_intrinsic = (cv::Mat_<float>(3, 3) <<
		Q_map.at<float>(2, 3), 0, -Q_map.at<float>(0, 3),
		0, Q_map.at<float>(2, 3), -Q_map.at<float>(1, 3),
		0, 0, 1);

	std::cout << "depth_input.rows" << depth_input.rows << std::endl;
	std::cout << "depth_input.cols" << depth_input.cols << std::endl;

	std::string folder_path = frame_path;
	std::string pointcloud_path = folder_path + "_pointcloud.xyz";

	std::fstream handle(pointcloud_path, std::ios::out);
	int width = depth_input.cols;
	int height = depth_input.rows;


	for (int row = 0; row < height; row += 1)
	{
		float* depth_input_ptr = depth_input.ptr<float>(row);
		unsigned char* bright_ptr = bright_map.ptr<unsigned char>(row);

		for (int col = 0; col < width; col += 1)
		{
			if (depth_input_ptr[col] <= 0)
			{
				continue;
			}
			float x = depth_input_ptr[col] * ((col - camera_intrinsic.at<float>(0, 2)) / camera_intrinsic.at<float>(0, 0));
			float y = depth_input_ptr[col] * ((row - camera_intrinsic.at<float>(1, 2)) / camera_intrinsic.at<float>(1, 1));

			handle << x << "," << y << "," << depth_input_ptr[col] << "," << (int)bright_ptr[col] << "," << (int)bright_ptr[col] << "," << (int)bright_ptr[col] << "\n";

		}
	}
	handle.close();
	return true;
}

bool depth_to_point_cloud_map(cv::Mat& depth_input, cv::Mat& Q_map, cv::Mat& pointCloudMat)
{
	cv::Mat camera_intrinsic = (cv::Mat_<float>(3, 3) <<
		Q_map.at<float>(2, 3), 0, -Q_map.at<float>(0, 3),
		0, Q_map.at<float>(2, 3), -Q_map.at<float>(1, 3),
		0, 0, 1);

	std::cout << "depth_input.rows" << depth_input.rows << std::endl;
	std::cout << "depth_input.cols" << depth_input.cols << std::endl;

	cv::Mat points_map(depth_input.rows, depth_input.cols, CV_32FC3, cv::Scalar(0, 0, 0));


	int width = depth_input.cols;
	int height = depth_input.rows;


	for (int row = 0; row < height; row += 1)
	{
		float* depth_input_ptr = depth_input.ptr<float>(row);
		cv::Vec3f* pointsMapPtr = points_map.ptr<cv::Vec3f>(row);

		for (int col = 0; col < width; col += 1)
		{
			if (depth_input_ptr[col] <= 0)
			{
				continue;
			}
			pointsMapPtr[col][0] = depth_input_ptr[col] * ((col - camera_intrinsic.at<float>(0, 2)) / camera_intrinsic.at<float>(0, 0));
			pointsMapPtr[col][1] = depth_input_ptr[col] * ((row - camera_intrinsic.at<float>(1, 2)) / camera_intrinsic.at<float>(1, 1));
			pointsMapPtr[col][2] = depth_input_ptr[col];
		}
	}
	pointCloudMat = points_map;
	return true;
}

bool depth_to_point_cloud_map_use_intrinsic(cv::Mat& depth_input, cv::Mat& camera_intrinsic, cv::Mat& pointCloudMat)
{

	std::cout << "depth_input.rows" << depth_input.rows << std::endl;
	std::cout << "depth_input.cols" << depth_input.cols << std::endl;

	cv::Mat points_map(depth_input.rows, depth_input.cols, CV_32FC3, cv::Scalar(0, 0, 0));


	int width = depth_input.cols;
	int height = depth_input.rows;


	for (int row = 0; row < height; row += 1)
	{
		float* depth_input_ptr = depth_input.ptr<float>(row);
		cv::Vec3f* pointsMapPtr = points_map.ptr<cv::Vec3f>(row);

		for (int col = 0; col < width; col += 1)
		{
			if (depth_input_ptr[col] <= 0)
			{
				continue;
			}
			pointsMapPtr[col][0] = depth_input_ptr[col] * ((col - camera_intrinsic.at<float>(0, 2)) / camera_intrinsic.at<float>(0, 0));
			pointsMapPtr[col][1] = depth_input_ptr[col] * ((row - camera_intrinsic.at<float>(1, 2)) / camera_intrinsic.at<float>(1, 1));
			pointsMapPtr[col][2] = depth_input_ptr[col];
		}
	}
	pointCloudMat = points_map;
	return true;
}

bool depth_to_point_cloud_map_use_intrinsic_and_texture_roi(cv::Mat& depth_input, cv::Mat& camera_intrinsic, cv::Mat& depth2color_map, cv::Mat& input_roi_mask, cv::Mat& pointCloudMat)
{
	std::cout << "depth_input.rows" << depth_input.rows << std::endl;
	std::cout << "depth_input.cols" << depth_input.cols << std::endl;

	cv::Mat points_map(depth_input.rows, depth_input.cols, CV_32FC3, cv::Scalar(0, 0, 0));


	int width = depth_input.cols;
	int height = depth_input.rows;
	int rgb_height = input_roi_mask.rows;
	int rgb_width = input_roi_mask.cols;


	for (int row = 0; row < height; row += 1)
	{
		float* depth_input_ptr = depth_input.ptr<float>(row);
		cv::Vec3f* pointsMapPtr = points_map.ptr<cv::Vec3f>(row);

		for (int col = 0; col < width; col += 1)
		{
			int rgb_col = depth2color_map.at<ushort>(row, col * 2);
			rgb_col = rgb_col > 0 && rgb_col < rgb_height ? rgb_col : 0;

			int rgb_row = depth2color_map.at<ushort>(row, col * 2 + 1);
			rgb_row = rgb_row > 0 && rgb_row < rgb_width ? rgb_row : 0;

			if (depth_input_ptr[col] <= 0 || input_roi_mask.at<uchar>(rgb_row, rgb_col) != 255 || rgb_row == 0 || rgb_col == 0)
			{
				continue;
			}

			pointsMapPtr[col][0] = depth_input_ptr[col] * ((col - camera_intrinsic.at<float>(0, 2)) / camera_intrinsic.at<float>(0, 0));
			pointsMapPtr[col][1] = depth_input_ptr[col] * ((row - camera_intrinsic.at<float>(1, 2)) / camera_intrinsic.at<float>(1, 1));
			pointsMapPtr[col][2] = depth_input_ptr[col];
		}
	}
	pointCloudMat = points_map;
	return true;
}

bool convertDepthToRGBDepth(cv::Mat& depth_input, cv::Mat& depth_output, cv::Mat& l2rgb_r, cv::Mat& l2rgb_t, cv::Mat& camera_intrinsic, cv::Mat& rgb_camera_intrinsic)
{
	std::cout << "depth_input.rows" << depth_input.rows << std::endl;
	std::cout << "depth_input.cols" << depth_input.cols << std::endl;
	std::cout << "depth_output.rows" << depth_output.rows << std::endl;
	std::cout << "depth_output.cols" << depth_output.cols << std::endl;

	if (depth_output.cols == 0 || depth_output.rows == 0)
	{
		std::cout << "error depth_output size! " << std::endl;
		return false;
	}

	int width = depth_input.cols;
	int height = depth_input.rows;
	int rgb_height = depth_output.rows;
	int rgb_width = depth_output.cols;

	float* l2rgb_r_ptr = l2rgb_r.ptr<float>(0, 0);
	float* l2rgb_t_ptr = l2rgb_t.ptr<float>(0, 0);
	float* rgb_camera_intrinsic_ptr = rgb_camera_intrinsic.ptr<float>(0, 0);

#pragma omp parallel for
	for (int row = 0; row < height; row += 1)
	{
		float* depth_input_ptr = depth_input.ptr<float>(row);
		for (int col = 0; col < width; col += 1)
		{
			if (depth_input_ptr[col] < 1)
			{
				continue;
			}

			// 计算成点云
			float z = depth_input_ptr[col];
			float x = z * ((col - camera_intrinsic.at<float>(0, 2)) / camera_intrinsic.at<float>(0, 0));
			float y = z * ((row - camera_intrinsic.at<float>(1, 2)) / camera_intrinsic.at<float>(1, 1));

			// 然后旋转平移
			float rgb_x = l2rgb_r_ptr[0] * x + l2rgb_r_ptr[1] * y + l2rgb_r_ptr[2] * z + l2rgb_t_ptr[0];
			float rgb_y = l2rgb_r_ptr[3] * x + l2rgb_r_ptr[4] * y + l2rgb_r_ptr[5] * z + l2rgb_t_ptr[1];
			float rgb_z = l2rgb_r_ptr[6] * x + l2rgb_r_ptr[7] * y + l2rgb_r_ptr[8] * z + l2rgb_t_ptr[2];

			// 转换坐标系成为图像坐标
			int rgb_u = (rgb_camera_intrinsic_ptr[0] * rgb_x) / rgb_z + rgb_camera_intrinsic_ptr[2] + 0.5;
			int rgb_v = (rgb_camera_intrinsic_ptr[4] * rgb_y) / rgb_z + rgb_camera_intrinsic_ptr[5] + 0.5;

			// 然后判断并且赋值保存
			if (rgb_u > 0 && rgb_u < rgb_width && rgb_v > 0 && rgb_v < rgb_height)
			{
				if (depth_output.at<float>(rgb_v, rgb_u) < rgb_z)
					depth_output.at<float>(rgb_v, rgb_u) = rgb_z;
			}
			
		}
	}

	return true;
}

void save_images(const char* raw_image_dir, unsigned char* buffer, int width, int height, int image_num)
{
	std::string folderPath = raw_image_dir;
	std::string mkdir_cmd = std::string("mkdir ") + folderPath;
	system(mkdir_cmd.c_str());

	int image_size = width * height;

	for (int i = 0; i < image_num; i++)
	{
		std::stringstream ss;
		cv::Mat image(height, width, CV_8UC1, buffer + (long)(image_size * i));
		ss << std::setw(2) << std::setfill('0') << i;
		std::string filename = folderPath + "/phase" + ss.str() + ".bmp";
		cv::imwrite(filename, image);
	}
}

void save_images_16bit(const char* raw_image_dir, unsigned short* buffer, int width, int height, int image_num)
{
	std::string folderPath = raw_image_dir;
	std::string mkdir_cmd = std::string("mkdir ") + folderPath;
	system(mkdir_cmd.c_str());

	int image_size = width * height * sizeof(unsigned short);

	for (int i = 0; i < image_num; i++)
	{
		std::stringstream ss;
		cv::Mat image(height, width, CV_16UC1, buffer + (long)(image_size * i));
		ss << std::setw(2) << std::setfill('0') << i;
		std::string filename = folderPath + "/phase" + ss.str() + ".tiff";
		cv::imwrite(filename, image);
	}
}

void save_color_point_cloud(float* point_cloud_buffer, unsigned char* brightness_buffer, const char* pointcloud_path)
{
	std::ofstream ofile;
	ofile.open(pointcloud_path);
	for (int i = 0; i < 1920 * 1200; i++)
	{
		if (point_cloud_buffer[i * 3 + 2] > 0.01)
			ofile << point_cloud_buffer[i * 3] << " " << point_cloud_buffer[i * 3 + 1] << " " << point_cloud_buffer[i * 3 + 2] << " "
			<< (int)brightness_buffer[i] << " " << (int)brightness_buffer[i] << " " << (int)brightness_buffer[i] << std::endl;
	}
	ofile.close();
}

inline void write_fbin(std::ofstream& out, float val) {
	out.write(reinterpret_cast<char*>(&val), sizeof(float));
}

inline void write_fbin(std::ofstream& out, unsigned char val) {
	out.write(reinterpret_cast<char*>(&val), sizeof(unsigned char));
}

//保存Bin点云到ply文件
bool SaveBinPointsToPly(cv::Mat deep_mat, string path, cv::Mat texture_map)
{

	if (deep_mat.empty())
	{
		return false;
	}

	if (path.empty())
	{
		return false;
	}

	std::ofstream file;
	file.open(path);
	if (!file.is_open())
	{
		std::cout << "Save points Error";
		return false;
	}
	else
	{

		if (texture_map.data)
		{

			std::vector<cv::Vec3f> points_list;
			std::vector<cv::Vec3b> color_list;



			if (1 == texture_map.channels())
			{

				/****************************************************************************************************/

				string str = "";

				if (CV_32FC3 == deep_mat.type())
				{
					for (int r = 0; r < deep_mat.rows; r++)
					{
						cv::Vec3f* ptr_dr = deep_mat.ptr<cv::Vec3f>(r);
						uchar* ptr_color = texture_map.ptr<uchar>(r);

						for (int c = 0; c < deep_mat.cols; c++)
						{
							if (0 != ptr_dr[c][0] && 0 != ptr_dr[c][1] && 0 != ptr_dr[c][2])
							{

								cv::Vec3f point;
								cv::Vec3b color;

								point[0] = ptr_dr[c][0];
								point[1] = ptr_dr[c][1];
								point[2] = ptr_dr[c][2];

								color[0] = ptr_color[c];
								color[1] = ptr_color[c];
								color[2] = ptr_color[c];

								points_list.push_back(point);
								color_list.push_back(color);
							}

						}
					}
				}
				else if (CV_64FC3 == deep_mat.type())
				{
					for (int r = 0; r < deep_mat.rows; r++)
					{
						cv::Vec3d* ptr_dr = deep_mat.ptr<cv::Vec3d>(r);
						uchar* ptr_color = texture_map.ptr<uchar>(r);

						for (int c = 0; c < deep_mat.cols; c++)
						{
							if (0 != ptr_dr[c][0] && 0 != ptr_dr[c][1] && 0 != ptr_dr[c][2])
							{


								cv::Vec3f point;
								cv::Vec3b color;

								point[0] = ptr_dr[c][0];
								point[1] = ptr_dr[c][1];
								point[2] = ptr_dr[c][2];

								color[0] = ptr_color[c];
								color[1] = ptr_color[c];
								color[2] = ptr_color[c];

								points_list.push_back(point);
								color_list.push_back(color);

							}

						}
					}
				}

				/*****************************************************************************************************/
			}
			else if (3 == texture_map.channels())
			{
				/****************************************************************************************************/

				string str = "";

				if (CV_32FC3 == deep_mat.type())
				{
					for (int r = 0; r < deep_mat.rows; r++)
					{
						cv::Vec3f* ptr_dr = deep_mat.ptr<cv::Vec3f>(r);
						cv::Vec3b* ptr_color = texture_map.ptr<cv::Vec3b>(r);

						for (int c = 0; c < deep_mat.cols; c++)
						{
							if (0 != ptr_dr[c][0] && 0 != ptr_dr[c][1] && 0 != ptr_dr[c][2])
							{


								cv::Vec3f point;
								cv::Vec3b color;

								point[0] = ptr_dr[c][0];
								point[1] = ptr_dr[c][1];
								point[2] = ptr_dr[c][2];

								color[0] = ptr_color[c][2];
								color[1] = ptr_color[c][1];
								color[2] = ptr_color[c][0];

								points_list.push_back(point);
								color_list.push_back(color);

							}

						}
					}
				}
				else if (CV_64FC3 == deep_mat.type())
				{
					for (int r = 0; r < deep_mat.rows; r++)
					{
						cv::Vec3d* ptr_dr = deep_mat.ptr<cv::Vec3d>(r);
						cv::Vec3b* ptr_color = texture_map.ptr<cv::Vec3b>(r);

						for (int c = 0; c < deep_mat.cols; c++)
						{
							if (0 != ptr_dr[c][0] && 0 != ptr_dr[c][1] && 0 != ptr_dr[c][2])
							{
								cv::Vec3f point;
								cv::Vec3b color;

								point[0] = ptr_dr[c][0];
								point[1] = ptr_dr[c][1];
								point[2] = ptr_dr[c][2];

								color[0] = ptr_color[c][2];
								color[1] = ptr_color[c][1];
								color[2] = ptr_color[c][0];

								points_list.push_back(point);
								color_list.push_back(color);

							}

						}
					}
				}

				/*****************************************************************************************************/
			}


			//Header 
			file << "ply" << "\n";
			file << "format binary_little_endian 1.0" << "\n";
			file << "element vertex " << points_list.size() << "\n";
			file << "property float x" << "\n";
			file << "property float y" << "\n";
			file << "property float z" << "\n";
			file << "property uchar red" << "\n";
			file << "property uchar green" << "\n";
			file << "property uchar blue" << "\n";
			file << "end_header" << "\n";

			file.close();


			/*********************************************************************************************************/
			//以二进行制保存
			std::ofstream outFile(path, std::ios::app | std::ios::binary);


			for (int i = 0; i < points_list.size(); i++)
			{
				write_fbin(outFile, static_cast<float>(points_list[i][0]));
				write_fbin(outFile, static_cast<float>(points_list[i][1]));
				write_fbin(outFile, static_cast<float>(points_list[i][2]));
				write_fbin(outFile, static_cast<unsigned char>(color_list[i][0]));
				write_fbin(outFile, static_cast<unsigned char>(color_list[i][1]));
				write_fbin(outFile, static_cast<unsigned char>(color_list[i][2]));
			}


			outFile.close();


			/**********************************************************************************************************/

		}
		else
		{

			std::vector<cv::Vec3f> points_list;

			if (CV_32FC3 == deep_mat.type())
			{
				for (int r = 0; r < deep_mat.rows; r++)
				{
					cv::Vec3f* ptr_dr = deep_mat.ptr<cv::Vec3f>(r);
					for (int c = 0; c < deep_mat.cols; c++)
					{
						if (0 != ptr_dr[c][0] && 0 != ptr_dr[c][1] && 0 != ptr_dr[c][2])
						{

							cv::Vec3f point;

							point[0] = ptr_dr[c][0];
							point[1] = ptr_dr[c][1];
							point[2] = ptr_dr[c][2];
							points_list.push_back(point);

						}

					}
				}
			}
			else if (CV_64FC3 == deep_mat.type())
			{
				for (int r = 0; r < deep_mat.rows; r++)
				{
					cv::Vec3d* ptr_dr = deep_mat.ptr<cv::Vec3d>(r);
					for (int c = 0; c < deep_mat.cols; c++)
					{
						if (0 != ptr_dr[c][0] && 0 != ptr_dr[c][1] && 0 != ptr_dr[c][2])
						{


							cv::Vec3f point;

							point[0] = ptr_dr[c][0];
							point[1] = ptr_dr[c][1];
							point[2] = ptr_dr[c][2];
							points_list.push_back(point);

						}

					}
				}
			}

			//Header 
			file << "ply" << "\n";
			file << "format binary_little_endian 1.0" << "\n";
			file << "element vertex " << points_list.size() << "\n";
			file << "property float x" << "\n";
			file << "property float y" << "\n";
			file << "property float z" << "\n";
			file << "end_header" << "\n";
			file.close();

			/*********************************************************************************************************/
			//以二进行制保存
			std::ofstream outFile(path, std::ios::app | std::ios::binary);

			for (int i = 0; i < points_list.size(); i++)
			{
				write_fbin(outFile, static_cast<float>(points_list[i][0]));
				write_fbin(outFile, static_cast<float>(points_list[i][1]));
				write_fbin(outFile, static_cast<float>(points_list[i][2]));
			}


			outFile.close();


			/**********************************************************************************************************/
		}

		std::cout << "Save points" << path;
	}

	return true;
}

bool convertDepthToColor(cv::Mat& deep_mat, cv::Mat& color_texture_map, cv::Mat& depth2color_map, cv::Mat& output_color_depth)
{
	std:cout << "convertDepthToColor" << std::endl;
	output_color_depth = cv::Mat(deep_mat.size(), CV_8UC3);
	for (int row = 0; row < deep_mat.rows; row += 1)
	{
		for (int col = 0; col < deep_mat.cols; col += 1)
		{
			if (deep_mat.at<float>(row, col) > 0)
			{
				int rgb_u = depth2color_map.at<unsigned short>(row, 2 * col);
				int rgb_v = depth2color_map.at<unsigned short>(row, 2 * col + 1);
				if (rgb_u < color_texture_map.cols && rgb_u > 0 && rgb_v < color_texture_map.cols && rgb_v > 0)
				{
					output_color_depth.at<cv::Vec3b>(row, col) = 
						color_texture_map.at<cv::Vec3b>(rgb_v, rgb_u);
				}

			}
		}
	}
	std::cout << "convertDepthToColor finish" << std::endl;

	return true;
}

int get_frame_01(const char* ip, const char* frame_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetGrayCameraResolution(&width, &height);


	ret = DfGetCalibrationParam(calibration_param_);

	int image_size = width * height;

	int depth_buf_size = image_size * 1 * 4;
	float* depth_buf = (float*)(new char[depth_buf_size]);

	int brightness_bug_size = image_size;
	unsigned char* brightness_buf = new unsigned char[brightness_bug_size];

	ret = DfGetFrame01(depth_buf, depth_buf_size, brightness_buf, brightness_bug_size);

	DfDisconnectNet();

	save_frame(height, width, depth_buf, brightness_buf, frame_path);

	cv::Mat depthTemp(height, width, CV_32F, depth_buf);
	cv::Mat brightnessTemp(height, width, CV_8U, brightness_buf);

	//cv::Mat Q_mat(4, 4, CV_32F, Q_matrix);

	cv::Mat camera_intrinsic_temp(3, 3, CV_32F, calibration_param_.camera_intrinsic);

	//depth_to_xyz(depthTemp, brightnessTemp, Q_mat, frame_path);
	cv::Mat pointCloud;
	//depth_to_point_cloud_map(depthTemp, Q_mat, pointCloud);
	depth_to_point_cloud_map_use_intrinsic(depthTemp, camera_intrinsic_temp, pointCloud);

	std::string folder_path = frame_path;
	std::string pointcloud_path = folder_path + "_pointcloud.ply";

	SaveBinPointsToPly(pointCloud, pointcloud_path, brightnessTemp);




	delete[] depth_buf;
	delete[] brightness_buf;



	return 1;
}

int get_frame_01_hdr(const char* ip, const char* frame_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetGrayCameraResolution(&width, &height);


	ret = DfGetCalibrationParam(calibration_param_);

	int image_size = width * height;

	int depth_buf_size = image_size * 1 * 4;
	float* depth_buf = (float*)(new char[depth_buf_size]);

	int brightness_bug_size = image_size;
	unsigned char* brightness_buf = new unsigned char[brightness_bug_size];

	ret = DfGetFrame01HDR(depth_buf, depth_buf_size, brightness_buf, brightness_bug_size);

	DfDisconnectNet();

	save_frame(height, width, depth_buf, brightness_buf, frame_path);

	cv::Mat depthTemp(height, width, CV_32F, depth_buf);
	cv::Mat brightnessTemp(height, width, CV_8U, brightness_buf);

	//cv::Mat Q_mat(4, 4, CV_32F, Q_matrix);

	cv::Mat camera_intrinsic_temp(3, 3, CV_32F, calibration_param_.camera_intrinsic);

	//depth_to_xyz(depthTemp, brightnessTemp, Q_mat, frame_path);
	cv::Mat pointCloud;
	//depth_to_point_cloud_map(depthTemp, Q_mat, pointCloud);
	depth_to_point_cloud_map_use_intrinsic(depthTemp, camera_intrinsic_temp, pointCloud);

	std::string folder_path = frame_path;
	std::string pointcloud_path = folder_path + "_pointcloud.ply";

	SaveBinPointsToPly(pointCloud, pointcloud_path, brightnessTemp);




	delete[] depth_buf;
	delete[] brightness_buf;



	return 1;
}

int get_frame_03(const char* ip, const char* frame_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnect(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	int rgb_width, rgb_height;
	DfGetGrayCameraResolution(&width, &height);
	DfGetRGBCameraResolution(&rgb_width, &rgb_height);


	ret = DfGetCalibrationParam(calibration_param_);

	int image_size = width * height;
	int rgb_image_size = rgb_width * rgb_height;

	int depth_buf_size = image_size * 1 * 4;
	float* depth_buf = (float*)(new char[depth_buf_size]);

	int brightness_bug_size = image_size;
	unsigned char* brightness_buf = new unsigned char[brightness_bug_size];

	int color_brightness_buf_size = rgb_image_size * 3;
	unsigned char* color_brightness_buf = new unsigned char[color_brightness_buf_size];

	int depth2color_buf_size = image_size * sizeof(unsigned short) * 2;
	unsigned short* depth2color_buf = (unsigned short*)(new unsigned char[depth2color_buf_size]);

	ret = DfGetFrame03(depth_buf, depth_buf_size, brightness_buf, brightness_bug_size, color_brightness_buf, color_brightness_buf_size, depth2color_buf, depth2color_buf_size);

	DfDisconnect(ip);

	save_frame(height, width, depth_buf, brightness_buf, frame_path);

	cv::Mat depthTemp(height, width, CV_32F, depth_buf);
	cv::Mat brightnessTemp(height, width, CV_8U, brightness_buf);
	cv::Mat colorBrightnessTemp(rgb_height, rgb_width, CV_8UC3, color_brightness_buf);
	cv::Mat depthToColorTemp(height, width * 2, CV_16U, depth2color_buf);

	//cv::Mat Q_mat(4, 4, CV_32F, Q_matrix);

	cv::Mat camera_intrinsic_temp(3, 3, CV_32F, calibration_param_.camera_intrinsic);
	cv::Mat rgb_camera_intrinsic_temp(3, 3, CV_32F, calibration_param_.rgb_camera_intrinsic);
	cv::Mat rgb_distortion_temp(1, 5, CV_32F, calibration_param_.rgb_camera_distortion);

	//depth_to_xyz(depthTemp, brightnessTemp, Q_mat, frame_path);
	cv::Mat pointCloud;
	//depth_to_point_cloud_map(depthTemp, Q_mat, pointCloud);
	//depth_to_point_cloud_map_use_intrinsic(depthTemp, camera_intrinsic_temp, pointCloud);

	cv::Mat input_roi_mask = cv::imread("./roi.bmp", 0);
	//cv::imshow("input_roi_mask", input_roi_mask);
	//cv::waitKey(0);

	depth_to_point_cloud_map_use_intrinsic_and_texture_roi(depthTemp, camera_intrinsic_temp, depthToColorTemp, input_roi_mask, pointCloud);

	cv::Mat color_depth;
	convertDepthToColor(depthTemp, colorBrightnessTemp, depthToColorTemp, color_depth);

	std::string folder_path = frame_path;
	std::string pointcloud_path = folder_path + "_pointcloud.ply";
	std::string rgb_brightness_path = folder_path + "_color.bmp";
	std::string depth2color_path = folder_path + "_depth2color1.tiff";

	SaveBinPointsToPly(pointCloud, pointcloud_path, color_depth);

	std::cout << "save color_depth" << std::endl;
	cv::imwrite(rgb_brightness_path, colorBrightnessTemp);
	cv::imwrite(depth2color_path, color_depth);

	delete[] depth_buf;
	delete[] brightness_buf;
	delete[] color_brightness_buf;
	delete[] depth2color_buf;

	return 1;
}

int get_frame_04_(const char* ip, const char* frame_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnect(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	int rgb_width, rgb_height;
	DfGetGrayCameraResolution(&width, &height);
	DfGetRGBCameraResolution(&rgb_width, &rgb_height);


	ret = DfGetCalibrationParam(calibration_param_);

	int image_size = width * height;
	int rgb_image_size = rgb_width * rgb_height;

	int depth_buf_size = image_size * 1 * 4;
	float* depth_buf = (float*)(new char[depth_buf_size]);

	int brightness_bug_size = image_size;
	unsigned char* brightness_buf = new unsigned char[brightness_bug_size];

	int color_brightness_buf_size = rgb_image_size * 3;
	unsigned char* color_brightness_buf = new unsigned char[color_brightness_buf_size];

	int depth2color_buf_size = image_size * sizeof(unsigned short) * 2;
	unsigned short* depth2color_buf = (unsigned short*)(new unsigned char[depth2color_buf_size]);

	ret = DfGetFrame03(depth_buf, depth_buf_size, brightness_buf, brightness_bug_size, color_brightness_buf, color_brightness_buf_size, depth2color_buf, depth2color_buf_size);

	DfDisconnect(ip);

	save_frame(height, width, depth_buf, brightness_buf, frame_path);

	/****************************保存点云*********************************/
	// 先转换得到正确的RGB坐标系需要原始深度图，RGB resolution，RGB的内参和旋转
	cv::Mat camera_intrinsic_temp(3, 3, CV_32F, calibration_param_.camera_intrinsic);
	cv::Mat rgb_camera_intrinsic_temp(3, 3, CV_32F, calibration_param_.rgb_camera_intrinsic);
	cv::Mat l2rgb_r_temp(3, 3, CV_32F, calibration_param_.rotation_matrix);
	cv::Mat l2rgb_t_temp(3, 1, CV_32F, calibration_param_.translation_matrix);



	cv::Mat depthTemp(height, width, CV_32F, depth_buf);
	cv::Mat brightnessTemp(height, width, CV_8U, brightness_buf);
	cv::Mat colorBrightnessTemp(rgb_height, rgb_width, CV_8UC3, color_brightness_buf);
	cv::Mat colorBrightnessResult;
	cv::Mat depthToColorTemp(height, width * 2, CV_16U, depth2color_buf);

	cv::resize(colorBrightnessTemp, colorBrightnessResult, cv::Size(rgb_width / 2, rgb_height / 2));
	cv::Mat RGBDepthTemp(colorBrightnessResult.size(), CV_32F);

	// 然后对内参进行修正
	rgb_camera_intrinsic_temp.at<float>(0, 0) /= 2.;
	rgb_camera_intrinsic_temp.at<float>(1, 1) /= 2.;
	rgb_camera_intrinsic_temp.at<float>(0, 2) /= 2.;
	rgb_camera_intrinsic_temp.at<float>(1, 2) /= 2.;

	convertDepthToRGBDepth(depthTemp, RGBDepthTemp, l2rgb_r_temp, l2rgb_t_temp, camera_intrinsic_temp, rgb_camera_intrinsic_temp);

	// 转换深度图的坐标系

	/****************************保存点云*********************************/

	cv::Mat pointCloud;

	depth_to_point_cloud_map_use_intrinsic(RGBDepthTemp, rgb_camera_intrinsic_temp, pointCloud);

	std::string folder_path = frame_path;
	std::string pointcloud_path = folder_path + "_pointcloud.ply";
	std::string rgb_brightness_path = folder_path + "_color.bmp";
	std::string rgb_depth_path = folder_path + "_depth_of_rgb_camera.tiff";
	std::string depth2color_path = folder_path + "_depth2color1.tiff";

	std::cout << "pointCloud size" << pointCloud.size() << std::endl;

	SaveBinPointsToPly(pointCloud, pointcloud_path, colorBrightnessResult);

	cv::imwrite(rgb_brightness_path, colorBrightnessResult);
	cv::imwrite(rgb_depth_path, RGBDepthTemp);

	delete[] depth_buf;
	delete[] brightness_buf;
	delete[] color_brightness_buf;
	delete[] depth2color_buf;

	return 1;
}

int get_frame_04(const char* ip, const char* frame_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnect(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	int rgb_width, rgb_height;
	DfGetGrayCameraResolution(&width, &height);
	DfGetRGBCameraResolution(&rgb_width, &rgb_height);


	ret = DfGetCalibrationParam(calibration_param_);

	int image_size = width * height;
	int rgb_image_size = rgb_width * rgb_height;

	int depth_buf_size = image_size * 1 * 4;
	float* depth_buf = (float*)(new char[depth_buf_size]);

	int brightness_bug_size = image_size;
	unsigned char* brightness_buf = new unsigned char[brightness_bug_size];

	int color_brightness_buf_size = rgb_image_size * 3;
	unsigned char* color_brightness_buf = new unsigned char[color_brightness_buf_size];

	cv::Mat depthTemp(height, width, CV_32F, depth_buf);
	cv::Mat brightnessTemp(height, width, CV_8U, brightness_buf);
	cv::Mat colorBrightnessTemp(rgb_height, rgb_width, CV_8UC3, color_brightness_buf);
	cv::Mat colorBrightnessResult(rgb_height / 2, rgb_width / 2, CV_8UC3);
	cv::Mat RGBDepthTemp(colorBrightnessResult.size(), CV_32F, cv::Scalar(0));

	ret = DfGetFrame04(depth_buf, depth_buf_size, brightness_buf, brightness_bug_size, color_brightness_buf, color_brightness_buf_size, colorBrightnessResult.data, colorBrightnessResult.rows * colorBrightnessResult.cols * 3, (float*)RGBDepthTemp.data, RGBDepthTemp.rows * RGBDepthTemp.cols);

	DfDisconnect(ip);

	save_frame(height, width, depth_buf, brightness_buf, frame_path);

	/****************************保存点云*********************************/
	// 先转换得到正确的RGB坐标系需要原始深度图，RGB resolution，RGB的内参和旋转
	cv::Mat camera_intrinsic_temp(3, 3, CV_32F, calibration_param_.camera_intrinsic);
	cv::Mat rgb_camera_intrinsic_temp(3, 3, CV_32F, calibration_param_.rgb_camera_intrinsic);
	cv::Mat l2rgb_r_temp(3, 3, CV_32F, calibration_param_.rotation_matrix);
	cv::Mat l2rgb_t_temp(3, 1, CV_32F, calibration_param_.translation_matrix);



	//cv::Mat depthTemp(height, width, CV_32F, depth_buf);
	//cv::Mat brightnessTemp(height, width, CV_8U, brightness_buf);
	//cv::Mat colorBrightnessTemp(rgb_height, rgb_width, CV_8UC3, color_brightness_buf);
	//cv::Mat colorBrightnessResult;
	//cv::Mat depthToColorTemp(height, width * 2, CV_16U, depth2color_buf);

	//cv::resize(colorBrightnessTemp, colorBrightnessResult, cv::Size(rgb_width / 2, rgb_height / 2));

	// 然后对内参进行修正
	rgb_camera_intrinsic_temp.at<float>(0, 0) /= 2.;
	rgb_camera_intrinsic_temp.at<float>(1, 1) /= 2.;
	rgb_camera_intrinsic_temp.at<float>(0, 2) /= 2.;
	rgb_camera_intrinsic_temp.at<float>(1, 2) /= 2.;

	//convertDepthToRGBDepth(depthTemp, RGBDepthTemp, l2rgb_r_temp, l2rgb_t_temp, camera_intrinsic_temp, rgb_camera_intrinsic_temp);

	// 转换深度图的坐标系

	/****************************保存点云*********************************/

	cv::Mat pointCloud;

	depth_to_point_cloud_map_use_intrinsic(RGBDepthTemp, rgb_camera_intrinsic_temp, pointCloud);

	std::string folder_path = frame_path;
	std::string pointcloud_path = folder_path + "_pointcloud.ply";
	std::string rgb_brightness_path = folder_path + "_color.bmp";
	std::string rgb_depth_path = folder_path + "_depth_of_rgb_camera.tiff";
	std::string depth2color_path = folder_path + "_depth2color1.tiff";

	std::cout << "pointCloud size" << pointCloud.size() << std::endl;

	SaveBinPointsToPly(pointCloud, pointcloud_path, colorBrightnessResult);

	cv::imwrite(rgb_brightness_path, colorBrightnessResult);
	cv::imwrite(rgb_depth_path, RGBDepthTemp);

	delete[] depth_buf;
	delete[] brightness_buf;
	delete[] color_brightness_buf;

	return 1;
}

int get_frame_user(const char* ip, const char* frame_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnect(ip);
	if (ret != DF_SUCCESS)
	{
		return 0;
	}

	DfSelectCamera(LumosCameraSelect::RGBCamera);

	int width, height, channels;
	DfGetGrayCameraResolution(&width, &height);
	DfGetCameraChannels(&channels);

	CalibrationParam calib_param;
	ret = DfGetCalibrationParam(&calib_param);

	int image_size = width * height;

	std::cout << "width: " << width << std::endl;
	std::cout << "height: " << height << std::endl;

	int depth_buf_size = image_size * 1 * 4;
	float* depth_buf = (float*)(new char[depth_buf_size]);

	int brightness_bug_size = image_size * channels;
	unsigned char* brightness_buf = new unsigned char[brightness_bug_size];

	int pointcloud_buf_size = image_size * 3 * 4;
	float* pointcloud_buf = (float*)(new char[pointcloud_buf_size]);

	cv::Mat depthTemp(height, width, CV_32F, depth_buf);
	cv::Mat brightnessTemp;

	if (channels == 1)
	{
		brightnessTemp = cv::Mat(height, width, CV_8U, brightness_buf);
	}
	else if (channels == 3)
	{
		brightnessTemp = cv::Mat(height, width, CV_8UC3, brightness_buf);
	}

	int num = 3;
	int led_param[5] = { 1023,1023,1023,1023,1023 };
	int exposure_param[5] = { 25000, 35000, 45000, 60000, 80000 };

	//设置多曝光参数
	ret = DfSetParamMixedHdr(num, exposure_param, led_param);

	char time_stamp[50];
	ret = DfCaptureData(3, time_stamp);

	DfGetBrightnessData(brightness_buf);
	DfGetDepthDataFloat(depth_buf);
	DfGetPointcloudData(pointcloud_buf);

	DfDisconnect(ip);

	save_frame(height, width, channels, depth_buf, brightness_buf, frame_path);

	/****************************保存点云*********************************/
	// 先转换得到正确的RGB坐标系需要原始深度图，RGB resolution，RGB的内参和旋转
	cv::Mat camera_intrinsic_temp(3, 3, CV_32F, calib_param.intrinsic);

	std::cout << "camera_intrinsic_temp: " << camera_intrinsic_temp << std::endl;

	/****************************保存点云*********************************/

	cv::Mat pointCloud(height, width, CV_32FC3, pointcloud_buf);

	std::string folder_path = frame_path;
	std::string pointcloud_path = folder_path + "_pointcloud.ply";
	std::string rgb_brightness_path = folder_path + "_color.bmp";
	std::string rgb_depth_path = folder_path + "_depth_of_rgb_camera.tiff";
	std::string depth2color_path = folder_path + "_depth2color1.tiff";

	SaveBinPointsToPly(pointCloud, pointcloud_path, brightnessTemp);

	delete[] depth_buf;
	delete[] brightness_buf;

	return 1;
}

int get_frame_04_hdr(const char* ip, const char* frame_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnect(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	int rgb_width, rgb_height;
	DfGetGrayCameraResolution(&width, &height);
	DfGetRGBCameraResolution(&rgb_width, &rgb_height);


	ret = DfGetCalibrationParam(calibration_param_);

	int image_size = width * height;
	int rgb_image_size = rgb_width * rgb_height;

	int depth_buf_size = image_size * 1 * 4;
	float* depth_buf = (float*)(new char[depth_buf_size]);

	int brightness_bug_size = image_size;
	unsigned char* brightness_buf = new unsigned char[brightness_bug_size];

	int color_brightness_buf_size = rgb_image_size * 3;
	unsigned char* color_brightness_buf = new unsigned char[color_brightness_buf_size];

	cv::Mat depthTemp(height, width, CV_32F, depth_buf);
	cv::Mat brightnessTemp(height, width, CV_8U, brightness_buf);
	cv::Mat colorBrightnessTemp(rgb_height, rgb_width, CV_8UC3, color_brightness_buf);
	cv::Mat colorBrightnessResult(rgb_height / 2, rgb_width / 2, CV_8UC3);
	cv::Mat RGBDepthTemp(colorBrightnessResult.size(), CV_32F, cv::Scalar(0));

	int num = 3;
	int led_param[5] = { 1023,1023,1023,1023,1023 };
	int exposure_param[5] = { 25000, 35000, 45000, 60000, 80000 };

	//设置多曝光参数
	ret = DfSetParamMixedHdr(num, exposure_param, led_param);

	ret = DfGetFrame04HDR(depth_buf, depth_buf_size, brightness_buf, brightness_bug_size, color_brightness_buf, color_brightness_buf_size, colorBrightnessResult.data, colorBrightnessResult.rows * colorBrightnessResult.cols * 3, (float*)RGBDepthTemp.data, RGBDepthTemp.rows * RGBDepthTemp.cols);

	DfDisconnect(ip);

	save_frame(height, width, depth_buf, brightness_buf, frame_path);

	/****************************保存点云*********************************/
	// 先转换得到正确的RGB坐标系需要原始深度图，RGB resolution，RGB的内参和旋转
	cv::Mat camera_intrinsic_temp(3, 3, CV_32F, calibration_param_.camera_intrinsic);
	cv::Mat rgb_camera_intrinsic_temp(3, 3, CV_32F, calibration_param_.rgb_camera_intrinsic);
	cv::Mat l2rgb_r_temp(3, 3, CV_32F, calibration_param_.rotation_matrix);
	cv::Mat l2rgb_t_temp(3, 1, CV_32F, calibration_param_.translation_matrix);

	// 然后对内参进行修正
	rgb_camera_intrinsic_temp.at<float>(0, 0) /= 2.;
	rgb_camera_intrinsic_temp.at<float>(1, 1) /= 2.;
	rgb_camera_intrinsic_temp.at<float>(0, 2) /= 2.;
	rgb_camera_intrinsic_temp.at<float>(1, 2) /= 2.;

	/****************************保存点云*********************************/

	cv::Mat pointCloud;

	depth_to_point_cloud_map_use_intrinsic(RGBDepthTemp, rgb_camera_intrinsic_temp, pointCloud);

	std::string folder_path = frame_path;
	std::string pointcloud_path = folder_path + "_pointcloud.ply";
	std::string rgb_brightness_path = folder_path + "_color.bmp";
	std::string rgb_depth_path = folder_path + "_depth_of_rgb_camera.tiff";
	std::string depth2color_path = folder_path + "_depth2color1.tiff";

	std::cout << "pointCloud size" << pointCloud.size() << std::endl;

	SaveBinPointsToPly(pointCloud, pointcloud_path, colorBrightnessResult);

	cv::imwrite(rgb_brightness_path, colorBrightnessResult);
	cv::imwrite(rgb_depth_path, RGBDepthTemp);

	delete[] depth_buf;
	delete[] brightness_buf;
	delete[] color_brightness_buf;

	return 1;
}

int get_frame_test(const char* ip, const char* frame_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetGrayCameraResolution(&width, &height);


	ret = DfGetCalibrationParam(calibration_param_);

	int image_size = width * height;

	int depth_buf_size = image_size * 1 * 4;
	float* depth_buf = (float*)(new char[depth_buf_size]);

	int brightness_buf_size = image_size;
	unsigned char* brightness_buf = new unsigned char[brightness_buf_size];

	int patterns_buf_size = image_size * 28;
	unsigned char* patterns_buf = new unsigned char[patterns_buf_size];

	ret = DfGetFrameTest(depth_buf, depth_buf_size, brightness_buf, brightness_buf_size, patterns_buf, patterns_buf_size);

	DfDisconnectNet();

	save_frame(height, width, depth_buf, brightness_buf, frame_path);

	save_images(frame_path, patterns_buf, width, height, patterns_buf_size / image_size);

	cv::Mat depthTemp(height, width, CV_32F, depth_buf);
	cv::Mat brightnessTemp(height, width, CV_8U, brightness_buf);

	//cv::Mat Q_mat(4, 4, CV_32F, Q_matrix);

	cv::Mat camera_intrinsic_temp(3, 3, CV_32F, calibration_param_.camera_intrinsic);

	//depth_to_xyz(depthTemp, brightnessTemp, Q_mat, frame_path);
	cv::Mat pointCloud;
	//depth_to_point_cloud_map(depthTemp, Q_mat, pointCloud);
	depth_to_point_cloud_map_use_intrinsic(depthTemp, camera_intrinsic_temp, pointCloud);

	std::string folder_path = frame_path;
	std::string pointcloud_path = folder_path + "_pointcloud.ply";
	SaveBinPointsToPly(pointCloud, pointcloud_path, brightnessTemp);

	delete[] depth_buf;
	delete[] brightness_buf;
	delete[] patterns_buf;



	return 1;
}

int percentile(cv::Mat& image, int percent)
{
	int size = image.rows * image.cols;
	float* data = new float[size];
	int count = 0;
	for (int i = 0; i < size; i++)
	{
		if (((float*)(image.data))[i] > 0)
		{
			data[count] = ((float*)(image.data))[i];
			count++;
		}
	}
	std::sort(data, data + count);
	int index = (count - 1) * percent / 100;
	int result = data[index];
	delete[] data;
	return result;
}

bool depthToDepthColor(cv::Mat depth_map, cv::Mat& color_map, cv::Mat& grey_map, float low_z, float high_z)
{

	if (!depth_map.data)
	{
		return false;
	}

	cv::Mat handle_map(depth_map.size(), CV_8U, cv::Scalar(0));
	cv::Mat mask_map(depth_map.size(), CV_8U, cv::Scalar(0));

	// 设置用于非线性显示的部分
	float out_range_rate = 26. / 255.;
	int gray_offset = 255 * out_range_rate / 2;
	float range = high_z - low_z;

	if (depth_map.type() != CV_32FC1)
	{
		depth_map.convertTo(depth_map, CV_32FC1);
	}


	int gray_temp;
	for (int r = 0; r < handle_map.rows; r++)
	{
		uchar* ptr_h = handle_map.ptr<uchar>(r);
		uchar* ptr_m = mask_map.ptr<uchar>(r);
		float* ptr_dr = depth_map.ptr<float>(r);

		for (int c = 0; c < handle_map.cols; c++)
		{
			if (low_z >= ptr_dr[c])
			{
				gray_temp = gray_offset - (255.0 - gray_offset * 2) * (low_z - ptr_dr[c]) / range;
				if (gray_temp < 0)
				{
					ptr_h[c] = 0;
				}
				else
				{
					ptr_h[c] = gray_temp;
				}
			}
			else if (ptr_dr[c] >= high_z)
			{
				gray_temp = 255 - gray_offset + (255.0 - gray_offset * 2) * (ptr_dr[c] - high_z) / range;
				if (gray_temp > 255)
				{
					ptr_h[c] = 255;
				}
				else
				{
					ptr_h[c] = gray_temp;
				}
			}
			else
			{
				ptr_h[c] = gray_offset + (255.0 - gray_offset * 2) * (ptr_dr[c] - low_z) / range;
			}

			if (ptr_dr[c] > 1)
			{
				ptr_m[c] = 255;
			}
		}
	}

	grey_map = handle_map.clone();

	cv::applyColorMap(handle_map, color_map, cv::COLORMAP_JET);

	maskZMap(color_map, mask_map);

	return true;
}


bool maskZMap(cv::Mat& z_map, cv::Mat mask)
{
	if (!z_map.data)
	{
		return false;
	}
	if (!mask.data)
	{
		return false;
	}

	if (3 == z_map.channels())
	{
		for (int r = 0; r < z_map.rows; r++)
		{
			uchar* ptr_m = mask.ptr<uchar>(r);
			cv::Vec3b* ptr_dr = z_map.ptr<cv::Vec3b>(r);

			for (int c = 0; c < z_map.cols; c++)
			{
				if (0 == ptr_m[c])
				{
					ptr_dr[c][0] = 0;
					ptr_dr[c][1] = 0;
					ptr_dr[c][2] = 0;
				}
			}
		}
	}

	if (1 == z_map.channels())
	{
		for (int r = 0; r < z_map.rows; r++)
		{
			uchar* ptr_m = mask.ptr<uchar>(r);
			uchar* ptr_dr = z_map.ptr<uchar>(r);

			for (int c = 0; c < z_map.cols; c++)
			{
				if (0 == ptr_m[c])
				{
					ptr_dr[c] = 0;
				}
			}
		}
	}



	return true;
}

int get_raw_01(const char* ip, const char* raw_image_dir)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetGrayCameraResolution(&width, &height);

	int capture_num = 36;

	int image_size = width * height;

	unsigned char* raw_buf = new unsigned char[image_size * capture_num];

	ret = DfGetCameraRawData01(raw_buf, image_size * capture_num);

	save_images(raw_image_dir, raw_buf, width, height, capture_num);

	delete[] raw_buf;

	DfDisconnectNet();
	return 1;
}

int get_raw_02(const char* ip, const char* raw_image_dir)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	DfGetGrayCameraResolution(&width, &height);

	int capture_num = 36;

	int image_size = width * height * sizeof(unsigned short);

	unsigned short* raw_buf = new unsigned short[(long)image_size * capture_num];

	ret = DfGetCameraRawData02(raw_buf, image_size * capture_num);

	save_images_16bit(raw_image_dir, raw_buf, width, height, capture_num);

	delete[] raw_buf;

	DfDisconnectNet();
	return 1;
}

int get_raw_03(const char* ip, const char* raw_image_dir)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	int width, height;
	int rgb_width, rgb_height;

	DfGetGrayCameraResolution(&width, &height);
	DfGetRGBCameraResolution(&rgb_width, &rgb_height);

	int capture_num = 36;

	int image_size = width * height;
	int rgb_image_size = rgb_width * rgb_height * 3;

	size_t buf_size = image_size * capture_num + rgb_image_size;

	unsigned char* raw_buf = new unsigned char[buf_size];

	ret = DfGetCameraRawData03(raw_buf, buf_size);

	save_images(raw_image_dir, raw_buf, width, height, capture_num);

	std::stringstream ss;
	ss << std::setw(2) << std::setfill('0') << capture_num;
	std::string folderPath = raw_image_dir;
	std::string fileName = folderPath + +"/phase" + ss.str() + ".bmp";
	cv::Mat rgb_image(rgb_height, rgb_width, CV_8UC3, raw_buf + (image_size * capture_num));
	//cv::imshow("rgb_image", rgb_image);
	//cv::waitKey(0);
	
	cv::imwrite(fileName, rgb_image);

	delete[] raw_buf;

	DfDisconnectNet();
	return 1;
}

int get_calib_param(const char* ip, const char* calib_param_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	struct CameraCalibParam calibration_param;
	DfGetCalibrationParam(calibration_param);
	std::ofstream ofile;
	ofile.open(calib_param_path);
	for (int i = 0; i < sizeof(calibration_param) / sizeof(float); i++)
	{
		ofile << ((float*)(&calibration_param))[i] << std::endl;
	}
	ofile.close();

	DfDisconnectNet();
	return 1;
}

int get_camera_exposure_param(const char* ip, float& exposure)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}


	DfGetParamCameraExposure(exposure);


	DfDisconnectNet();


	std::cout << "camera exposure: " << exposure << std::endl;
}

int set_camera_exposure_param(const char* ip, float exposure)
{
	if (exposure < 8 || exposure> 1000000)
	{
		std::cout << "exposure param out of range!" << std::endl;
		return 0;
	}


	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}


	DfSetParamCameraExposure(exposure);


	DfDisconnectNet();


	std::cout << "camera exposure: " << exposure << std::endl;

	return 1;
}

int set_hdr_param(const char* ip, const char* param_path)
{
	int brightness_list[5];
	int exposure_list[5];
	int hdr_count;

	read_hdr_param_from_file(param_path, hdr_count, exposure_list, brightness_list);

	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	DfSetParamHDR(hdr_count, exposure_list, brightness_list);

	DfDisconnectNet();

	return 1;
}

int set_camera_gain(const char* ip, float gain)
{
	if (gain < 0)
	{
		std::cout << "gain out of range!" << std::endl;
		return 0;
	}

	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}


	DfSetParamCameraGain(gain);


	DfDisconnectNet();


	std::cout << "camera gain: " << gain << std::endl;

	return 1;
}

int get_camera_gain(const char* ip, float& gain)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}


	DfGetParamCameraGain(gain);


	DfDisconnectNet();


	std::cout << "camera gain: " << gain << std::endl;
}

int get_generate_brightness_param(const char* ip, int& model, float& exposure)
{

	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	DfGetParamGenerateBrightness(model, exposure);

	DfDisconnectNet();


	std::cout << "model: " << model << std::endl;
	std::cout << "exposure: " << exposure << std::endl;
	return 1;
}

int set_generate_brightness_param(const char* ip, int model, float exposure)
{
	if (exposure < 8 || exposure> 1000000)
	{
		std::cout << "exposure param out of range!" << std::endl;
		return 0;
	}

	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	DfSetParamGenerateBrightness(model, exposure);


	DfDisconnectNet();

	std::cout << "model: " << model << std::endl;
	std::cout << "exposure: " << exposure << std::endl;
	return 1;
}

int set_calib_param(const char* ip, const char* calib_param_path)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}

	struct CameraCalibParam calibration_param;
	std::ifstream ifile;
	ifile.open(calib_param_path);
	for (int i = 0; i < sizeof(calibration_param) / sizeof(float); i++)
	{
		ifile >> ((float*)(&calibration_param))[i];
		std::cout << ((float*)(&calibration_param))[i] << std::endl;
	}
	ifile.close();

	DfSetCalibrationParam(calibration_param);

	DfDisconnectNet();
	return 1;
}

bool read_hdr_param_from_file(const char* param_path, int& hdr_count, int* exposure_time, int* brightness)
{
	std::string file_path(param_path);
	cv::FileStorage fs_in, fs_out;

	if (!fs_in.open(file_path, cv::FileStorage::READ))
	{
		fs_in.release();
		fs_out.open(file_path, cv::FileStorage::WRITE);

		cv::Mat HDR_exposure_list = (cv::Mat_<int>(5, 1) << 12000,
			12000,
			12000,
			12000,
			12000);

		cv::Mat HDR_brightness_list = (cv::Mat_<int>(5, 1) << 100,
			200,
			400,
			800,
			1023);

		fs_out << "hdr_count" << 1;
		fs_out << "hdr_exposure_list" << HDR_exposure_list;
		fs_out << "hdr_brightness_list" << HDR_brightness_list;
		fs_out.release();

		fs_in.open(file_path, cv::FileStorage::READ);
	}

	cv::Mat brightness_list(5, 1, CV_32S, brightness);
	cv::Mat exposure_list(5, 1, CV_32S, exposure_time);

	fs_in["hdr_count"] >> hdr_count;
	fs_in["hdr_exposure_list"] >> exposure_list;
	fs_in["hdr_brightness_list"] >> brightness_list;
	
	return true;
}

int set_camera_gamma(const char* ip, float gamma)
{
	if (gamma < 0.1 || gamma > 2)
	{
		std::cout << "gain out of range!" << std::endl;
		return 0;
	}

	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}


	DfSetParamCameraGamma(gamma);


	DfDisconnectNet();


	std::cout << "camera gamma: " << gamma << std::endl;

	return 1;
}

int get_camera_gamma(const char* ip, float& gamma)
{
	DfRegisterOnDropped(on_dropped);

	int ret = DfConnectNet(ip);
	if (ret == DF_FAILED)
	{
		return 0;
	}


	DfGetParamCameraGamma(gamma);


	DfDisconnectNet();


	std::cout << "camera gamma: " << gamma << std::endl;
}

