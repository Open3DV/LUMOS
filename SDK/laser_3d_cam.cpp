#define _CRT_SECURE_NO_WARNINGS

#include "laser_3d_cam_dev.h"
#include "socket_tcp.h"
#include <assert.h>
#include <iostream>
#include <thread>
#include "utils.h"
#include "../firmware/easylogging++.h"
#include<chrono>
#include<ctime>
#include <time.h>
#include <stddef.h> 
#include "../test/triangulation.h"
#include "../firmware/protocol.h" 
#include "../firmware/system_config_settings.h"
//#include <configuring_network.h>

#ifdef _WIN32 

#elif __linux 
#include <dirent.h> 
#include <unistd.h>
#endif 


using namespace std;
using namespace std::chrono;

//socket
INITIALIZE_EASYLOGGINGPP

//const int image_width = 1920;
//const int image_height = 1200;
//const int image_size = image_width * image_height;
bool connected = false;
long long token = 0;
//const char* camera_id_;
std::string camera_id_;
std::thread heartbeat_thread;
int heartbeat_error_count_ = 0;

extern SOCKET g_sock_heartbeat;
extern SOCKET g_sock;

int (*p_OnDropped)(void*) = 0;

//int camera_version = 0;

int multiple_exposure_model_ = 1;
int repetition_exposure_model_ = 2;


std::timed_mutex command_mutex_;
/**************************************************************************************************************/


struct CameraCalibParam calibration_param_;
bool connected_flag_ = false;

int camera_width_ = 1920;
int camera_height_ = 1200;
int image_size_ = camera_width_ * camera_height_;

const char* camera_ip_ = "";


int depth_buf_size_ = 0;
int pointcloud_buf_size_ = 0;
int brightness_bug_size_ = 0;
float* point_cloud_buf_ = NULL;
float* trans_point_cloud_buf_ = NULL;
bool transform_pointcloud_flag_ = false;
float* depth_buf_ = NULL;
unsigned char* brightness_buf_ = NULL;
float* undistort_map_x_ = NULL;
float* undistort_map_y_ = NULL;


/**************************************************************************************************************/

std::time_t getTimeStamp(long long& msec)
{
	std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
	auto tmp = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch());
	seconds sec = duration_cast<seconds>(tp.time_since_epoch());


	std::time_t timestamp = tmp.count();

	msec = tmp.count() - sec.count() * 1000;
	//std::time_t timestamp = std::chrono::system_clock::to_time_t(tp);
	return timestamp;
}

std::tm* gettm(long long timestamp)
{
	auto milli = timestamp + (long long)8 * 60 * 60 * 1000; //此处转化为东八区北京时间，如果是其它时区需要按需求修改
	auto mTime = std::chrono::milliseconds(milli);
	auto tp = std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>(mTime);
	auto tt = std::chrono::system_clock::to_time_t(tp);
	std::tm* now = std::gmtime(&tt);
	//printf("%4d年%02d月%02d日 %02d:%02d:%02d\n", now->tm_year + 1900, now->tm_mon + 1, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);
	return now;
}


std::string get_timestamp()
{

	long long msec = 0;
	char time_str[7][16];
	auto t = getTimeStamp(msec);
	//std::cout << "Millisecond timestamp is: " << t << std::endl;
	auto time_ptr = gettm(t);
	sprintf(time_str[0], "%02d", time_ptr->tm_year + 1900); //月份要加1
	sprintf(time_str[1], "%02d", time_ptr->tm_mon + 1); //月份要加1
	sprintf(time_str[2], "%02d", time_ptr->tm_mday);//天
	sprintf(time_str[3], "%02d", time_ptr->tm_hour);//时
	sprintf(time_str[4], "%02d", time_ptr->tm_min);// 分
	sprintf(time_str[5], "%02d", time_ptr->tm_sec);//时
	sprintf(time_str[6], "%02lld", msec);// 分
	//for (int i = 0; i < 7; i++)
	//{
	//	std::cout << "time_str[" << i << "] is: " << time_str[i] << std::endl;
	//}

	std::string timestamp = "";

	timestamp += time_str[0];
	timestamp += "-";
	timestamp += time_str[1];
	timestamp += "-";
	timestamp += time_str[2];
	timestamp += " ";
	timestamp += time_str[3];
	timestamp += ":";
	timestamp += time_str[4];
	timestamp += ":";
	timestamp += time_str[5];
	timestamp += ",";
	timestamp += time_str[6];

	//std::cout << timestamp << std::endl;

	return timestamp;
}


/**************************************************************************************************************/


int on_dropped(void* param)
{
	LOG(INFO) << "Network dropped!" << std::endl;
	return 0;
}

int depthTransformPointcloud(float* depth_map, float* point_cloud_map)
{

	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}

	float camera_fx = calibration_param_.camera_intrinsic[0];
	float camera_fy = calibration_param_.camera_intrinsic[4];

	float camera_cx = calibration_param_.camera_intrinsic[2];
	float camera_cy = calibration_param_.camera_intrinsic[5];


	float k1 = calibration_param_.camera_distortion[0];
	float k2 = calibration_param_.camera_distortion[1];
	float p1 = calibration_param_.camera_distortion[2];
	float p2 = calibration_param_.camera_distortion[3];
	float k3 = calibration_param_.camera_distortion[4];


	int nr = camera_height_;
	int nc = camera_width_;

#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{

		for (int c = 0; c < nc; c++)
		{



			int offset = r * camera_width_ + c;
			if (depth_map[offset] > 0)
			{
				//double undistort_x = c;
				//double undistort_y = r;
				//undistortPoint(c, r, camera_fx, camera_fy,
				//	camera_cx, camera_cy, k1, k2, k3, p1, p2, undistort_x, undistort_y);

				float undistort_x = undistort_map_x_[offset];
				float undistort_y = undistort_map_y_[offset];

				point_cloud_map[3 * offset + 0] = (undistort_x - camera_cx) * depth_map[offset] / camera_fx;
				point_cloud_map[3 * offset + 1] = (undistort_y - camera_cy) * depth_map[offset] / camera_fy;
				point_cloud_map[3 * offset + 2] = depth_map[offset];


			}
			else
			{
				point_cloud_map[3 * offset + 0] = 0;
				point_cloud_map[3 * offset + 1] = 0;
				point_cloud_map[3 * offset + 2] = 0;
			}


		}

	}


	return DF_SUCCESS;
}

void rolloutHandler(const char* filename, std::size_t size)
{
#ifdef _WIN32 
	/// 备份日志
	system("mkdir lumosLog");
	system("DIR .\\lumosLog\\ .log / B > LIST.TXT");
	ifstream name_in("LIST.txt", ios_base::in);//文件流

	int num = 0;
	std::vector<std::string> name_list;
	char buf[1024] = { 0 };
	while (name_in.getline(buf, sizeof(buf)))
	{
		//std::cout << "name: " << buf << std::endl;
		num++;
		name_list.push_back(std::string(buf));
	}

	if (num < 5)
	{
		num++;
	}
	else
	{
		num = 5;
		name_list.pop_back();
	}


	for (int i = num; i > 0 && !name_list.empty(); i--)
	{
		std::stringstream ss;
		std::string path = ".\\lumosLog\\" + name_list.back();
		name_list.pop_back();
		ss << "move " << path << " lumosLog\\log_" << i - 1 << ".log";
		std::cout << ss.str() << std::endl;
		system(ss.str().c_str());
	}

	std::stringstream ss;
	ss << "move " << filename << " lumosLog\\log_0" << ".log";
	system(ss.str().c_str());
#elif __linux 

	/// 备份日志
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
	if (num < 5)
	{
		num++;
	}
	else
	{
		num = 5;
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

#endif 

}

//函数名： DfConnect
//功能： 连接相机
//输入参数： camera_id（相机id）
//输出参数： 无
//返回值： 类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfConnect(const char* camera_id)
{

	LOG(INFO) << "DfConnect: ";
	/*******************************************************************************************************************/
	//关闭log输出
	el::Configurations conf;
	conf.setToDefault();
	//conf.setGlobally(el::ConfigurationType::Format, "[%datetime{%H:%m:%s} | %level] %msg");

#ifdef _WIN32 
	//conf.setGlobally(el::ConfigurationType::Filename, "log\\log_%datetime{%Y%M%d}.log");
	conf.setGlobally(el::ConfigurationType::Filename, "lumos_log.log");
#elif __linux 
	//conf.setGlobally(el::ConfigurationType::Filename, "log/log_%datetime{%Y%M%d}.log");
	conf.setGlobally(el::ConfigurationType::Filename, "lumos_log.log");
#endif 
	conf.setGlobally(el::ConfigurationType::Enabled, "true");
	conf.setGlobally(el::ConfigurationType::ToFile, "true");
	//conf.setGlobally(el::ConfigurationType::MaxLogFileSize, "204800");//1024*1024*1024=1073741824 
	el::Loggers::reconfigureAllLoggers(conf);
	el::Loggers::reconfigureAllLoggers(el::ConfigurationType::ToStandardOutput, "false");

	/*******************************************************************************************************************/

	DfRegisterOnDropped(on_dropped);


	int ret = DfConnectNet(camera_id);
	if (ret != DF_SUCCESS)
	{
		return ret;
	}

	ret = DfGetCalibrationParam(calibration_param_);

	if (ret != DF_SUCCESS)
	{
		DfDisconnectNet();
		return ret;
	}

	int width, height;
	ret = DfGetCameraResolution(&width, &height);
	if (ret != DF_SUCCESS)
	{
		DfDisconnectNet();
		return ret;
	}

	if (width <= 0 || height <= 0)
	{
		DfDisconnectNet();
		return DF_ERROR_2D_CAMERA;
	}

	camera_width_ = width;
	camera_height_ = height;

	camera_ip_ = camera_id;
	connected_flag_ = true;

	image_size_ = camera_width_ * camera_height_;

	depth_buf_size_ = image_size_ * 1 * 4;
	depth_buf_ = (float*)(new char[depth_buf_size_]);

	pointcloud_buf_size_ = depth_buf_size_ * 3;
	point_cloud_buf_ = (float*)(new char[pointcloud_buf_size_]);

	trans_point_cloud_buf_ = (float*)(new char[pointcloud_buf_size_]);

	brightness_bug_size_ = image_size_;
	brightness_buf_ = new unsigned char[brightness_bug_size_];


	/******************************************************************************************************/
	//产生畸变校正表
	undistort_map_x_ = (float*)(new char[depth_buf_size_]);
	undistort_map_y_ = (float*)(new char[depth_buf_size_]);


	float camera_fx = calibration_param_.camera_intrinsic[0];
	float camera_fy = calibration_param_.camera_intrinsic[4];

	float camera_cx = calibration_param_.camera_intrinsic[2];
	float camera_cy = calibration_param_.camera_intrinsic[5];


	float k1 = calibration_param_.camera_distortion[0];
	float k2 = calibration_param_.camera_distortion[1];
	float p1 = calibration_param_.camera_distortion[2];
	float p2 = calibration_param_.camera_distortion[3];
	float k3 = calibration_param_.camera_distortion[4];


	int nr = camera_height_;
	int nc = camera_width_;

#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{

		for (int c = 0; c < nc; c++)
		{
			double undistort_x = c;
			double undistort_y = r;


			int offset = r * camera_width_ + c;

			undistortPoint(c, r, camera_fx, camera_fy,
				camera_cx, camera_cy, k1, k2, k3, p1, p2, undistort_x, undistort_y);

			undistort_map_x_[offset] = (float)undistort_x;
			undistort_map_y_[offset] = (float)undistort_y;
		}

	}
	/*****************************************************************************************************************/

	el::Loggers::addFlag(el::LoggingFlag::StrictLogFileSizeCheck);
	el::Loggers::reconfigureAllLoggers(el::ConfigurationType::MaxLogFileSize, "104857600");//100MB 104857600

	/// 注册回调函数
	el::Helpers::installPreRollOutCallback(rolloutHandler);


	/********************************************************************************************************/



	return 0;
}

//函数名： DfGetCameraResolution
//功能： 获取相机分辨率
//输入参数： 无
//输出参数： width(图像宽)、height(图像高)
//返回值： 类型（int）:返回0表示获取参数成功;返回-1表示获取参数失败.
DF_SDK_API int DfGetCameraResolution(int* width, int* height)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetCameraResolution:";
	*width = camera_width_;
	*height = camera_height_;

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_CAMERA_RESOLUTION, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = recv_buffer((char*)(width), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = recv_buffer((char*)(height), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{

		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	image_size_ = (*width) * (*height);

	LOG(INFO) << "width: " << *width;
	LOG(INFO) << "height: " << *height;
	return DF_SUCCESS;

}

//函数名： DfConnect
//功能： 断开相机连接
//输入参数： camera_id（相机id）
//输出参数： 无
//返回值： 类型（int）:返回0表示断开成功;返回-1表示断开失败.
DF_SDK_API int DfDisconnect(const char* camera_id)
{

	LOG(INFO) << "DfDisconnect:";
	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}


	int ret = DfDisconnectNet();

	if (DF_FAILED == ret)
	{
		return DF_FAILED;
	}

	delete[] depth_buf_;
	delete[] brightness_buf_;
	delete[] point_cloud_buf_;
	delete[] trans_point_cloud_buf_;
	delete[] undistort_map_x_;
	delete[] undistort_map_y_;


	connected_flag_ = false;

	/// 注销回调函数
	el::Helpers::uninstallPreRollOutCallback();

	return DF_SUCCESS;
}

//函数名： DfGetCalibrationParam
//功能： 获取相机标定参数
//输入参数： 无
//输出参数： calibration_param（相机标定参数结构体）
//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
DF_SDK_API int DfGetCalibrationParam(struct CalibrationParam* calibration_param)
{
	LOG(INFO) << "DfGetCalibrationParam:";
	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}

	//calibration_param = &calibration_param_;

	for (int i = 0; i < 9; i++)
	{
		calibration_param->intrinsic[i] = calibration_param_.camera_intrinsic[i];
	}

	for (int i = 0; i < 5; i++)
	{
		calibration_param->distortion[i] = calibration_param_.camera_distortion[i];
	}

	for (int i = 5; i < 12; i++)
	{
		calibration_param->distortion[i] = 0;
	}

	float extrinsic[4 * 4] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

	for (int i = 0; i < 16; i++)
	{
		calibration_param->extrinsic[i] = extrinsic[i];
	}


	return 0;
}


/**************************************************************************************************************/


int HeartBeat()
{
	//std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	//while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	//{
	//	LOG(INFO) << "--";
	//}

	LOG(TRACE) << "heart beat: ";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock_heartbeat);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock_heartbeat);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_HEARTBEAT, g_sock_heartbeat);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to send disconnection cmd";
		close_socket(g_sock_heartbeat);
		return DF_FAILED;
	}
	ret = send_buffer((char*)&token, sizeof(token), g_sock_heartbeat);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to send token";
		close_socket(g_sock_heartbeat);
		return DF_FAILED;
	}

	int command;
	ret = recv_command(&command, g_sock_heartbeat);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock_heartbeat);
		return DF_FAILED;
	}
	if (command == DF_CMD_OK)
	{
		ret = DF_SUCCESS;
	}
	else if (command == DF_CMD_REJECT)
	{
		ret = DF_BUSY;
	}
	else
	{
		LOG(ERROR) << "Failed recv heart beat command";
		assert(0);
	}
	close_socket(g_sock_heartbeat);
	return ret;
}

int HeartBeat_loop()
{
	heartbeat_error_count_ = 0;
	while (connected)
	{
		int ret = HeartBeat();
		if (ret == DF_FAILED)
		{
			heartbeat_error_count_++;
			LOG(ERROR) << "heartbeat error count: " << heartbeat_error_count_;

			if (heartbeat_error_count_ > 2)
			{
				LOG(ERROR) << "close connect";
				connected = false;
				//close_socket(g_sock); 
				p_OnDropped(0);
			}

		}
		else if (DF_SUCCESS == ret)
		{
			heartbeat_error_count_ = 0;
		}


		for (int i = 0; i < 100; i++)
		{
			if (!connected)
			{
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
	return 0;
}

DF_SDK_API int DfConnectNet(const char* ip)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfConnectNet: ";
	/*******************************************************************************************************************/
	//关闭log输出
	//el::Configurations conf;
	//conf.setToDefault();
	//conf.setGlobally(el::ConfigurationType::Format, "[%datetime{%H:%m:%s} | %level] %msg");
	//conf.setGlobally(el::ConfigurationType::Filename, "log\\log_%datetime{%Y%M%d}.log");
	//conf.setGlobally(el::ConfigurationType::Enabled, "true");
	//conf.setGlobally(el::ConfigurationType::ToFile, "true");
	//el::Loggers::reconfigureAllLoggers(conf);
	//el::Loggers::reconfigureAllLoggers(el::ConfigurationType::ToStandardOutput, "false");


	//DfRegisterOnDropped(on_dropped);
	/*******************************************************************************************************************/


	camera_id_ = ip;
	LOG(INFO) << "start connection: " << ip;
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}


	LOG(INFO) << "sending connection cmd";
	ret = send_command(DF_CMD_CONNECT, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(INFO) << "Failed to send connection cmd";
		close_socket(g_sock);
		return DF_FAILED;
	}
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}
	if (ret == DF_SUCCESS)
	{
		if (command == DF_CMD_OK)
		{
			LOG(INFO) << "Recieved connection ok";
			ret = recv_buffer((char*)&token, sizeof(token), g_sock);
			if (ret == DF_SUCCESS)
			{
				connected = true;
				LOG(INFO) << "token: " << token;
				close_socket(g_sock);
				if (heartbeat_thread.joinable())
				{
					heartbeat_thread.join();
				}
				heartbeat_thread = std::thread(HeartBeat_loop);
				return DF_SUCCESS;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			LOG(INFO) << "connection rejected";
			close_socket(g_sock);
			return DF_BUSY;
		}
	}
	else
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	return DF_FAILED;
}

DF_SDK_API int DfDisconnectNet()
{

	LOG(INFO) << "token " << token << " try to disconnection";
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";

	}


	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(INFO) << "Failed to setup_socket";
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_DISCONNECT, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(INFO) << "Failed to send disconnection cmd";
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	if (ret == DF_FAILED)
	{
		LOG(INFO) << "Failed to send token";
		close_socket(g_sock);
		return DF_FAILED;
	}
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}
	connected = false;
	token = 0;

	if (heartbeat_thread.joinable())
	{
		heartbeat_thread.join();
	}

	LOG(INFO) << "Camera disconnected";
	return close_socket(g_sock);
}

DF_SDK_API int DfGetFrame01(float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}
	LOG(INFO) << "GetFrame01";
	assert(depth_buf_size == image_size_ * sizeof(float) * 1);
	assert(brightness_buf_size == image_size_ * sizeof(char) * 1);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}


	ret = send_command(DF_CMD_GET_FRAME_01, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		LOG(INFO) << "brightness received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		//brightness = (unsigned char*)depth + depth_buf_size;
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}

	LOG(INFO) << "Get frame01 success";
	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfGetFrame01HDR(float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}
	LOG(INFO) << "GetFrame01";
	assert(depth_buf_size == image_size_ * sizeof(float) * 1);
	assert(brightness_buf_size == image_size_ * sizeof(char) * 1);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}


	ret = send_command(DF_CMD_GET_FRAME_01_HDR, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		LOG(INFO) << "brightness received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		//brightness = (unsigned char*)depth + depth_buf_size;
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}

	LOG(INFO) << "Get frame01 success";
	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfGetFrameTest(float* depth, int depth_buf_size,
	unsigned char* brightness, int brightness_buf_size, unsigned char* patterns, int patterns_buf_size)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}
	LOG(INFO) << "GetFrameTest";
	LOG(INFO) << "image_size_ * sizeof(float) * 1： " << image_size_ * sizeof(float) * 1;
	LOG(INFO) << "depth_buf_size： " << depth_buf_size;

	assert(depth_buf_size == image_size_ * sizeof(float) * 1);
	assert(brightness_buf_size == image_size_ * sizeof(char) * 1);
	assert(patterns_buf_size == image_size_ * sizeof(char) * 28);
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}


	ret = send_command(DF_CMD_GET_FRAME_TEST, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, depth_buf_size=" << depth_buf_size;
		ret = recv_buffer((char*)depth, depth_buf_size, g_sock);
		LOG(INFO) << "depth received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, brightness_buf_size=" << brightness_buf_size;
		ret = recv_buffer((char*)brightness, brightness_buf_size, g_sock);
		LOG(INFO) << "brightness received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		LOG(INFO) << "receiving buffer, patterns_buf_size=" << patterns_buf_size;
		ret = recv_buffer((char*)patterns, patterns_buf_size, g_sock);
		LOG(INFO) << "patterns received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}

	LOG(INFO) << "Get frame test success";
	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfGetCameraRawData01(unsigned char* raw, int raw_buf_size)
{
	int img_num = 28;
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	if (raw)
	{
		LOG(INFO) << "GetRaw01";
		assert(raw_buf_size >= image_size_ * sizeof(unsigned char) * img_num);
		int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_RAW_01, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			LOG(INFO) << "token checked ok";
			LOG(INFO) << "receiving buffer, raw_buf_size=" << raw_buf_size;
			ret = recv_buffer((char*)raw, raw_buf_size, g_sock);
			LOG(INFO) << "images received";
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			LOG(INFO) << "Get raw rejected";
			close_socket(g_sock);
			return DF_BUSY;
		}

		LOG(INFO) << "Get raw success";
		close_socket(g_sock);
		return DF_SUCCESS;
	}
	return DF_FAILED;
}

DF_SDK_API int DfGetCameraRawData02(unsigned short* raw, int raw_buf_size)
{
	int img_num = 28;
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	if (raw)
	{
		LOG(INFO) << "GetRaw02";
		assert(raw_buf_size >= image_size_ * sizeof(unsigned char) * img_num);
		int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_command(DF_CMD_GET_RAW_02, g_sock);
		ret = send_buffer((char*)&token, sizeof(token), g_sock);
		int command;
		ret = recv_command(&command, g_sock);
		if (ret == DF_FAILED)
		{
			LOG(ERROR) << "Failed to recv command";
			close_socket(g_sock);
			return DF_FAILED;
		}

		if (command == DF_CMD_OK)
		{
			LOG(INFO) << "token checked ok";
			LOG(INFO) << "receiving buffer, raw_buf_size=" << raw_buf_size;
			ret = recv_buffer((char*)raw, raw_buf_size, g_sock);
			LOG(INFO) << "images received";
			if (ret == DF_FAILED)
			{
				close_socket(g_sock);
				return DF_FAILED;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			LOG(INFO) << "Get raw rejected";
			close_socket(g_sock);
			return DF_BUSY;
		}

		LOG(INFO) << "Get raw success";
		close_socket(g_sock);
		return DF_SUCCESS;
	}
	return DF_FAILED;
}

DF_SDK_API int DfGetSystemConfigParam(struct SystemConfigParam& config_param)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_SYSTEM_CONFIG_PARAMETERS, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&config_param), sizeof(config_param), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfSetSystemConfigParam(const struct SystemConfigParam& config_param)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_SYSTEM_CONFIG_PARAMETERS, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&config_param), sizeof(config_param), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfGetCalibrationParam(struct CameraCalibParam& calibration_param)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_CAMERA_PARAMETERS, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&calibration_param), sizeof(calibration_param), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfSetCalibrationParam(const struct CameraCalibParam& calibration_param)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_CAMERA_PARAMETERS, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&calibration_param), sizeof(calibration_param), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfRegisterOnDropped(int (*p_function)(void*))
{
	p_OnDropped = p_function;
	return 0;
}

DF_SDK_API int DfSetParamCameraGain(float gain)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamCameraGain:";

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_CAMERA_GAIN, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = send_buffer((char*)(&gain), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfGetParamCameraGain(float& gain)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}
	LOG(INFO) << "DfGetParamCameraGain:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_CAMERA_GAIN, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = recv_buffer((char*)(&gain), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfSetParamCameraGamma(float gamma)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamCameraGamma:";

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_CAMERA_GAMMA, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = send_buffer((char*)(&gamma), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfGetParamCameraGamma(float& gamma)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}
	LOG(INFO) << "DfGetParamCameraGamma:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_CAMERA_GAMMA, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = recv_buffer((char*)(&gamma), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfSetParamHDR(int hdr_count, int* exposure_list, int* brightness_list)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamHDR:";

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_HDR, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = send_buffer((char*)(&hdr_count), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_buffer((char*)exposure_list, 5 * sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
		ret = send_buffer((char*)brightness_list, 5 * sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamCameraExposure
//功能： 设置相机曝光时间
//输入参数：exposure(相机曝光时间)
//输出参数： 无
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfSetParamCameraExposure(float exposure)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamCameraExposure:";

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_CAMERA_EXPOSURE_TIME, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = send_buffer((char*)(&exposure), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetParamCameraExposure
//功能： 获取相机曝光时间
//输入参数： 无
//输出参数：exposure(相机曝光时间)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetParamCameraExposure(float& exposure)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetParamCameraExposure:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_CAMERA_EXPOSURE_TIME, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = recv_buffer((char*)(&exposure), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamGenerateBrightness
//功能： 设置生成亮度图参数
//输入参数：model(1:与条纹图同步连续曝光、2：单独发光曝光、3：不发光单独曝光)、exposure(亮度图曝光时间)
//输出参数： 无
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfSetParamGenerateBrightness(int model, float exposure)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamGenerateBrightness: ";
	if (exposure < 20 || exposure> 1000000)
	{
		std::cout << "exposure param out of range!" << std::endl;
		return DF_FAILED;
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_GENERATE_BRIGHTNESS, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&model), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = send_buffer((char*)(&exposure), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetParamGenerateBrightness
//功能： 获取生成亮度图参数
//输入参数： 无
//输出参数：model(1:与条纹图同步连续曝光、2：单独发光曝光、3：不发光单独曝光)、exposure(亮度图曝光时间)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetParamGenerateBrightness(int& model, float& exposure)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetParamGenerateBrightness: ";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_GENERATE_BRIGHTNESS, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = recv_buffer((char*)(&model), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = recv_buffer((char*)(&exposure), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

bool transformPointcloud(float* org_point_cloud_map, float* transform_point_cloud_map, float* rotate, float* translation)
{


	int point_num = camera_height_ * camera_width_;

	int nr = camera_height_;
	int nc = camera_width_;

#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{

		for (int c = 0; c < nc; c++)
		{

			int offset = r * camera_width_ + c;

			float x = org_point_cloud_map[3 * offset + 0];
			float y = org_point_cloud_map[3 * offset + 1];
			float z = org_point_cloud_map[3 * offset + 2];

			//if (z > 0)
			//{
			transform_point_cloud_map[3 * offset + 0] = rotate[0] * x + rotate[1] * y + rotate[2] * z + translation[0];
			transform_point_cloud_map[3 * offset + 1] = rotate[3] * x + rotate[4] * y + rotate[5] * z + translation[1];
			transform_point_cloud_map[3 * offset + 2] = rotate[6] * x + rotate[7] * y + rotate[8] * z + translation[2];

			//}
			//else
			//{
			   // point_cloud_map[3 * offset + 0] = 0;
			   // point_cloud_map[3 * offset + 1] = 0;
			   // point_cloud_map[3 * offset + 2] = 0;
			//}


		}

	}


	return true;
}

DF_SDK_API int DfGetDeviceTemperature(float& temperature)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_TEMPERATURE, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&temperature), sizeof(temperature), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}


		LOG(ERROR) << "CPU temperature: " << temperature;
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamSmoothing
//功能： 设置点云平滑参数
//输入参数：smoothing(0:关、1：小、2：中、3：大)
//输出参数： 无
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfSetParamSmoothing(int smoothing)
{

	LOG(INFO) << "DfSetParamSmoothing:";
	int ret = -1;
	if (0 == smoothing)
	{
		ret = DfSetParamBilateralFilter(0, 5);
	}
	else
	{
		ret = DfSetParamBilateralFilter(1, 2 * smoothing + 1);
	}

	return ret;
}

//函数名： DfGetParamSmoothing
//功能： 设置点云平滑参数
//输入参数：无
//输出参数：smoothing(0:关、1：小、2：中、3：大)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetParamSmoothing(int& smoothing)
{

	LOG(INFO) << "DfGetParamSmoothing:";
	int use = 0;
	int d = 0;

	int ret = DfGetParamBilateralFilter(use, d);

	if (DF_FAILED == ret)
	{
		return DF_FAILED;
	}

	if (0 == use)
	{
		smoothing = 0;
	}
	else if (1 == use)
	{
		smoothing = d / 2;
	}
	return DF_SUCCESS;
}

//函数名： DfSetParamBilateralFilter
//功能： 设置双边滤波参数
//输入参数： use（开关：1为开、0为关）、param_d（平滑系数：3、5、7、9、11）
//输出参数： 无
//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
DF_SDK_API int DfSetParamBilateralFilter(int use, int param_d)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	if (use != 1 && use != 0)
	{
		std::cout << "use param should be 1 or 0:  " << use << std::endl;
		return DF_FAILED;
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_BILATERAL_FILTER, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = send_buffer((char*)(&use), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = send_buffer((char*)(&param_d), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetParamBilateralFilter
//功能： 获取混合多曝光参数（最大曝光次数为6次）
//输入参数： 无
//输出参数： use（开关：1为开、0为关）、param_d（平滑系数：3、5、7、9、11）
//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
DF_SDK_API int DfGetParamBilateralFilter(int& use, int& param_d)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_BILATERAL_FILTER, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = recv_buffer((char*)(&use), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = recv_buffer((char*)(&param_d), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamDepthFilter
//功能： 设置深度图滤波参数
//输入参数：use(开关：1开、0关)、depth_filterthreshold(深度图在1000mm距离过滤的噪声阈值)
//输出参数： 无
//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
DF_SDK_API int DfSetParamDepthFilter(int use, float depth_filter_threshold)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamDepthFilter:";
	if (use != 1 && use != 0)
	{
		std::cout << "use param should be 1 or 0:  " << use << std::endl;
		return DF_FAILED;
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_DEPTH_FILTER, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = send_buffer((char*)(&use), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = send_buffer((char*)(&depth_filter_threshold), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetParamDepthFilter
//功能： 设置深度图滤波参数
//输入参数：use(开关：1开、0关)、depth_filterthreshold(深度图在1000mm距离过滤的噪声阈值)
//输出参数： 无
//返回值： 类型（int）:返回0表示获取参数成功;否则失败。
DF_SDK_API int DfGetParamDepthFilter(int& use, float& depth_filter_threshold)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetParamDepthFilter:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_DEPTH_FILTER, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = recv_buffer((char*)(&use), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = recv_buffer((char*)(&depth_filter_threshold), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamCameraConfidence
//功能： 设置相机曝光时间
//输入参数：confidence(相机置信度)
//输出参数： 无
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfSetParamCameraConfidence(float confidence)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamCameraConfidence:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_CAMERA_CONFIDENCE, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&confidence), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetParamCameraConfidence
//功能： 获取相机曝光时间
//输入参数： 无
//输出参数：confidence(相机置信度)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetParamCameraConfidence(float& confidence)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetParamCameraConfidence:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_CAMERA_CONFIDENCE, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = recv_buffer((char*)(&confidence), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamLedCurrent
//功能： 设置LED电流
//输入参数： led（电流值）
//输出参数： 无
//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
DF_SDK_API int DfSetParamLedCurrent(int led)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamLedCurrent: " << led;
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_LED_CURRENT, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&led), sizeof(led), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}


//函数名： DfGetParamLedCurrent
//功能： 设置LED电流
//输入参数： 无
//输出参数： led（电流值）
//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
DF_SDK_API int DfGetParamLedCurrent(int& led)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetParamLedCurrent: ";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_LED_CURRENT, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&led), sizeof(led), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	LOG(INFO) << "led: " << led;
	return DF_SUCCESS;
}

//函数名： DfSetParamMixedHdr
//功能： 设置混合多曝光参数（最大曝光次数为5次）
//输入参数： num（曝光次数）、exposure_param[5]（5个曝光参数、前num个有效）、led_param[5]（5个led亮度参数、前num个有效）
//输出参数： 无
//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
DF_SDK_API int DfSetParamMixedHdr(int num, int exposure_param[5], int led_param[5])
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamMixedHdr:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_MIXED_HDR, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		int param[13];
		param[0] = num;

		memcpy(param + 1, exposure_param, sizeof(int) * 5);
		memcpy(param + 7, led_param, sizeof(int) * 5);

		ret = send_buffer((char*)(param), sizeof(int) * 13, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetParamMixedHdr
//功能： 获取混合多曝光参数（最大曝光次数为5次）
//输入参数： 无
//输出参数： num（曝光次数）、exposure_param[5]（5个曝光参数、前num个有效）、led_param[5]（5个led亮度参数、前num个有效）
//返回值： 类型（int）:返回0表示获取标定参数成功;返回-1表示获取标定参数失败.
DF_SDK_API int DfGetParamMixedHdr(int& num, int exposure_param[5], int led_param[5])
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetParamMixedHdr:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_MIXED_HDR, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		int param[13];

		ret = recv_buffer((char*)(param), sizeof(int) * 13, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}


		memcpy(exposure_param, param + 1, sizeof(int) * 5);
		memcpy(led_param, param + 7, sizeof(int) * 5);
		num = param[0];

	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamStandardPlaneExternal
//功能： 设置基准平面的外参
//输入参数：R(旋转矩阵：3*3)、T(平移矩阵：3*1)
//输出参数： 无
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfSetParamStandardPlaneExternal(float* R, float* T)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamStandardPlaneExternal: ";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		float plane_param[12];

		memcpy(plane_param, R, 9 * sizeof(float));
		memcpy(plane_param + 9, T, 3 * sizeof(float));

		ret = send_buffer((char*)(plane_param), sizeof(float) * 12, g_sock);


		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;


}

//函数名： DfGetParamStandardPlaneExternal
//功能： 获取基准平面的外参
//输入参数：无
//输出参数： R(旋转矩阵：3*3)、T(平移矩阵：3*1)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetParamStandardPlaneExternal(float* R, float* T)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetParamStandardPlaneExternal: ";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_STANDARD_PLANE_EXTERNAL_PARAM, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		//int param_buf_size = 12 * sizeof(float);
		//float* plane_param = new float[param_buf_size];
		float plane_param[12];

		ret = recv_buffer((char*)(plane_param), sizeof(float) * 12, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		memcpy(R, plane_param, 9 * sizeof(float));
		memcpy(T, plane_param + 9, 3 * sizeof(float));

	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名：  DfGetProjectVersion
//功能：    获取相机型号
//输入参数：无
//输出参数：型号（3010、4710）
//返回值：  类型（int）:返回0表示连接成功;返回-1表示连接失败.
DF_SDK_API int DfGetProjectorVersion(int& version)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_PROJECTOR_VERSION, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&version), sizeof(version), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamRadiusFilter
//功能： 设置点云半径滤波参数
//输入参数：use(开关：1开、0关)、radius(半径）、num（有效点）
//输出参数： 无
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfSetParamRadiusFilter(int use, float radius, int num)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfSetParamRadiusFilter:";
	if (use != 1 && use != 0)
	{
		std::cout << "use param should be 1 or 0:  " << use << std::endl;
		return DF_FAILED;
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_SET_PARAM_RADIUS_FILTER, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = send_buffer((char*)(&use), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = send_buffer((char*)(&radius), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = send_buffer((char*)(&num), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetParamRadiusFilter
//功能： 获取点云半径滤波参数
//输入参数：无
//输出参数：use(开关：1开、0关)、radius(半径）、num（有效点）
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetParamRadiusFilter(int& use, float& radius, int& num)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetParamRadiusFilter:";
	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PARAM_RADIUS_FILTER, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{

		ret = recv_buffer((char*)(&use), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = recv_buffer((char*)(&radius), sizeof(float), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = recv_buffer((char*)(&num), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfSetParamMultipleExposureModel
//功能： 设置多曝光模式
//输入参数： model(1：HDR(默认值)、2：重复曝光)
//输出参数：无
//返回值： 类型（int）:返回0表示设置参数成功;否则失败。
DF_SDK_API int DfSetParamMultipleExposureModel(int model)
{
	if (model != 1 && model != 2)
	{
		return DF_ERROR_INVALID_PARAM;
	}
	multiple_exposure_model_ = model;

	return DF_SUCCESS;
}

//函数名： DfCaptureData
//功能： 采集一帧数据并阻塞至返回状态
//输入参数： exposure_num（曝光次数）：大于1的为多曝光模式
//输出参数： timestamp(时间戳)
//返回值： 类型（int）:返回0表示获取采集数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfCaptureData(int exposure_num, char* timestamp)
{

	LOG(INFO) << "DfCaptureData: " << exposure_num;
	bool ret = -1;

	if (exposure_num > 1)
	{
		switch (multiple_exposure_model_)
		{
		case 1:
		{
			ret = DfGetFrame01HDR(depth_buf_, depth_buf_size_, brightness_buf_, brightness_bug_size_);
			if (DF_SUCCESS != ret)
			{
				return ret;
			}
		}
		break;
		default:
			break;
		}

	}
	else
	{

		ret = DfGetFrame01(depth_buf_, depth_buf_size_, brightness_buf_, brightness_bug_size_);
		if (DF_SUCCESS != ret)
		{
			return ret;
		}
	}


	std::string time = get_timestamp();
	for (int i = 0; i < time.length(); i++)
	{
		timestamp[i] = time[i];
	}

	transform_pointcloud_flag_ = false;

	int status = DF_SUCCESS;
	ret = DfGetFrameStatus(status);
	if (DF_SUCCESS != ret && DF_UNKNOWN != ret)
	{
		LOG(INFO) << "DfGetFrameStatus Failed!";
		//return ret;
	}

	LOG(INFO) << "Frame Status: " << status;
	if (DF_SUCCESS != status)
	{
		return status;
	}

	return DF_SUCCESS;
}

//函数名： DfGetFrameStatus
//功能： 获取当前帧数据状态
//输入参数：无
//输出参数： status（状态码）
//返回值： 类型（int）:返回0表示获取数据成功;否则表示获取数据失败.
DF_SDK_API int DfGetFrameStatus(int& status)
{
	//std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	//while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	//{
	//	LOG(INFO) << "--";
	//}

	int get_status = 0;

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_ERROR_NETWORK;
	}
	ret = send_command(DF_CMD_GET_FRAME_STATUS, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_ERROR_NETWORK;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&get_status), sizeof(int), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_ERROR_NETWORK;
		}

		LOG(INFO) << "Frame Status: " << status;
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}
	else if (command == DF_CMD_UNKNOWN)
	{
		close_socket(g_sock);
		return DF_UNKNOWN;
	}

	close_socket(g_sock);
	status = get_status;
	return DF_SUCCESS;
}

//函数名： DfGetBrightnessData
//功能： 采集点云数据并阻塞至返回结果
//输入参数：无
//输出参数： brightness(亮度图)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetBrightnessData(unsigned char* brightness)
{
	LOG(INFO) << "DfGetBrightnessData:";
	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}


	LOG(INFO) << "Trans Brightness:";

	memcpy(brightness, brightness_buf_, brightness_bug_size_);

	//brightness = brightness_buf_;

	LOG(INFO) << "Get Brightness!";

	return 0;
}

//函数名： DfGetDepthDataFloat
//功能： 获取深度图
//输入参数：无
//输出参数： depth(深度图)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetDepthDataFloat(float* depth)
{
	LOG(INFO) << "DfGetDepthDataFloat:";
	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}


	LOG(INFO) << "Trans Depth:";
	int point_num = camera_height_ * camera_width_;

	int nr = camera_height_;
	int nc = camera_width_;

#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{
		for (int c = 0; c < nc; c++)
		{
			int offset = r * camera_width_ + c;
			depth[offset] = depth_buf_[offset];

		}

	}

	LOG(INFO) << "Get Depth!";

	return DF_SUCCESS;
}

DF_SDK_API int DfGetFirmwareVersion(char* pVersion, int length)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_FIRMWARE_VERSION, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer(pVersion, length, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfGetProductInfo(char* info, int length)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_PRODUCT_INFO, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer(info, length, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

DF_SDK_API int DfGetNetworkBandwidth(int& speed)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_NETWORK_BANDWIDTH, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&speed), sizeof(speed), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

//函数名： DfGetHeightMapData
//功能： 采集点云数据并阻塞至返回结果
//输入参数：无
//输出参数： height_map(高度映射图)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetHeightMapData(float* height_map)
{

	LOG(INFO) << "DfGetHeightMapData:";
	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}


	struct SystemConfigParam system_config_param;
	int ret_code = DfGetSystemConfigParam(system_config_param);
	if (0 != ret_code)
	{
		std::cout << "Get Param Error;";
		return -1;
	}

	LOG(INFO) << "Transform Pointcloud:";

	if (!transform_pointcloud_flag_)
	{
		depthTransformPointcloud((float*)depth_buf_, (float*)point_cloud_buf_);
		transform_pointcloud_flag_ = true;
	}

	//memcpy(trans_point_cloud_buf_, point_cloud_buf_, pointcloud_buf_size_);
	transformPointcloud((float*)point_cloud_buf_, (float*)trans_point_cloud_buf_, system_config_param.standard_plane_external_param, &system_config_param.standard_plane_external_param[9]);


	int nr = camera_height_;
	int nc = camera_width_;
#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{
		for (int c = 0; c < nc; c++)
		{
			int offset = r * camera_width_ + c;
			if (depth_buf_[offset] > 0)
			{
				height_map[offset] = trans_point_cloud_buf_[offset * 3 + 2];
			}
			else
			{
				height_map[offset] = NULL;
			}

		}


	}


	LOG(INFO) << "Get Height Map!";

	return 0;
}

//函数名： DfGetPointcloudData
//功能： 采集点云数据并阻塞至返回结果
//输入参数：无
//输出参数： point_cloud(点云)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetPointcloudData(float* point_cloud)
{
	LOG(INFO) << "DfGetPointcloudData:";
	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}


	if (!transform_pointcloud_flag_)
	{
		depthTransformPointcloud(depth_buf_, point_cloud_buf_);
		transform_pointcloud_flag_ = true;
	}

	memcpy(point_cloud, point_cloud_buf_, pointcloud_buf_size_);


	LOG(INFO) << "Get Pointcloud!";

	return 0;
}

//函数名： DfGetStandardPlaneParam
//功能： 获取基准平面参数
//输入参数：无
//输出参数： R(旋转矩阵：3*3)、T(平移矩阵：3*1)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetStandardPlaneParam(float* R, float* T)
{
	std::unique_lock<std::timed_mutex> lck(command_mutex_, std::defer_lock);
	while (!lck.try_lock_for(std::chrono::milliseconds(1)))
	{
		LOG(INFO) << "--";
	}

	LOG(INFO) << "DfGetStandardPlaneParam: ";

	int ret = setup_socket(camera_id_.c_str(), DF_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}

	int param_buf_size = 12 * 4;
	float* plane_param = new float[param_buf_size];

	ret = send_command(DF_CMD_GET_STANDARD_PLANE_PARAM, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(ERROR) << "Failed to recv command";
		close_socket(g_sock);
		return DF_FAILED;
	}

	if (command == DF_CMD_OK)
	{
		LOG(INFO) << "token checked ok";
		LOG(INFO) << "receiving buffer, param_buf_size=" << param_buf_size;
		ret = recv_buffer((char*)plane_param, param_buf_size, g_sock);
		LOG(INFO) << "plane param received";
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		LOG(INFO) << "Get frame rejected";
		close_socket(g_sock);
		return DF_BUSY;
	}

	close_socket(g_sock);


	memcpy(R, plane_param, 9 * 4);
	memcpy(T, plane_param + 9, 3 * 4);

	delete[] plane_param;

	LOG(INFO) << "Get plane param success";
	return DF_SUCCESS;

}

//函数名： DfGetHeightMapDataBaseParam
//功能： 获取校正到基准平面的高度映射图
//输入参数：R(旋转矩阵)、T(平移矩阵)
//输出参数： height_map(高度映射图)
//返回值： 类型（int）:返回0表示获取数据成功;返回-1表示采集数据失败.
DF_SDK_API int DfGetHeightMapDataBaseParam(float* R, float* T, float* height_map)
{
	LOG(INFO) << "DfGetHeightMapDataBaseParam:";
	if (!connected_flag_)
	{
		return DF_NOT_CONNECT;
	}



	if (!transform_pointcloud_flag_)
	{
		depthTransformPointcloud((float*)depth_buf_, (float*)point_cloud_buf_);
		transform_pointcloud_flag_ = true;
	}

	transformPointcloud((float*)point_cloud_buf_, (float*)trans_point_cloud_buf_, R, T);


	int nr = camera_height_;
	int nc = camera_width_;
#pragma omp parallel for
	for (int r = 0; r < nr; r++)
	{
		for (int c = 0; c < nc; c++)
		{
			int offset = r * camera_width_ + c;
			if (depth_buf_[offset] > 0)
			{
				height_map[offset] = trans_point_cloud_buf_[offset * 3 + 2];
			}
			else
			{
				height_map[offset] = NULL;
			}

		}


	}


	LOG(INFO) << "Get Height Map!";

	return 0;
}

