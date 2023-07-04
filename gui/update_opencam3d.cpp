#include <assert.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <ctime>
#include <time.h>
#include <stddef.h> 
#include <vector>
#include "../firmware/protocol.h" 
#include "../SDK/socket_tcp.h"
#include "update_opencam3d.h"
#include "../firmware/easylogging++.h"

INITIALIZE_EASYLOGGINGPP

using namespace std;
using namespace std::chrono;

bool connected = false;
long long token = 0;
std::string camera_id_;

extern SOCKET g_sock;

int (*p_OnDropped)(void*) = 0;
bool connected_flag_ = false;
const char* camera_ip_ = "";

std::vector<std::string> mac_list_;
std::vector<std::string> ip_list_;

int on_dropped(void* param)
{
	std::cout << "Network dropped!" << std::endl;
	return 0;
}

int UpdateOnDropped(int (*p_function)(void*))
{
	p_OnDropped = p_function;
	return 0;
}

int UpdateConnect(const char* ip)
{
	camera_id_ = ip;
	LOG(INFO) << "update start connection: " << ip;
	int ret = setup_socket(camera_id_.c_str(), UPDATE_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}

	LOG(INFO) << "update sending connection cmd";
	ret = send_command(DF_CMD_CONNECT, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(INFO) << "update Failed to send connection cmd";
		close_socket(g_sock);
		return DF_FAILED;
	}
	int command;
	ret = recv_command(&command, g_sock);
	if (ret == DF_SUCCESS)
	{
		if (command == DF_CMD_OK)
		{
			LOG(INFO) << "update Recieved connection ok";
			ret = recv_buffer((char*)&token, sizeof(token), g_sock);
			if (ret == DF_SUCCESS)
			{
				connected = true;
				LOG(INFO) << "update token: " << token;
				close_socket(g_sock);

				return DF_SUCCESS;
			}
		}
		else if (command == DF_CMD_REJECT)
		{
			LOG(INFO) << "update connection rejected";
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	return DF_FAILED;
}

int UpdateDisconnect()
{
	LOG(INFO) << "update token " << token << " update try to disconnection";

	int ret = setup_socket(camera_id_.c_str(), UPDATE_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(INFO) << "update Failed to setup_socket";
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_DISCONNECT, g_sock);
	if (ret == DF_FAILED)
	{
		LOG(INFO) << "udpate Failed to send disconnection cmd";
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	if (ret == DF_FAILED)
	{
		LOG(INFO) << "update Failed to send token";
		close_socket(g_sock);
		return DF_FAILED;
	}
	int command;
	ret = recv_command(&command, g_sock);

	connected = false;
	token = 0;

	LOG(INFO) << "update Camera disconnected";
	return close_socket(g_sock);
}

int KillCameraServer(int & feedback)
{
	int ret = setup_socket(camera_id_.c_str(), UPDATE_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_KILL_CAMERA_SERVER, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&feedback), sizeof(feedback), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

int GetCameraServer(char* org_buffer, unsigned int file_size)
{
	int ret = setup_socket(camera_id_.c_str(), UPDATE_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_GET_CAMERA_SERVER, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		ret = send_buffer((char*)(&file_size), sizeof(file_size), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}

		ret = send_buffer(org_buffer, file_size, g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

int ChmodCameraServer(int& feedback)
{
	int ret = setup_socket(camera_id_.c_str(), UPDATE_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_CHMOD_CAMERA_SERVER, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&feedback), sizeof(feedback), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}

int RebootDevice(int& feedback)
{
	int ret = setup_socket(camera_id_.c_str(), UPDATE_PORT, g_sock);
	if (ret == DF_FAILED)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}
	ret = send_command(DF_CMD_REBOOT_DEVICE, g_sock);
	ret = send_buffer((char*)&token, sizeof(token), g_sock);
	int command;
	ret = recv_command(&command, g_sock);
	if (command == DF_CMD_OK)
	{
		ret = recv_buffer((char*)(&feedback), sizeof(feedback), g_sock);
		if (ret == DF_FAILED)
		{
			close_socket(g_sock);
			return DF_FAILED;
		}
	}
	else if (command == DF_CMD_REJECT)
	{
		close_socket(g_sock);
		return DF_FAILED;
	}

	close_socket(g_sock);
	return DF_SUCCESS;
}
