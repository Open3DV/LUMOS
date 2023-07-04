#pragma once

int KillCameraServer(int &feedback);

int GetCameraServer(char* org_buffer, unsigned int pattern_size);

int ChmodCameraServer(int& feedback);

int RebootDevice(int& feedback);

int UpdateOnDropped(int (*p_function)(void*));

int UpdateConnect(const char* ip);

int UpdateDisconnect();

int on_dropped(void* param);