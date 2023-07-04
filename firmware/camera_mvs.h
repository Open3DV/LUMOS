#pragma once
#include <stdlib.h>
#include <stdio.h>
#include <malloc.h> 
#include <pthread.h>
#include <MvCameraControl.h>
#include "camera.h"
#include <chrono>         // std::chrono::milliseconds
#include <thread>         // std::thread
#include <mutex>          // std::timed_mutex


#define NUM_BUFFERS 5         /* Number of buffers used for grabbing. */

class CameraMVS: public Camera
{
public:
	CameraMVS();
	~CameraMVS();
 
	bool openCamera(); 
	bool openCameraLeft(); 
	bool openCameraRight(); 
	bool closeCamera(); 

	bool switchToInternalTriggerMode(); 
	bool switchToExternalTriggerMode();

	bool getExposure(double &val); 
	bool setExposure(double val); 

	bool getGain(double &value);
	bool setGain(double value);  

    bool streamOn(); 
	bool streamOff();
 
    bool trigger_software();
    bool grap(unsigned char* buf);

	bool grap(unsigned short* buf);

	bool setPixelFormat(int val);
private:
	void streamOffThread();
private:
  
    void* handle_ = NULL;                     /* Handle for the pylon device. */ 
 
	std::mutex operate_mutex_;
};