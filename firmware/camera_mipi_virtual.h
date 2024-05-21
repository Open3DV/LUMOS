#pragma once
#include <stdlib.h>
#include <stdio.h>
#include <malloc.h> 
#include <pthread.h>
#include "camera.h"
#include <chrono>         // std::chrono::milliseconds
#include <thread>         // std::thread
#include <mutex>          // std::timed_mutex

#include <unistd.h>
#include <getopt.h>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

class CameraMIPIVirtual: public Camera
{
public:
	CameraMIPIVirtual();
	~CameraMIPIVirtual(){}
 
	bool openCamera();
	bool openCameraLeft(){return false;}
	bool openCameraRight(){return false;}
	bool openCameraBySN(std::string sn){return false;}
	bool closeCamera(){return false;}

	bool switchToInternalTriggerMode(){return false;}
	bool switchToExternalTriggerMode(){return false;}

	bool getExposure(double &val){return false;}
	bool setExposure(double val){return false;}

	bool getGain(double &value){return false;}
	bool setGain(double value){return false;}

    bool streamOn(); 
	bool streamOff();
 
    bool trigger_software(){return false;}
    bool grap(unsigned char* buf);

	bool grap(unsigned short* buf){return false;}

	bool setPixelFormat(int val){return false;}

private:
	cv::Mat image_to_grap_;
};