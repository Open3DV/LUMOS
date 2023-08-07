#pragma once
#include <stdlib.h>
#include <stdio.h>
#include <malloc.h> 
#include <pthread.h>
#include "camera.h"
#include <chrono>         // std::chrono::milliseconds
#include <thread>         // std::thread
#include <mutex>          // std::timed_mutex

#include "Error.h"
#include "Thread.h"

#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>

#include <NvEglRenderer.h>
#include <NvJpegEncoder.h>

#include <unistd.h>
#include <getopt.h>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

using namespace Argus;
using namespace EGLStream;

class FactoryThread : public ArgusSamples::Thread
{
public:
    FactoryThread()
    {

    }
    virtual ~FactoryThread()
    {

    }

    OutputStream* outstream_ptr_m;
    UniqueObj<FrameConsumer> consumer_m;
    UniqueObj<CaptureSession> captureSession_m;
    Request *request_ptr_m;
    IRequest *iRequest_ptr_m;
    ICaptureSession *iCaptureSession_m;

    int image_width_m;
    int image_height_m;

protected:
    virtual bool threadInitialize(){std::cout << "threadInitialize Factory" << std::endl;return true;}
    virtual bool threadExecute();
    virtual bool threadShutdown(){return true;}
};

/* Configurations below can be overrided by cmdline */
static int      CAPTURE_FPS   = 10;
static Size2D<uint32_t> PREVIEW_SIZE (1632*2, 1232*2);

#define JPEG_BUFFER_SIZE    (CAPTURE_SIZE.area() * 3 / 2)

/* Debug print macros. */
#define PRODUCER_PRINT(...) printf("PRODUCER: " __VA_ARGS__)
#define CONSUMER_PRINT(...) printf("CONSUMER: " __VA_ARGS__)


class CameraMIPI: public Camera
{
public:
	CameraMIPI();
	~CameraMIPI();
 
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
	void streamOffThread(){return;}
    bool capture(int& i, int& capture_buf, IEGLOutputStream *iEglOutputStream, IFrameConsumer *iFrameConsumer, unsigned char* output_buffer);
private:
  
    FactoryThread* factoryThread_m;

    int capture_buf_m;
    int i_m;
    OutputStream *stream_m;

    IEGLOutputStream *iEglOutputStream_m;
    IFrameConsumer *iFrameConsumer_m;

};