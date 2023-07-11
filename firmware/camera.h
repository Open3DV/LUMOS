#pragma once
#include<iostream> 

class Camera
{
public:
	Camera();
	~Camera();

	virtual bool openCamera() = 0;

	virtual bool openCameraLeft(){return false;};
	
	virtual bool openCameraRight(){return false;};

	virtual bool openCameraBySN(std::string sn){return false;}

	virtual bool closeCamera(); 
	
	virtual bool switchToInternalTriggerMode();

	virtual bool switchToExternalTriggerMode();

	virtual bool getExposure(double &val){}; 
	virtual bool setExposure(double val){}; 
    
	virtual bool getGain(double &val){};
	virtual bool setGain(double val){};
	  
	virtual bool setTriggerDelay(double val) {};
	virtual bool getTriggerDelay(double& val) {};

	virtual bool streamOn(){}; 
	virtual bool streamOff(){};

    virtual bool trigger_software(){};

    virtual bool grap(unsigned char* buf){};

	virtual bool grap(unsigned short* buf){};

	virtual bool setPixelFormat(int val){};

	bool getImageSize(int &width,int &height);
	
	bool getMinExposure(float &val);


protected:
 
	bool camera_opened_state_; 
 

	long int image_width_;
	long int image_height_;

	float min_camera_exposure_; 
	float max_camera_exposure_; 
	
	bool stream_off_flag_;
	bool trigger_on_flag_;
};

