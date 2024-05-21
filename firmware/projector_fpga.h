#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "projector_base.h"
#include "spi.h"
#include "protocol.h"
#include "devmem.h"

#define SPI_WRITE_HEAD 0xF0F0
#define SPI_READ_HEAD 0x8080
#define CV_2PI 6.283185307179586476925286766559
#define PERIOD_POINT    4096
#define LASER_NUM_BUFFER 250 //代表的是用于缓冲的不投光的部分
#define SHIFT_OFFSET 0

#define ROLLING_US_PER_ROW 7.79

#define SCANNER_ROLL_BACK_TIME_MS 8

struct FpgaRegSingle {
	uint32_t reg;
	uint32_t val;
};

struct FpgaRegSet {
	struct FpgaRegSingle LaserCmd;              // 振镜输出指令
	struct FpgaRegSingle LaserFreq;             // 输出点的间隔
	struct FpgaRegSingle RunTime;               // 
	struct FpgaRegSingle OutPoint;              // 
	struct FpgaRegSingle BankSelect;            //
	struct FpgaRegSingle InnerAddr;             //
	struct FpgaRegSingle InnerData;             // 
	struct FpgaRegSingle Trigger;               //
	struct FpgaRegSingle LaserBrightness;       //
	struct FpgaRegSingle LaserDarkness;         //
	struct FpgaRegSingle LaserClosedCurrent;    //
	struct FpgaRegSingle LaserCyclesNum;        //
};

class ProjectorTemp {
public:
    ProjectorTemp();
    ~ProjectorTemp();

    bool init();
    bool project();
    bool stopProject();
    bool setProjectorFrequency(int freq);
    bool setLaserCurrent(int brightness);
    bool setProjectorWorkingMode(int serial_num);
    bool generatePatternDataMode0(int* data);
    bool generatePatternDataMode1(int* data);
private:
    void camera_reg_setting();
    std::vector<cv::Mat> generateGrayCode(std::vector<int> period, int width, int height);

    /*********************************************/
    bool generateBlackWhitePattern(std::vector<cv::Mat>& input_output_patterns, int width, int height);
    bool generateGrayCode(std::vector<cv::Mat>& input_output_patterns, std::vector<int> period, int width, int height);
    bool generateGrayCodeShift(std::vector<cv::Mat>& input_output_pattern, int pattern_period, int shift_val, int width, int height);
    /*********************************************/
    int FpgaSpi_Read(uint32_t RegAddr, uint32_t *data);
    int FpgaSpi_SeqWrite(uint32_t RegAddr, uint32_t *txBuf, uint32_t len);
    int FpgaSpi_Write(uint32_t RegAddr, uint32_t data);
    uint8_t reverse_bit(uint8_t x) ;
    uint32_t byte_order(uint32_t orgdata);

    spi_t m_spi;
    FpgaRegSet m_RegSet;

    int beginIdx = -1;
    int endIdx = -1;
};

class FpgaProjector: public BaseProjector
{
    public:
    FpgaProjector();
    ~FpgaProjector();

    bool init();

    bool project();

    bool setProjectorCurrent(int current);

    bool setProjectorExposure(int exposure);

    bool getProjectorExposure(int& exposure);

    bool setProjectorTriggerDlay(int dlay);

    bool setProjectorWorkingMode(int mode);

    bool getMinExposure(float& minExposure);
    
    bool getCorrectExposure(int& exposureTime);

    private:

    ProjectorTemp* spi_projector_ori_;

    int trigger_offset_;

    int max_exposure_;

    int min_exposure_;
};
