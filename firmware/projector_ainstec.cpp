#include "projector_ainstec.h"
#include <iostream>
#include <unistd.h>

AinstecProjector::AinstecProjector()
{
    devName_ = "/dev/ttyTHS2";

    serialHandle_ = -1;

    minProjectorExposure_ = 1000;

    projectorCurrent_ = 1023;

    projectorExposure_ = 10000;

    projectorTriggerDlay_ = 0;

    projectorWorkingMode_ = 0;
}

AinstecProjector::~AinstecProjector()
{

}

bool AinstecProjector::init()
{
    // 打开串口
    serialHandle_ = open_serial(devName_.c_str());
    if (serialHandle_ == -1)
    {
        std::cerr << "Open serial err! " << std::endl;
        return false;
    }
    
    // 配置好光机的图像
    setProjectorWorkingMode(1);

    float minExposure;

    getMinExposure(minExposure);

    // 设置光机的曝光时间
    int exposure = 10000;

    getCorrectExposure(exposure);
    
    if (setProjectorExposure(exposure))
    {
        projectorExposure_ = exposure;
    }
    else
    {
        return false;
    }

    // 设置光机的亮度
    int current = 1023;
    if (setProjectorCurrent(current))
    {
        projectorCurrent_ = current;
    }
    else
    {
        return false;
    }

}

bool AinstecProjector::project()
{
    char send_buff[] = { 0x5a, 0xe1, 0x04, 0xa5 };
    int len = -1;
    len = serial_write(serialHandle_, send_buff, sizeof(send_buff));
    if (len == -1)
    {
        return false;
    }

    return true;
}

bool AinstecProjector::setProjectorCurrent(int current)
{
    // 待确认光强设置的问题
    return true;
}

bool AinstecProjector::setProjectorExposure(int exposure)
{
    // 要用无符号整型表示，并且单位转换成0.1ms

    if (exposure > 110000)
    {
        std::cerr << "Exposure out of range! " << std::endl;
        return false;
    }
    else if (exposure < 15000)
    {
        exposure = 15000;
    }

    exposure += 1000;

    unsigned short exposure_100us = exposure / 100;

    std::cout << "Set projector exposure: " << exposure_100us << " * 100us" << std::endl;

    char send_buff[] = { 0x5a, 0xe3, 0x01, 0x00, 0x00, 0xa5 };

    send_buff[3] = exposure_100us >> 8;
    send_buff[4] = (exposure_100us << 8) >> 8;

    int len = -1;
    len = serial_write(serialHandle_, send_buff, sizeof(send_buff));
    if (len == -1)
    {
        return false;
    }

    return true;

}

bool AinstecProjector::getProjectorExposure(int& exposure)
{
    // 要用无符号整型表示，并且单位转换成0.1ms

    if (exposure > 100000)
    {
        std::cerr << "Exposure out of range! " << std::endl;
        return false;
    }
    else if (exposure < 15000)
    {
        exposure = 15000;
    }

    unsigned short exposure_100us = exposure / 100;

    std::cout << "Set projector exposure: " << exposure_100us << " * 100us" << std::endl;

    char send_buff[] = { 0x5a, 0xe3, 0x01, 0x00, 0x00, 0xa5 };

    send_buff[3] = exposure_100us >> 8;
    send_buff[4] = (exposure_100us << 8) >> 8;

    int len = -1;
    len = serial_write(serialHandle_, send_buff, sizeof(send_buff));
    if (len == -1)
    {
        return false;
    }

    return true;

}

bool AinstecProjector::setProjectorTriggerDlay(int dlay)
{
    return true;
}

bool AinstecProjector::setProjectorWorkingMode(int mode)
{
    char mode0[] = {0x5a, 0xc1, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    //设置新的预置图

                    //8张格雷码照片
                    0x5a, 0xc1, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x04, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x04, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x04, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x04, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x04, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    //128周期0,π/2,π,3π/2相位
                    0x5a, 0xc1, 0x03, 0x00, 0x80, 0x40, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x03, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x03, 0x00, 0x80, 0xc0, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x03, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x06, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa5,

                    //保存预置图像
                    0x5a, 0xc1, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    //设置相机触发信号脉宽
                    0x5a, 0xe2, 0x05, 0xa5
                    };

    char mode1[] = {0x5a, 0xc1, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    //设置新的预置图

                    //8张格雷码照片
                    0x5a, 0xc1, 0x04, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x04, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x04, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x04, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x04, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x04, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0xa5,

                    //128周期0,π/2,π,3π/2相位
                    0x5a, 0xc1, 0x03, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x03, 0x00, 0x80, 0x20, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x03, 0x00, 0x80, 0x40, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x03, 0x00, 0x80, 0x60, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x03, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x03, 0x00, 0x80, 0xa0, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x03, 0x00, 0x80, 0xc0, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x03, 0x00, 0x80, 0xe0, 0x00, 0x00, 0x00, 0xa5,

                    0x5a, 0xc1, 0x06, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    0x5a, 0xc1, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa5,

                    //保存预置图像
                    0x5a, 0xc1, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa5,
                    //设置相机触发信号脉宽
                    0x5a, 0xe2, 0x05, 0xa5
                    };

                    
    int len = -1;
    
    switch (mode)
    {
    case 0:
        len = serial_write(serialHandle_, mode0, sizeof(mode0));
        break;
    case 1:
        len = serial_write(serialHandle_, mode1, sizeof(mode1));
        break;
    
    default:
        break;
    }

    if (len == -1)
    {
        return false;
    }

    return true;

}

bool AinstecProjector::getMinExposure(float& minExposure)
{
    unsigned char minExposeurBuff[100] = {0};

    char cmd1[] = {0x5a, 0xb1, 0x01, 0xa5};
    char cmd2[] = {0x5a, 0xb1, 0x02, 0xa5};

    int len = -1;

    sleep(1);

    len = serial_write(serialHandle_, cmd1, sizeof(cmd1));
    if (len == -1)
    {
        return false;
    }

    len = serial_read(serialHandle_, (char*)minExposeurBuff, sizeof(char));
    if (len == -1)
    {
        return false;
    }

    len = serial_write(serialHandle_, cmd2, sizeof(cmd2));
    if (len == -1)
    {
        return false;
    }

    len = serial_read(serialHandle_, (char*)minExposeurBuff + 1, sizeof(char) * 50);
    if (len == -1)
    {
        return false;
    }

    unsigned short frequency = 0;
    frequency = minExposeurBuff[0];
    frequency = (frequency << 8) + minExposeurBuff[1];
    std::cout << "frequency: " << frequency << std::endl;

    minExposure = 1000000. / frequency;

    minExposure_ = minExposure;

    return true;
}

bool AinstecProjector::getCorrectExposure(int& exposureTime)
{
    if (exposureTime < 2500)
    {
        exposureTime = 2500;
    }
    if (exposureTime > 100000)
    {
        exposureTime = 100000;
    }

    exposureTime = ((int)(exposureTime / minExposure_) + 1) * minExposure_;

    std::cout << "getCorrectExposure: " << exposureTime << std::endl;
    std::cout << "minExposure_: " << minExposure_ << std::endl;

    return true;
}


