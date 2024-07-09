#include "camera_mvs.h"


bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }
    return true;
}

CameraMVS::CameraMVS()
{
    pData_ = NULL;
    nDataSize_ = 0;
    camera_sn_ = "";
    camera_is_opened_ = false;
}

CameraMVS::~CameraMVS()
{

}

bool CameraMVS::openCamera()
{
    int nRet = MV_OK;

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 枚举设备
    // enum device
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);  
        return false;
    }

    if (stDeviceList.nDeviceNum > 0)
    {
        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                break;
            }
            PrintDeviceInfo(pDeviceInfo);
        }
    }
    else
    {
        printf("Find No Devices!\n"); 
        return false;
    }

    // 选择设备并创建句柄
    // select device and create handle
    nRet = MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[0]);
    if (MV_OK != nRet)
    {
        printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
        return false;
    }

    // 打开设备
    // open device
    nRet = MV_CC_OpenDevice(handle_);
    if (MV_OK != nRet)
    {
        printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);

        nRet = MV_CC_DestroyHandle(handle_);
 
        return false;
    }

    // 设置触发模式为off
    // set trigger mode as on
    nRet = MV_CC_SetEnumValue(handle_, "TriggerMode", 1);
    if (MV_OK != nRet)
    {
        printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
        return false;
    }

    // 设置触发源
    // set trigger source
    nRet = MV_CC_SetEnumValue(handle_, "TriggerSource", MV_TRIGGER_SOURCE_LINE2);
    if (MV_OK != nRet)
    {
        printf("MV_CC_Set TriggerSource fail! nRet [%x]\n", nRet);
        return false;
    }

    // nRet = MV_CC_SetEnumValue(handle_, "PixelFormat", MvGvspPixelType::PixelType_Gvsp_Mono8);
    // if (MV_OK != nRet)
    // {
    //     printf("MV_CC_Set PixelFormat fail! nRet [%x]\n", nRet);
    //     return false;
    // }
    setPixelFormat(12);

    std::cout << "setPixelFormat(12)" << std::endl;

    // get IInteger variable
    MVCC_INTVALUE stHeight = {0};
    nRet = MV_CC_GetIntValue(handle_, "Height", &stHeight);
    if (MV_OK == nRet)
    {
        image_height_ = stHeight.nCurValue; 
    }
    else
    {
        printf("get height failed! nRet [%x]\n\n", nRet);
    }

    MVCC_INTVALUE stwidth = {0};
    nRet = MV_CC_GetIntValue(handle_, "Width", &stwidth);
    if (MV_OK == nRet)
    {
        image_width_ = stwidth.nCurValue; 
    }
    else
    {
        printf("get height failed! nRet [%x]\n\n", nRet);
    }
    

    return true;
}

bool CameraMVS::openCameraLeft()// 打开SN码为奇数
{
    int nRet = MV_OK;

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 枚举设备
    // enum device
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);  
        return false;
    }

    int openNum = 0;
    if (stDeviceList.nDeviceNum > 0)
    {
        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                continue;
            }
            PrintDeviceInfo(pDeviceInfo);

            int lastSnNum = int(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber[8] - '0');
            if (lastSnNum % 2 == 1)
            {
                std::cout << "open left cam: " << lastSnNum << std::endl;
                openNum = i;
            }
        }
    }
    else
    {
        printf("Find No Devices!\n"); 
        return false;
    }

    // 选择设备并创建句柄
    // select device and create handle
    nRet = MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[openNum]);
    if (MV_OK != nRet)
    {
        printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
        return false;
    }

    // 打开设备
    // open device
    nRet = MV_CC_OpenDevice(handle_);
    if (MV_OK != nRet)
    {
        printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);

        nRet = MV_CC_DestroyHandle(handle_);
 
        return false;
    }

    // 设置触发模式为off
    // set trigger mode as on
    nRet = MV_CC_SetEnumValue(handle_, "TriggerMode", 1);
    if (MV_OK != nRet)
    {
        printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
        return false;
    }

    // 设置触发源
    // set trigger source
    nRet = MV_CC_SetEnumValue(handle_, "TriggerSource", MV_TRIGGER_SOURCE_LINE2);
    if (MV_OK != nRet)
    {
        printf("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
        return false;
    }

    setPixelFormat(12);

    std::cout << "setPixelFormat(12)" << std::endl;

    // get IInteger variable
    MVCC_INTVALUE stHeight = {0};
    nRet = MV_CC_GetIntValue(handle_, "Height", &stHeight);
    if (MV_OK == nRet)
    {
        image_height_ = stHeight.nCurValue; 
    }
    else
    {
        printf("get height failed! nRet [%x]\n\n", nRet);
    }

    MVCC_INTVALUE stwidth = {0};
    nRet = MV_CC_GetIntValue(handle_, "Width", &stwidth);
    if (MV_OK == nRet)
    {
        image_width_ = stwidth.nCurValue; 
    }
    else
    {
        printf("get height failed! nRet [%x]\n\n", nRet);
    }
    

    return true;
}

bool CameraMVS::openCameraRight()// 打开SN码为偶数
{
    int nRet = MV_OK;

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 枚举设备
    // enum device
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);  
        return false;
    }

    int openNum = 0;
    if (stDeviceList.nDeviceNum > 0)
    {
        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                continue;
            }
            PrintDeviceInfo(pDeviceInfo);
            int lastSnNum = int(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber[8] - '0');
            if (lastSnNum % 2 == 0)
            {
                std::cout << "open right cam: " << lastSnNum << std::endl;
                openNum = i;
            }
        }
    }
    else
    {
        printf("Find No Devices!\n"); 
        return false;
    }

    // 选择设备并创建句柄
    // select device and create handle
    nRet = MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[openNum]);
    if (MV_OK != nRet)
    {
        printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
        return false;
    }

    // 打开设备
    // open device
    nRet = MV_CC_OpenDevice(handle_);
    if (MV_OK != nRet)
    {
        printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);

        nRet = MV_CC_DestroyHandle(handle_);
 
        return false;
    }

    // 设置触发模式为off
    // set trigger mode as on
    nRet = MV_CC_SetEnumValue(handle_, "TriggerMode", 1);
    if (MV_OK != nRet)
    {
        printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
        return false;
    }

    // 设置触发源
    // set trigger source
    nRet = MV_CC_SetEnumValue(handle_, "TriggerSource", MV_TRIGGER_SOURCE_LINE2);
    if (MV_OK != nRet)
    {
        printf("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
        return false;
    }

    setPixelFormat(12);

    std::cout << "setPixelFormat(12)" << std::endl;

    // get IInteger variable
    MVCC_INTVALUE stHeight = {0};
    nRet = MV_CC_GetIntValue(handle_, "Height", &stHeight);
    if (MV_OK == nRet)
    {
        image_height_ = stHeight.nCurValue; 
    }
    else
    {
        printf("get height failed! nRet [%x]\n\n", nRet);
    }

    MVCC_INTVALUE stwidth = {0};
    nRet = MV_CC_GetIntValue(handle_, "Width", &stwidth);
    if (MV_OK == nRet)
    {
        image_width_ = stwidth.nCurValue; 
    }
    else
    {
        printf("get height failed! nRet [%x]\n\n", nRet);
    }
    

    return true;
}

bool CameraMVS::openCameraBySN(std::string sn)
{
    int nRet = MV_OK;

    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    // 枚举设备
    // enum device
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);
    if (MV_OK != nRet)
    {
        printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);  
        return false;
    }

    std::cout << "open cam use sn: " << sn << std::endl;

    int openNum = 0;
    if (stDeviceList.nDeviceNum > 0)
    {
        for (int i = 0; i < stDeviceList.nDeviceNum; i++)
        {
            printf("[device %d]:\n", i);
            MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
            if (NULL == pDeviceInfo)
            {
                continue;
            }
            PrintDeviceInfo(pDeviceInfo);
            printf("sn: %s\n", pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
            if (0 == strcmp((const char*)pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber, sn.c_str()))
            {
                std::cout << "open cam by sn: " << pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber << std::endl;
                openNum = i;
                break;
            }
        }
    }
    else
    {
        printf("Find No Devices!\n"); 
        return false;
    }

    // 选择设备并创建句柄
    // select device and create handle
    nRet = MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[openNum]);
    if (MV_OK != nRet)
    {
        printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
        return false;
    }

    // 打开设备
    // open device
    nRet = MV_CC_OpenDevice(handle_);
    if (MV_OK != nRet)
    {
        printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);

        nRet = MV_CC_DestroyHandle(handle_);
 
        return false;
    }

    // 设置触发模式为off
    // set trigger mode as on
    nRet = MV_CC_SetEnumValue(handle_, "TriggerMode", 1);
    if (MV_OK != nRet)
    {
        printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
        return false;
    }

    // 设置触发源
    // set trigger source
    nRet = MV_CC_SetEnumValue(handle_, "TriggerSource", MV_TRIGGER_SOURCE_LINE2);
    if (MV_OK != nRet)
    {
        printf("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
        return false;
    }

    setPixelFormat(12);

    std::cout << "setPixelFormat(12)" << std::endl;

    // 设置帧率使能为Off
    nRet = MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", false);

    // get IInteger variable
    MVCC_INTVALUE stHeight = {0};
    nRet = MV_CC_GetIntValue(handle_, "Height", &stHeight);
    if (MV_OK == nRet)
    {
        image_height_ = stHeight.nCurValue; 
    }
    else
    {
        printf("get height failed! nRet [%x]\n\n", nRet);
    }

    MVCC_INTVALUE stwidth = {0};
    nRet = MV_CC_GetIntValue(handle_, "Width", &stwidth);
    if (MV_OK == nRet)
    {
        image_width_ = stwidth.nCurValue; 
    }
    else
    {
        printf("get height failed! nRet [%x]\n\n", nRet);
    }

    // 开始取流
    // start grab image
    if (!camera_is_opened_)
    {
        nRet = MV_CC_StartGrabbing(handle_);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            return false;
        }
        camera_is_opened_ = true;
    }


    camera_sn_ = sn;

    return true;
}

bool CameraMVS::closeCamera()
{

    int nRet = MV_OK;
    // 关闭设备
    // close device
    nRet = MV_CC_CloseDevice(handle_);
    if (MV_OK != nRet)
    {
        printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
    }

    // 销毁句柄
    // destroy handle
    nRet = MV_CC_DestroyHandle(handle_);
    if (MV_OK != nRet)
    {
        printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
        return false;
    }

    return false;
}

bool CameraMVS::streamOn()
{
    int i = 0;
    int nRet = MV_OK;

    printf("stream on");

    while(1)
    {
        nRet = MV_CC_GetOneFrameTimeout(handle_, pData_, nDataSize_, &stImageInfo_, 0);
        if (nRet != MV_OK)
        {
            std::cout << "noise iamges: " << i << std::endl;
            return true;
        }
        else
        {
            i += 1;
        }
    }
}

bool CameraMVS::streamOff()
{
    return true;
}

bool CameraMVS::grap(unsigned char *buf)
{
    int nRet = MV_OK;

    printf(("camera " + camera_sn_ + " captured!\n").c_str());

    nRet = MV_CC_GetOneFrameTimeout(handle_, pData_, nDataSize_, &stImageInfo_, 1000);
    if (nRet == MV_OK)
    {
        memcpy(buf, pData_, stImageInfo_.nHeight * stImageInfo_.nWidth);
    }
    else
    {
        printf("No data[%x]\n", nRet);

        return false;
    }

    return true;
}

bool CameraMVS::grap(unsigned short* buf)
{
    int nRet = MV_OK;

    printf(("camera " + camera_sn_ + " captured!\n").c_str());

    std::cout << "nDataSize_: " << nDataSize_ << std::endl;

    nRet = MV_CC_GetOneFrameTimeout(handle_, pData_, nDataSize_, &stImageInfo_, 1000);
    if (nRet == MV_OK)
    {
        memcpy(buf, pData_, nDataSize_);
    }
    else
    {
        printf("No data[%x]\n", nRet);

        return false;
    }

    return true;
}

bool CameraMVS::setPixelFormat(int val)
{
    int nRet = 0;

    if (camera_is_opened_)
    {
        nRet = MV_CC_StopGrabbing(handle_);
        if (nRet != MV_OK)
        {
            printf("Stop Grabbing fail! nRet [0x%x]\n", nRet);
            return false;
        }
        camera_is_opened_ = false;
    }

    switch (val)
    {
    case 8:
        nRet = MV_CC_SetEnumValue(handle_, "PixelFormat", MvGvspPixelType::PixelType_Gvsp_Mono8);
        if (MV_OK != nRet)
        {
            printf("Set Pixel Format fail! nRet [0x%x]\n", nRet);
            return false;
        }
        break;

    case 12:
        nRet = MV_CC_SetEnumValue(handle_, "PixelFormat", MvGvspPixelType::PixelType_Gvsp_Mono12);
        if (MV_OK != nRet)
        {
            printf("Set Pixel Format fail! nRet [0x%x]\n", nRet);
            return false;
        }
        break;   

    default:
        break;
    }

    if (!camera_is_opened_)
    {
        nRet = MV_CC_StartGrabbing(handle_);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            return false;
        }
        camera_is_opened_ = true;
    }

    // 设置完成参数之后应当对图片内存重新开辟；
    if (pData_ == NULL)
    {
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));

        nRet = MV_CC_GetIntValue(handle_, "PayloadSize", &stParam);
        if (MV_OK != nRet)
        {
            printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
            return false;
        }

        stImageInfo_ = {0};
        memset(&stImageInfo_, 0, sizeof(MV_FRAME_OUT_INFO_EX));

        std::cout << "malloc buffer: " << stParam.nCurValue << std::endl;
        pData_ = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
        nDataSize_ = stParam.nCurValue;
        if (NULL == pData_)
        {
            return false;
        }
    }
    else
    {
        free(pData_);

        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));

        nRet = MV_CC_GetIntValue(handle_, "PayloadSize", &stParam);
        if (MV_OK != nRet)
        {
            printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
            return false;
        }

        stImageInfo_ = {0};
        memset(&stImageInfo_, 0, sizeof(MV_FRAME_OUT_INFO_EX));

        std::cout << "malloc buffer: " << stParam.nCurValue << std::endl;
        pData_ = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
        nDataSize_ = stParam.nCurValue;
        if (NULL == pData_)
        {
            return false;
        }
    }


    return true;
}

bool CameraMVS::trigger_software()
{
    int nRet = MV_CC_SetCommandValue(handle_, "TriggerSoftware");
    if (MV_OK != nRet)
    {
        printf("failed in TriggerSoftware[%x]\n", nRet);
        return false;
    }

    return true;
}

bool CameraMVS::switchToInternalTriggerMode()
{
        // 设置触发模式为on
        // set trigger mode as on
        int nRet = MV_CC_SetEnumValue(handle_, "TriggerMode", 1);
        if (MV_OK != nRet)
        {
        printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
        return false;
        }
        // 设置触发源
        // set trigger source
        nRet = MV_CC_SetEnumValue(handle_, "TriggerSource", MV_TRIGGER_SOURCE_SOFTWARE);
        if (MV_OK != nRet)
        {
        printf("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
        return false;
        }

        return true;
}

bool CameraMVS::switchToExternalTriggerMode()
{
        // 设置触发模式为on
        // set trigger mode as on
        int nRet = MV_CC_SetEnumValue(handle_, "TriggerMode", 1);
        if (MV_OK != nRet)
        {
        printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
        return false;
        }
        // 设置触发源
        // set trigger source
        nRet = MV_CC_SetEnumValue(handle_, "TriggerSource", MV_TRIGGER_SOURCE_LINE0);
        if (MV_OK != nRet)
        {
        printf("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
        return false;
        }

        return true;
}

bool CameraMVS::getExposure(double &val)
{

        MVCC_FLOATVALUE stExposureTime = {0};
        int nRet = MV_CC_GetFloatValue(handle_, "ExposureTime", &stExposureTime);
        if (MV_OK == nRet)
        {
        val = stExposureTime.fCurValue;
        }
        else
        {
        printf("get exposure time failed! nRet [%x]\n\n", nRet);
        return false;
        }

        return true;
}

bool CameraMVS::setExposure(double val)
{

        int nRet = MV_CC_SetFloatValue(handle_, "ExposureTime", val);
        if (MV_OK != nRet)
        {
        printf("set exposure time failed! nRet [%x]\n\n", nRet);
        return false;
        }

        return true;
}

bool CameraMVS::getGain(double &value)
{
   MVCC_FLOATVALUE stGain = {0};
   int nRet = MV_CC_GetFloatValue(handle_, "Gain", &stGain);
   if (MV_OK == nRet)
   {
        value = stGain.fCurValue;   
   }
   else
   {
   printf("get gain failed! nRet [%x]\n\n", nRet);
   return false;
   }

   return true; 
}


bool CameraMVS::setGain(double value)
{     
    int nRet = MV_CC_SetFloatValue(handle_, "Gain", value);
    if (MV_OK != nRet)
    {
        printf("set gain failed! nRet [%x]\n\n", nRet);
        return false;
    }

    return true;
 
}



