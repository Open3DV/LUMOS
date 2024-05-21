#include "camera_mipi_virtual.h"

CameraMIPIVirtual::CameraMIPIVirtual()
{
    this->image_height_ = 1080;
    this->image_width_ = 1920;

    image_to_grap_ = cv::Mat(image_height_, image_width_, CV_8UC3, cv::Scalar(255, 255, 255));

}

bool CameraMIPIVirtual::openCamera()
{
    return true;
}

bool CameraMIPIVirtual::streamOn()
{
    return true;
}

bool CameraMIPIVirtual::streamOff()
{
    return true;
}

bool CameraMIPIVirtual::grap(unsigned char* buf)
{
    memcpy(buf, image_to_grap_.data, sizeof(unsigned char) * image_height_ * image_width_ * 3);

    return true;
}


