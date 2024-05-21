#pragma once
#include "projector_fpga.h"
#include "easylogging++.h"
#include <unistd.h>
#define MAX_LASER_CURRENT 65535//32000
#define MIN_LASER_CURRENT 0
#define SCAN_ADDITION 32
#define SCAN_ADDITION_TIMES 2047
uint32_t MIN_SCANER_VOLTAGE = 0;
uint32_t MAX_SCANER_VOLTAGE = MIN_SCANER_VOLTAGE + SCAN_ADDITION * SCAN_ADDITION_TIMES;

ProjectorTemp::ProjectorTemp()
{

}

ProjectorTemp::~ProjectorTemp()
{

}

bool ProjectorTemp::init() {
    char *device = "/dev/spidev0.0";
	int ret = spi_open(&m_spi, device, 0, 10000000);
	if (ret < 0) {
		printf("SpiInit failed \n");
	}

	char str[512] = {0};
	spi_tostring(&m_spi, str, sizeof(str));
	printf("m_spi string : %s \n", str);

    m_RegSet.LaserCmd.reg = AXI_CMD_ADDR; // 结束振镜输出指令
    m_RegSet.LaserCmd.val = 4;

    m_RegSet.LaserFreq.reg = AXI_FREQ_ADDR; // 输出一个点需要的时间，单位是10ns
    m_RegSet.LaserFreq.val = ROLLING_US_PER_ROW * 100;//最低限度是220

    m_RegSet.RunTime.reg = AXI_TIME_ADDR; // 运行次数运行无数次
    m_RegSet.RunTime.val = 18; 

    m_RegSet.OutPoint.reg = AXI_OUT_ADDR; // 总共输出的点数
    m_RegSet.OutPoint.val = 0; // 由于寄存器大小限制，所以这里需要除以一个2

    m_RegSet.LaserCyclesNum.reg = AXI_CYCLES_NUM_ADDR;
    m_RegSet.LaserCyclesNum.val = 1;

    m_RegSet.BankSelect.reg = AXI_BANK_ADDR; // 0、1、2分别代表激光、振镜、脉冲数据
    m_RegSet.BankSelect.val = 0;

    m_RegSet.InnerAddr.reg = AXI_ADDR_ADDR; // 写入的地址从零开始
    m_RegSet.InnerAddr.val = 0;

    m_RegSet.InnerData.reg = AXI_DATA_ADDR; // 写入的数据内容
    m_RegSet.InnerData.val = 0;

    m_RegSet.LaserBrightness.reg = AXI_BRIGHTNESS_ADDR; // 激光数据为1时候的电流值
    m_RegSet.LaserBrightness.val = MAX_LASER_CURRENT;

    m_RegSet.LaserDarkness.reg = AXI_DARKNESS_ADDR; // 激光数据为0时候的电流值
    m_RegSet.LaserDarkness.val = 0;

    m_RegSet.LaserClosedCurrent.reg = AXI_CLOSED_CURRENT_ADDR; // 激光关断时候的电流值
    m_RegSet.LaserClosedCurrent.val = 0;

    camera_reg_setting();

    return true;
}

bool ProjectorTemp::project() 
{
    m_RegSet.LaserCmd.val = 1;
    FpgaSpi_Write(m_RegSet.LaserCmd.reg, m_RegSet.LaserCmd.val);

    return true;
}

bool ProjectorTemp::stopProject() 
{
    m_RegSet.LaserCmd.val = 4;
    FpgaSpi_Write(m_RegSet.LaserCmd.reg, m_RegSet.LaserCmd.val);

    return true;
}

bool ProjectorTemp::setLaserCurrent(int brightness)
{
    if (brightness > 1024 || brightness < 0)
    {
        return false;
    }

    int maxCurrent = MAX_LASER_CURRENT, minCurrent = MIN_LASER_CURRENT;

    m_RegSet.LaserBrightness.val = (maxCurrent - minCurrent) / 1024 * brightness + minCurrent;

    FpgaSpi_Write(m_RegSet.LaserBrightness.reg, m_RegSet.LaserBrightness.val);

    return true;
}

bool ProjectorTemp::setProjectorWorkingMode(int serial_num)
{
    int max_frame_num = 20;
    unsigned int max_buffer_length = max_frame_num * PERIOD_POINT;
    int* data = new int[max_buffer_length];
    int frame_num = 0;

    switch (serial_num)
    {
    case 0:
        generatePatternDataMode0(data);
        frame_num = 14;
        break;

    case 1:
        generatePatternDataMode1(data);
        frame_num = 18;
        break;

    default:
        break;
    }

    int data_ret[128 * 18];
    for (int count = 0; count < 18; count += 1)
    {
        for (int j = 0; j < 128; j += 1)
        {
            unsigned short single_data = 0;
            for (int k = 0; k < 16; k += 1)
            {
                if (data[count * 4096 + 2048 + j * 16 + k] != 0)
                {
                    single_data = (single_data << 1) + 1;
                    // std::cout << "1";
                }
                else
                {
                    single_data = (single_data << 1);
                    // std::cout << "0";
                }
            }
            data_ret[count * 128 + j] = single_data;
            // data_ret[count * 128 + j] = 0x8000;
            std::cout << data_ret[count * 128 + j] << std::endl;
        }
        data_ret[count * 128 + ((int)SCAN_ADDITION_TIMES - 1) / 16] = 0;
    }


    uint32_t Addr = 0x0000;
    m_RegSet.BankSelect.reg = 0x0030;
    m_RegSet.BankSelect.val = 0x01;
    FpgaSpi_Write(m_RegSet.BankSelect.reg, m_RegSet.BankSelect.val);
    FpgaSpi_Write(m_RegSet.InnerAddr.reg, Addr);
    FpgaSpi_SeqWrite(m_RegSet.InnerData.reg, (uint32_t*)data_ret, 128 * 18);

    return true;
}

bool ProjectorTemp::generatePatternDataMode0(int* data)
{
    std::vector<int> periods = { 1, 2, 4, 8, 16, 32, 64, 128, 256 };
    std::vector<cv::Mat> imgs = generateGrayCode(periods, 2048, 1);
    cv::Mat img_black(1, 2048, CV_8UC3, cv::Scalar(0, 0, 0));
    imgs.push_back(img_black);

    for (int frame = 0; frame < 14; frame += 1)
    {
        for (int i = 0; i < PERIOD_POINT; i++) {
            if (i < PERIOD_POINT / 2)
            {
                data[PERIOD_POINT * frame + i] = 0;
            }
            else if (imgs[frame].at<cv::Vec3b>(0, i - PERIOD_POINT / 2)[0] == 255)
            {
                data[PERIOD_POINT * frame + i] = 1;
            }
            else
            {
                data[PERIOD_POINT * frame + i] = 0;
            }
        }

        for (int i = 0; i < LASER_NUM_BUFFER * 2; i += 1)
        {
            if (i < LASER_NUM_BUFFER)
            {
                data[PERIOD_POINT * frame + PERIOD_POINT / 2 + i] = 0;
            }
        }

        data[PERIOD_POINT * frame + PERIOD_POINT - 1] = 0;
    }

    return true;
}

bool ProjectorTemp::generatePatternDataMode1(int* data)
{
    std::vector<cv::Mat> patterns;
    std::vector<int> periods = { 1, 2, 4, 8, 16, 32, 64, 128 };
    generateGrayCode(patterns, periods, 2048, 1);
    generateGrayCodeShift(patterns, 16, 2, 2048, 1);
    generateBlackWhitePattern(patterns, 2048, 1);
    cv::Mat img_black(1, 2048, CV_8UC3, cv::Scalar(0, 0, 0));
    patterns.push_back(img_black);

    for (int frame = 0; frame < 18; frame += 1)
    {
        for (int i = 0; i < PERIOD_POINT; i++) {
            if (i < PERIOD_POINT / 2)
            {
                data[PERIOD_POINT * frame + i] = 0;
            }
            else if (patterns[frame].at<cv::Vec3b>(0, i - PERIOD_POINT / 2)[0] == 255)
            {
                data[PERIOD_POINT * frame + i] = 1;
            }
            else
            {
                data[PERIOD_POINT * frame + i] = 0;
            }
        }

        for (int i = 0; i < LASER_NUM_BUFFER * 2; i += 1)
        {
            if (i < LASER_NUM_BUFFER)
            {
                data[PERIOD_POINT * frame + PERIOD_POINT / 2 + i] = 0;
            }

            data[PERIOD_POINT * frame + PERIOD_POINT - 1] = 0;
        }
    }

    return true;
}

bool ProjectorTemp::generateBlackWhitePattern(std::vector<cv::Mat>& input_output_patterns, int width, int height)
{
    cv::Mat pattern = cv::Mat(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    input_output_patterns.push_back(pattern);

    pattern = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
    input_output_patterns.push_back(pattern);

    return true;
}

bool ProjectorTemp::generateGrayCode(std::vector<cv::Mat>& input_output_patterns, std::vector<int> period, int width, int height)
{
    cv::Mat img(height, width, CV_8UC3, cv::Scalar::all(0));

    for (int k = 0; k < period.size(); ++k)
    {
        cv::Mat img(height, width, CV_8UC3);
        int flag = k == 0 ? 0 : 255;
        for (int i = 0; i < img.rows; ++i)
        {
            int inv_flag = 0;
            int bit_width = width / (period[k] * 2);
            int invert_num = 0;
            for (int j = 0; j < img.cols; ++j)
            {
                if (j % bit_width == 0)
                {
                    ++invert_num;
                    flag = 255 - flag;
                    invert_num = invert_num > 4 ? invert_num - 4 : invert_num;
                    inv_flag = invert_num > 2 ? 255 : 0;
                }
                img.at<cv::Vec3b>(i, j) = inv_flag == 255 ? cv::Vec3b(255 - flag, 255 - flag, 255 - flag) : cv::Vec3b(flag, flag, flag);
            }
        }
        input_output_patterns.push_back(img.clone());
    }

    return true;
}

std::vector<cv::Mat> ProjectorTemp::generateGrayCode(std::vector<int> period, int width, int height) {
    std::vector<cv::Mat> ret;
    if (period.empty()) return ret;
    cv::Mat img(height, width, CV_8UC3, cv::Scalar::all(0));

    std::vector<cv::Mat> temp_vec;

    for (int m = 0; m > -16; m -= 4) {//是frame数量
        for (int i = 0; i < img.rows; ++i) {
            int range = (img.cols + 7) / 8 + 1;
            for (int j = -3; j < range; ++j) {//j是周期数
                for (int n = 0; n < 8; ++n) {//n是线宽的数
                    int idx_x = 8 * j + m + n;
                    if (idx_x >= img.cols || idx_x < 0) continue;
                    img.at<cv::Vec3b>(i, idx_x) = !(j % 2 == 0) ? cv::Vec3b(255, 255, 255) : cv::Vec3b(0, 0, 0);
                }
            }
        }
        temp_vec.push_back(img.clone());
    }
    ret.push_back(temp_vec[3]);
    ret.push_back(temp_vec[0]);
    ret.push_back(temp_vec[1]);
    ret.push_back(temp_vec[2]);

    img = cv::Mat(height, width, CV_8UC3, cv::Scalar::all(0));
    ret.push_back(img.clone());
    img = cv::Mat(height, width, CV_8UC3, cv::Scalar::all(255));
    ret.push_back(img.clone());
    for (int k = 0; k < period.size(); ++k) {
        cv::Mat img(height, width, CV_8UC3);
        for (int i = 0; i < img.rows; ++i) {
            int flag = 255;
            int inv_flag = 0;
            int bit_width = width / (period[k] * 2);
            int invert_num = 0;
            for (int j = 0; j < img.cols; ++j) {
                if (j % bit_width == 0) {
                    ++invert_num;
                    flag = 255 - flag;
                    invert_num = invert_num > 4 ? invert_num - 4 : invert_num;
                    inv_flag = invert_num > 2 ? 255 : 0;
                }
                img.at<cv::Vec3b>(i, j) = inv_flag == 255 ? cv::Vec3b(255 - flag, 255 - flag, 255 - flag) : cv::Vec3b(flag, flag, flag);
            }
        }
        ret.push_back(img.clone());
    }
    img = cv::Mat(height, width, CV_8UC3, cv::Scalar::all(0));

    for (int m = 0; m > -16; m -= 4) {//是frame数量
        for (int i = 0; i < img.rows; ++i) {
            int range = (img.cols + 7) / 8 + 1;
            for (int j = -3; j < range; ++j) {//j是周期数
                for (int n = 0; n < 8; ++n) {//n是线宽的数
                    int idx_x = 8 * j + m + n;
                    if (idx_x >= img.cols || idx_x < 0) continue;
                    img.at<cv::Vec3b>(i, idx_x) = !(j % 2 == 0) ? cv::Vec3b(255, 255, 255) : cv::Vec3b(0, 0, 0);
                }
            }
        }
        ret.push_back(img.clone());
    }
    return ret;
}

// 生成出来的条纹是左移的
bool ProjectorTemp::generateGrayCodeShift(std::vector<cv::Mat>& input_output_pattern, int pattern_period, int shift_val, int width, int height)
{
    cv::Mat img = cv::Mat(height, width, CV_8UC3, cv::Scalar::all(0));

    for (int m = 0; m > -pattern_period; m -= shift_val) 
    {//是frame数量
        int shift_val_0 = m + shift_val;
        for (int i = 0; i < img.rows; ++i) 
        {
            int range = (img.cols + (pattern_period / 2) - 1) / (pattern_period / 2) + 1;
            for (int j = -3; j < range; ++j) 
            {//j是周期数
                for (int n = 0; n < (pattern_period / 2); ++n) 
                {//n是线宽的数
                    int idx_x = (pattern_period / 2) * j + shift_val_0 + n;
                    if (idx_x >= img.cols || idx_x < 0) continue;
                    img.at<cv::Vec3b>(i, idx_x) = !(j % 2 == 0) ? cv::Vec3b(255, 255, 255) : cv::Vec3b(0, 0, 0);
                }
            }
        }
        input_output_pattern.push_back(img.clone());
    }

    return true;
}

void ProjectorTemp::camera_reg_setting() {
    m_RegSet.LaserCmd.val = 4;
    FpgaSpi_Write(m_RegSet.LaserCmd.reg, m_RegSet.LaserCmd.val);

    char string[100] = { '\0' };
    sprintf(string, "LaserCmd addr = 0x%X, val = %d", m_RegSet.LaserCmd.reg, m_RegSet.LaserCmd.val);
    LOG(INFO) << string;
    sprintf(string, "LaserFreq addr = 0x%X, val = %d", m_RegSet.LaserFreq.reg, m_RegSet.LaserFreq.val);
    LOG(INFO) << string;
    sprintf(string, "RunTime addr = 0x%X, val = %d", m_RegSet.RunTime.reg, m_RegSet.RunTime.val);
    LOG(INFO) << string;
    sprintf(string, "OutPoint addr = 0x%X, val = %d", m_RegSet.OutPoint.reg, m_RegSet.OutPoint.val);
    LOG(INFO) << string;
    sprintf(string, "InnerAddr addr = 0x%X, val = %d", m_RegSet.InnerAddr.reg, m_RegSet.InnerAddr.val);
    LOG(INFO) << string;
    sprintf(string, "InnerData addr = 0x%X, val = %d", m_RegSet.InnerData.reg, m_RegSet.InnerData.val);
    LOG(INFO) << string;
    sprintf(string, "Trigger addr = 0x%X, val = %d", m_RegSet.Trigger.reg, m_RegSet.Trigger.val);
    LOG(INFO) << string;
    sprintf(string, "LaserBrightness addr = 0x%X, val = %d", m_RegSet.LaserBrightness.reg, m_RegSet.LaserBrightness.val);
    LOG(INFO) << string;
    sprintf(string, "LaserDarkness addr = 0x%X, val = %d", m_RegSet.LaserDarkness.reg, m_RegSet.LaserDarkness.val);
    LOG(INFO) << string;
    sprintf(string, "LaserClosedCurrent addr = 0x%X, val = %d", m_RegSet.LaserClosedCurrent.reg, m_RegSet.LaserClosedCurrent.val);
    LOG(INFO) << string;

    FpgaSpi_Write(m_RegSet.LaserFreq.reg, m_RegSet.LaserFreq.val);
    FpgaSpi_Write(m_RegSet.RunTime.reg, m_RegSet.RunTime.val);
    FpgaSpi_Write(m_RegSet.OutPoint.reg, m_RegSet.OutPoint.val);
    FpgaSpi_Write(m_RegSet.LaserBrightness.reg, m_RegSet.LaserBrightness.val);
    FpgaSpi_Write(m_RegSet.LaserDarkness.reg, m_RegSet.LaserDarkness.val);
    FpgaSpi_Write(m_RegSet.LaserClosedCurrent.reg, m_RegSet.LaserClosedCurrent.val);
    /****************************************************/
    FpgaSpi_Write((uint32_t)0x0012, (uint32_t)0);
    FpgaSpi_Write((uint32_t)0x0013, (uint32_t)18);
    FpgaSpi_Write((uint32_t)0x0014, (uint32_t)1);
    FpgaSpi_Write((uint32_t)0x0015, (uint32_t)2048);
    FpgaSpi_Write((uint32_t)0x0016, (uint32_t)SCAN_ADDITION_TIMES);
    FpgaSpi_Write((uint32_t)0x0017, (uint32_t)0);
    FpgaSpi_Write((uint32_t)0x0018, (uint32_t)2248);
    FpgaSpi_Write((uint32_t)0x0019, (uint32_t)MIN_SCANER_VOLTAGE);
    FpgaSpi_Write((uint32_t)0x001a, (uint32_t)32);

    // 计算扫描的增量
    FpgaSpi_Write((uint32_t)0x001b, (uint32_t)SCAN_ADDITION);

    LOG(INFO) << "init end";
}

uint8_t ProjectorTemp::reverse_bit(uint8_t x) 
{
	uint8_t b0 = (x >> 7) & 0x01;
	uint8_t b1 = (x >> 5) & 0x02;
	uint8_t b2 = (x >> 3) & 0x04;
	uint8_t b3 = (x >> 1) & 0x08;
	uint8_t b4 = (x << 1) & 0x10;
	uint8_t b5 = (x << 3) & 0x20;
	uint8_t b6 = (x << 5) & 0x40;
	uint8_t b7 = (x << 7) & 0x80;

	uint8_t byte = b7 | b6 | b5 | b4 | b3 | b2 | b1 | b0;

	return byte;
}

uint32_t ProjectorTemp::byte_order(uint32_t orgdata)
{
	uint8_t order[4];
	order[3] = (orgdata & 0xff000000) >> 24;
	order[2] = (orgdata & 0x00ff0000) >> 16;
	order[1] = (orgdata & 0x0000ff00) >> 8;
	order[0] = (orgdata & 0x000000ff) >> 0;

	uint8_t byte0, byte1, byte2, byte3;
	byte3 = reverse_bit(order[3]);
	byte2 = reverse_bit(order[2]);
	byte1 = reverse_bit(order[1]);
	byte0 = reverse_bit(order[0]);

	uint32_t dwdata = (byte3 << 24) | (byte2 << 16) | (byte1 << 8) | (byte0 << 0);

	return dwdata;
}

int ProjectorTemp::FpgaSpi_Write(uint32_t RegAddr, uint32_t data) {
    uint32_t WriteCmd = (SPI_WRITE_HEAD << 16) | RegAddr;
    uint32_t SendCmd = byte_order(WriteCmd);
//    printf("spiWtCmd:0x%08x, invertAddr:0x%08x\n", WriteCmd, SendCmd);

    int ret;
    ret = spi_write(&m_spi, (uint8_t *)&SendCmd, sizeof(uint32_t));
    if (ret < 0) 
    {
        LOG(INFO)<<"SPI SendCmd failed \n";
        return DF_FAILED;
    }

    uint32_t SendData = byte_order(data);

    ret = spi_write(&m_spi, (uint8_t *)&SendData, sizeof(uint32_t));
    if (ret < 0) 
    {
        LOG(INFO)<<"SPI SendData failed \n";
        return DF_FAILED;
    }

    return DF_SUCCESS;
}

int ProjectorTemp::FpgaSpi_SeqWrite(uint32_t RegAddr, uint32_t *txBuf, uint32_t len) {
    uint32_t WriteCmd = (SPI_WRITE_HEAD << 16) | RegAddr;
    uint32_t SendCmd = byte_order(WriteCmd);
//    printf("spiWtCmd:0x%08x, invertAddr:0x%08x\n", WriteCmd, SendCmd);

    int ret;
    ret = spi_write(&m_spi, (uint8_t *)&SendCmd, sizeof(uint32_t));
    if (ret < 0) 
    {
        LOG(INFO)<<"SPI SendCmd failed \n";
        return DF_FAILED;
    }

    uint32_t SendData;
    for (int i = 0; i < len; i++)
    {
        SendData = byte_order(txBuf[i]);
//	    printf("data:0x%08x, SendData:0x%08x\n", txBuf[i], SendData);

        int ret = spi_write(&m_spi, (uint8_t *)&SendData, sizeof(uint32_t));
        if (ret < 0) 
        {
            LOG(INFO)<<"FpgaSpi_SeqWrite failed \n";
            return DF_FAILED;
        }
    }

    return DF_SUCCESS;
}

int ProjectorTemp::FpgaSpi_Read(uint32_t RegAddr, uint32_t *data) {
    uint32_t WriteCmd = (SPI_READ_HEAD << 16) | RegAddr;
    uint32_t SendCmd = byte_order(WriteCmd);
//    printf("spiWtCmd:0x%08x, invertAddr:0x%08x\n", WriteCmd, SendCmd);

    int ret;
    ret = spi_write(&m_spi, (uint8_t *)&SendCmd, sizeof(uint32_t));
    if (ret < 0) {
        LOG(INFO)<<"SPI SendCmd failed \n";
        return DF_FAILED;
    }

    ret = spi_read(&m_spi, (uint8_t *)data, sizeof(uint32_t));
    *data = byte_order(*data);
    if (ret < 0) 
    {
        LOG(INFO)<<"SPI SendData failed \n";
        return DF_FAILED;
    }

    return DF_SUCCESS;

}

bool ProjectorTemp::setProjectorFrequency(int freq)
{
    m_RegSet.LaserFreq.val = freq;
    FpgaSpi_Write(m_RegSet.LaserFreq.reg, m_RegSet.LaserFreq.val);

    float scanner_roll_back_time_10ns = SCANNER_ROLL_BACK_TIME_MS * 1000 * 100;
    float roll_back_point_nums = scanner_roll_back_time_10ns / freq;
    int rool_back_step = (MAX_SCANER_VOLTAGE - MIN_SCANER_VOLTAGE) / roll_back_point_nums;
    int roll_back_point_nums_int = (MAX_SCANER_VOLTAGE - MIN_SCANER_VOLTAGE) / rool_back_step;

    std::cout << "rool_back_step = " << rool_back_step << std::endl;
    std::cout << "roll_back_point_nums_int = " << roll_back_point_nums_int << std::endl;
    FpgaSpi_Write(0x0015, roll_back_point_nums_int); // 准备阶段点数
    FpgaSpi_Write(0x0017, roll_back_point_nums_int); // 启动激光的点数
    FpgaSpi_Write(0x0018, roll_back_point_nums_int); // 触发相机点数
    FpgaSpi_Write(0x001a, rool_back_step); // 
    return true;
}

FpgaProjector::FpgaProjector()
{
    spi_projector_ori_ = new ProjectorTemp;

    trigger_offset_ = PERIOD_POINT / 2 + LASER_NUM_BUFFER + 30;

    projectorExposure_ = 10000;

    max_exposure_ = 100000;
    min_exposure_ = 10000;
}

FpgaProjector::~FpgaProjector()
{
    delete spi_projector_ori_;
    spi_projector_ori_ = NULL;
}

bool FpgaProjector::init()
{
    spi_projector_ori_->init();
    setProjectorExposure(projectorExposure_);
    setProjectorWorkingMode(1);

    return true;
}

bool FpgaProjector::project()
{
    if (spi_projector_ori_->project())
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool FpgaProjector::setProjectorCurrent(int current)
{
    if (spi_projector_ori_->setLaserCurrent(current))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool FpgaProjector::setProjectorExposure(int exposure)
{
    if (exposure < min_exposure_ || exposure > max_exposure_)
    {
        std::cout << "max exposure: " << max_exposure_ << std::endl;
        std::cout << "min exposure: " << min_exposure_ << std::endl;
        std::cout << "exposure: " << exposure << std::endl;
        std::cout << "error: projector exposure out of range!" << std::endl;
        return false;
    }
    // 将曝光时间转换为对应的出点周期
    float exposure_per_point_10ns = exposure * 100. / (PERIOD_POINT / 2.);

    projectorExposure_ = exposure;

    if (spi_projector_ori_->setProjectorFrequency(exposure_per_point_10ns))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool FpgaProjector::getProjectorExposure(int& exposure)
{
    exposure = projectorExposure_;

    return true;
}

bool FpgaProjector::setProjectorTriggerDlay(int dlay) // 这个dlay应当是直接设置一次就行，可以使用之前的offset
{
    // spi_projector_ori_->allFrameTriggerBeginIdx(trigger_offset_);

    return true;
}

bool FpgaProjector::setProjectorWorkingMode(int mode)
{
    bool ret = false;
    switch (mode)
    {
    case 0: // 共14张图：2张黑白图像 + 4张相移图像 + 8张格雷码图像
        ret = spi_projector_ori_->setProjectorWorkingMode(0);
        break;

    case 1: // 共18张图：2张黑白图像 + 8张相移图像 + 8张格雷码图像
        ret = spi_projector_ori_->setProjectorWorkingMode(1);
        break;
    
    default:
        break;
    }

    return ret;
}

bool FpgaProjector::getMinExposure(float& minExposure)
{
    minExposure = min_exposure_;

    return true;
}

bool FpgaProjector::getCorrectExposure(int& exposureTime)
{
    exposureTime = exposureTime;

    return true;
}

