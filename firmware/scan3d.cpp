#include "scan3d.h"
#include "protocol.h"
#include "management.cuh"
#include <fstream>

//INITIALIZE_EASYLOGGINGPP
 
Scan3D::Scan3D()
{
    max_camera_exposure_ = 100000;
    min_camera_exposure_ = 0;

    camera_exposure_ = 10000.0;
    camera_gain_ = 0;

    generate_brightness_model_ = 2;
    generate_brightness_exposure_ = 5000;

    camera_opened_flag_ = false;

    camera_opened_ = false;
    camera_left_opened_ = false;
    camera_right_opened_ = false;

}

Scan3D::~Scan3D()
{

}

bool Scan3D::reboot_flag_enable(std::string file_path, int reboot_falg)
{
    std::ofstream ofs(file_path);

    if (ofs.is_open())
    {
        ofs << reboot_falg;
        ofs.close();
    }
    else
    {
        return false;
    }

    return true;

}

cudaStream_t stream1_, stream2_, stream3_, stream4_;

int Scan3D::init()
{
    cudaError_t result_e;

    result_e = cudaStreamCreate(&stream1_);
    result_e = cudaStreamCreate(&stream2_);
    result_e = cudaStreamCreate(&stream3_);
    result_e = cudaStreamCreate(&stream4_);
    int ret = 0;
    //激光振镜初始化
    projector_ = new FpgaProjector;
    ret = projector_->init();
    //projector_->setProjectorWorkingMode(0);

    if (true != ret)
    {
        LOG(ERROR) << "laser init FAILED";
        return false;
    }

    //相机初始化
    std::fstream sn_list;
    std::string sn_left;
    std::string sn_right;
    sn_list.open("./camera_sn.txt", std::ios::in);

    if (sn_list.is_open())
    {
        sn_list >> sn_left;
        sn_list >> sn_right;
        sn_list.close();
    }
    else
    {
        sn_list.close();
        sn_list.open("./camera_sn.txt", std::ios::out);
        LOG(ERROR) << "please edit the camera_sn.txt!";
        sn_list << std::string("please edit the camera_sn.txt!");
        sn_list.close();
    }

    LOG(INFO) << "camera list: \n" << "sn_left: " << sn_left << '\n' << "sn_right: " << sn_right;

    if (camera_opened_flag_ == false)
    {
        LOG(INFO) << "Open Camera:";
        camera_left_ = new CameraMVS();
        camera_right_ = new CameraMVS();
        if (!camera_left_->openCameraBySN(sn_left))
        {
            LOG(INFO) << "Open Left Camera Error!";
            camera_opened_flag_ = false;
            if (!camera_right_->openCameraBySN(sn_right))
            {
                LOG(INFO) << "Open Right Camera Error!";
                camera_opened_flag_ = false;
            }

            camera_opened_ = false;
            camera_left_opened_ = false;
            camera_right_opened_ = false;
            delete camera_left_;
            delete camera_right_;
            return false;
        }
        else if (!camera_right_->openCameraBySN(sn_right))
        {
            LOG(INFO) << "Open Right Camera Error!";
            camera_opened_flag_ = false;

            delete camera_left_;
            delete camera_right_;
            return false;
        }
        else
        {
            LOG(INFO) << "Open Camera:";
            camera_opened_flag_ = true;
        }
    }
    camera_rgb_ = new CameraMIPI();
    if (camera_rgb_->openCamera())
    {
        LOG(INFO) << "open rgb camera success!";
        if (!camera_rgb_->streamOn())
        {
            LOG(INFO) << "stream on rgb camera error";
            return false;
        }
        camera_rgb_->getImageSize(rgb_image_width_, rgb_image_height_);
        cv::Mat img_tepm(rgb_image_height_, rgb_image_width_, CV_8UC3);
        for (int i = 0; i < 10; i += 1)
        {
            camera_rgb_->grap(img_tepm.data);
            // cv::imshow("test", img_tepm);
            // cv::waitKey(0);
            LOG(INFO) << "grep rgb " << i << "th";
        }
        camera_rgb_->streamOff();

    }
    else
    {
        LOG(INFO) << "open rgb camera failed!";
        return false;
    }



    camera_left_->switchToExternalTriggerMode();   
    camera_right_->switchToExternalTriggerMode();
    //camera_->switchToInternalTriggerMode();
    LOG(INFO)<<"switchToExternalTriggerMode!"; 

    camera_left_->getImageSize(image_width_,image_height_);
    if (!camera_opened_flag_)
    {
        image_width_ = 1624;
        image_height_ = 1240;
    }
    int hdr_count;
    int exposure_time[5];
    int brightness[5];

    read_hdr_param_from_file("./HDR_params.xml", hdr_count, exposure_time, brightness);

    hdr_num_ = hdr_count;
    laser_current_ = 1023;
    laser_current_list_ = std::vector<int>(std::begin(brightness), std::end(brightness));
    camera_exposure_list_ = std::vector<int>(std::begin(exposure_time), std::end(exposure_time));

    float min_exposure = 0;
    camera_left_->getMinExposure(min_exposure);
    LOG(INFO)<<"scan3d min_exposure: "<<min_exposure; 
    min_camera_exposure_ = min_exposure;

    camera_left_->setExposure(camera_exposure_);
    camera_right_->setExposure(camera_exposure_);
    camera_left_->setGain(camera_gain_);
    camera_right_->setGain(camera_gain_);

    double exposure_temp;
    double gain_temp;
    camera_left_->getExposure(exposure_temp);
    camera_left_->getGain(gain_temp);
    LOG(INFO) << "scan3d camera_exposure_: " << exposure_temp;
    LOG(INFO) << "scan3d camera_gain_: " << gain_temp;

    buff_brightness_ = new unsigned char[image_width_*image_height_];
    buff_depth_ = new float[image_width_*image_height_];
    buff_pointcloud_ = new float[3*image_width_*image_height_];
    buff_color_brightness_ = new unsigned char[3*rgb_image_width_*rgb_image_height_];
    buff_depth_color_map_ = new unsigned short[2*image_width_*image_height_];
 

    if(0 == image_width_ && 0 == image_height_)
    {
        return false;
    }
    /******************************GPU init****************************/
    cuda_set_camera_resolution(image_width_, image_height_, rgb_image_width_, rgb_image_height_);
    cuda_malloc_basic_memory();

    /****************************gray to bin map***********************/
    unsigned char* gray_to_bin_map = new unsigned char[256];

    bool bin_code_val[8];
    bool gray_code_val[8];

    // index是格雷码的码值
    for (int i = 0; i < 256; i += 1)
    {
        // 十进制转二进制
        int val_temp = i;
        for (int j = 0; j < 8; j += 1)
        {
            gray_code_val[j] = val_temp >= pow(2, 7 - j);
            val_temp -= gray_code_val[j] ? pow(2, 7 - j) : 0;
        }
        grayCodeToBinCode(gray_code_val, bin_code_val);

        unsigned char bin_val = 0;

        for (int j = 0; j < 8; j += 1)
        {
            if (bin_code_val[j])
                bin_val += pow(2, 7 - j);
        }

        gray_to_bin_map[i] = 255 - bin_val;

    }

    // unsigned char decode_data[256] = {0, 1, 3, 7, 15, 31, 30, 62, 126, 254, 246, 247, 245, 213, 209, 145,
	// 	153, 152, 136, 8, 40, 42, 43, 35, 99, 103, 71, 70, 68, 76, 204, 220, 252, 253, 189, 185,
	// 	177, 179, 178, 146, 210, 82, 90, 91, 75, 107, 111, 109, 101, 100, 36, 164, 132, 134, 135,
	// 	143, 159, 155, 187, 186, 250, 242, 114, 112, 80, 81, 17, 21, 29, 13, 12, 44, 46, 174, 166,
	// 	167, 231, 199, 195, 193, 201, 200, 216, 88, 120, 56, 57, 49, 51, 55, 23, 22, 86, 94, 222,
	// 	206, 238, 239, 237, 233, 225, 161, 160, 128, 130, 2, 10, 11, 27, 59, 63, 127, 119, 118, 116,
	// 	244, 212, 148, 149, 157, 141, 137, 169, 168, 170, 162, 34, 98, 66, 67, 65, 69, 77, 93, 92,
	// 	124, 60, 188, 180, 181, 183, 151, 147, 211, 219, 218, 202, 74, 106, 104, 105, 97, 33, 37,
	// 	5, 4, 6, 14, 142, 158, 190, 191, 255, 251, 243, 241, 240, 208, 144, 16, 24, 25, 9, 41, 45,
	// 	47, 39, 38, 102, 230, 198, 196, 197, 205, 221, 217, 249, 248, 184, 176, 48, 50, 18, 19, 83,
	// 	87, 95, 79, 78, 110, 108, 236, 228, 229, 165, 133, 129, 131, 139, 138, 154, 26, 58, 122, 123,
	// 	115, 113, 117, 85, 84, 20, 28, 156, 140, 172, 173, 175, 171, 163, 227, 226, 194, 192, 64, 72,
	// 	73, 89, 121, 125, 61, 53, 52, 54, 182, 150, 214, 215, 223, 207, 203, 235, 234, 232, 224, 96, 32 };

    // unsigned char decode_data_upload[256];

    // for (int i = 0; i < 256; i += 1)
    // {
    //     decode_data_upload[decode_data[i]] = i;
    // }

    cuda_copy_decode_map_to_memory(gray_to_bin_map);

    delete[] gray_to_bin_map;



    /*****************************开辟cudaHost内存*************************/
    for (int i = 0; i < 18; i += 1)
    {
        cudaMallocHost(&host_img_left_[i], image_width_ * image_height_ * sizeof(unsigned short));
        cudaMallocHost(&host_img_right_[i], image_width_ * image_height_ * sizeof(unsigned short));
    }


    /*******************************生成查找表******************************/
    // 通过opencv生成查找表
    cv::Mat cameraMatrixL(3, 3, CV_64F);

	cv::Mat distCoeffL(1, 5, CV_64F);

	cv::Mat cameraMatrixR(3, 3, CV_64F);

	cv::Mat distCoeffR(1, 5, CV_64F);

    cv::Mat cameraMatrixRGB(3, 3, CV_64F);

	cv::Mat distCoeffRGB(1, 5, CV_64F);

	cv::Mat T(3, 1, CV_64F);

	cv::Mat RR(3, 3, CV_64F);

    cv::Mat T_RGB(3, 1, CV_64F);

	cv::Mat R_RGB(3, 3, CV_64F);

    if (!read_calib_param_from_file("./calib_param.xml", (double*)cameraMatrixL.data, (double*)cameraMatrixR.data, 
        (double*)cameraMatrixRGB.data, (double*)distCoeffL.data, (double*)distCoeffR.data, (double*)distCoeffRGB.data, 
            (double*)RR.data, (double*)R_RGB.data, (double*)T.data, (double*)T_RGB.data))
    {
        LOG(ERROR) << "failed to read calib_param.xml ";
        return false;
    }

	cv::Mat Rl, Rr, Pl, Pr, Q;
	cv::Rect roi1, roi2;
	cv::Mat mapL1, mapL2, mapR1, mapR2;
    cv::Mat weight_map;

    cv::stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, cv::Size(image_width_, image_height_), RR, 
        T, Rl, Rr, Pl, Pr, Q, /*cv::CALIB_ZERO_DISPARITY*/0, -1, cv::Size(image_width_, image_height_), &roi1, &roi2);

    cv::initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, cv::Size(image_width_, image_height_), CV_16SC2, mapL1, mapL2);
	cv::initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, cv::Size(image_width_, image_height_), CV_16SC2, mapR1, mapR2);

	cv::FileStorage fs;
	if (!fs.open("../weight.xml", cv::FileStorage::READ))
    {
        if (!fs.open("./weight.xml", cv::FileStorage::READ))
        {
            LOG(ERROR) << "failed to read weight.xml";
        }
    }

    if (fs.isOpened())
    {
        cv::FileNode fn = fs["weight"];
        fn >> weight_map;

        LOG(INFO) << "weight_map.size(): " << weight_map.size();

        cuda_copy_remap_maps_to_memory((short2*)mapL1.data, (short2*)mapR1.data, (unsigned short*)mapL2.data, 
            (unsigned short*)mapR2.data, (short4*)weight_map.data);
    }

    Q.convertTo(Q, CV_32F);

    /*********************************上传标定参数***************************/
    cameraMatrixRGB.convertTo(cameraMatrixRGB, CV_32F);
    R_RGB.convertTo(R_RGB, CV_32F);
    T_RGB.convertTo(T_RGB, CV_32F);

    LOG(INFO) << "cameraMatrixRGB" << cameraMatrixRGB;
    LOG(INFO) << "R_RGB" << R_RGB;
    LOG(INFO) << "T_RGB" << T_RGB;

    Rl.convertTo(Rl, CV_32F);
    R_RGB = R_RGB * Rl.inv();
    R_RGB.convertTo(R_RGB, CV_32F);

    cuda_copy_rgb_transform_to_memory((float*)cameraMatrixRGB.data, (float*)R_RGB.data, (float*)T_RGB.data);

    LOG(INFO) << "Q: " << Q;

    for (int i = 0; i < sizeof(calib_param_user_) / sizeof(float); i += 1)
    {
        calib_param_user_.camera_intrinsic[i] = 0;
    }

    calib_param_user_.camera_intrinsic[0] = Q.at<float>(2, 3);
    calib_param_user_.camera_intrinsic[4] = Q.at<float>(2, 3);
    calib_param_user_.camera_intrinsic[2] = -Q.at<float>(0, 3);
    calib_param_user_.camera_intrinsic[5] = -Q.at<float>(1, 3);
    calib_param_user_.camera_intrinsic[8] = 1;

    cameraMatrixRGB.convertTo(cameraMatrixRGB, CV_32F);
    distCoeffRGB.convertTo(distCoeffRGB, CV_32F);

    calib_param_user_.rgb_camera_intrinsic[0] = cameraMatrixRGB.at<float>(0, 0);
    calib_param_user_.rgb_camera_intrinsic[4] = cameraMatrixRGB.at<float>(1, 1);
    calib_param_user_.rgb_camera_intrinsic[2] = cameraMatrixRGB.at<float>(0, 2);//
    calib_param_user_.rgb_camera_intrinsic[5] = cameraMatrixRGB.at<float>(1, 2);
    calib_param_user_.rgb_camera_intrinsic[8] = 1;

    calib_param_user_.rgb_camera_distortion[0] = distCoeffRGB.at<float>(0, 0);
    calib_param_user_.rgb_camera_distortion[1] = distCoeffRGB.at<float>(0, 1);
    calib_param_user_.rgb_camera_distortion[2] = distCoeffRGB.at<float>(0, 2);
    calib_param_user_.rgb_camera_distortion[3] = distCoeffRGB.at<float>(0, 3);
    calib_param_user_.rgb_camera_distortion[4] = distCoeffRGB.at<float>(0, 4);

    calib_param_user_.translation_matrix[0] = T_RGB.at<float>(0, 0);
    calib_param_user_.translation_matrix[1] = T_RGB.at<float>(1, 0);
    calib_param_user_.translation_matrix[2] = T_RGB.at<float>(2, 0);

    float *R_RGB_ptr = R_RGB.ptr<float>(0, 0);
    for (int i = 0; i < 9; i += 1)
    {
        calib_param_user_.rotation_matrix[i] = R_RGB_ptr[i];
    }

    writeCalibParamUser();

    cuda_copy_Q_map_to_memory((float*)Q.data);

    setParamCameraGamma(0.5);
    
    LOG(INFO) << "init GPU end";

    setParamExposure(camera_exposure_);

    return true;
 
} 

bool Scan3D::grayCodeToBinCode(bool* gray_code, bool* bin_code) {


    bin_code[0] = gray_code[0];

    for (int i = 1; i < 8; ++i)
    {
        bool val = bin_code[i - 1] ^ gray_code[i];
        bin_code[i] = val;
    }

    int val_ = 0;
    for (int i = 0; i < 8; ++i)
    {
        if (bin_code[i])
            val_ += pow(2, 7 - i);
    }
    return true;
}

bool Scan3D::cameraIsValid()
{
    return camera_opened_flag_;
}

bool Scan3D::setParamConfidence(float confidence)

{
    return cuda_set_param_confidence(confidence);  
}

bool Scan3D::setParamExposure(float exposure)
{

    if(exposure > max_camera_exposure_ || exposure < min_camera_exposure_)
    {
        return false;
    }

    int int_exposure = exposure;

    projector_->getCorrectExposure(int_exposure);

    exposure = int_exposure;

    projector_->setProjectorExposure(exposure);


    if (!camera_left_->setExposure(exposure))
    {
        return false;
    }
    if (!camera_right_->setExposure(exposure))
    {
        return false;
    }
  
    camera_exposure_ = exposure;

    return true;
}

bool Scan3D::getParamExposure(float& exposure)
{
    double exposure_temp;
    if (!camera_left_->getExposure(exposure_temp))
    {
        return false;
    }

    exposure = exposure_temp;

    return true;
}

bool Scan3D::setParamGain(float gain)
{
    if (!camera_left_->setGain(gain))
    {
         return false;
    }
    if (!camera_right_->setGain(gain))
    {
         return false;
    }

    camera_gain_ = gain;
    
    return true;
}

bool Scan3D::getParamGain(float& gain)
{
    double gain_temp;

    if (!camera_left_->getGain(gain_temp))
    {
        return false;
    }

    LOG(INFO) << "gain_temp: " << gain_temp;

    gain = gain_temp;

    camera_gain_ = gain;

    return true;
}

bool Scan3D::setParamCameraGamma(float gamma)
{

    if (cuda_set_camera_gamma(gamma))
    {
        camera_gamma_ = gamma;
        return true;
    }
    return false;
    
}

bool Scan3D::getParamCameraGamma(float& gamma)
{
    if (cuda_get_camera_gamma(gamma))
    {
        camera_gamma_ = gamma;
        return true;
    }
    return false;
}

bool Scan3D::setParamGenerateBrightness(int model, int exposure)
{
    if (exposure > max_camera_exposure_ || exposure < min_camera_exposure_)
    {
        return false;
    }

    if (model == 1 || model == 2 || model == 3)
    {
        generate_brightness_model_ = model;
        generate_brightness_exposure_ = exposure;

        return true;
    }

    return false;
}

void Scan3D::setParamSystemConfig(SystemConfigDataStruct param)
{
    system_config_settings_machine_ = param;
}


bool Scan3D::captureTextureImage(int model,float exposure,unsigned char* buff)
{ 


    switch (model)
    {
        case 1:
        { 
            setParamExposure(exposure);
            camera_left_->switchToExternalTriggerMode();
            camera_left_->streamOn();
            LOG(INFO) << "Stream On"; 

            if(!camera_left_->grap(buff))
            { 
                 LOG(INFO) << "grap brightness failed!";
            }
            else
            {
                LOG(INFO) << "grap brightness!";
            }
            camera_left_->streamOff();
            LOG(INFO) << "Stream Off";
        }
        break;
        case 2:
        {
            LOG(INFO) << "sleep:";
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            LOG(INFO) << "end";

            camera_left_->switchToInternalTriggerMode();
            camera_left_->setExposure(exposure);
            camera_left_->streamOn();
            LOG(INFO) << "Stream On"; 

            if(!camera_left_->grap(buff))
            { 
                 LOG(INFO) << "grap brightness failed!";
            }
            else
            {
                LOG(INFO) << "grap brightness!";
            }

            
            camera_left_->streamOff();
            LOG(INFO) << "Stream Off";
            
            camera_left_->switchToExternalTriggerMode();
            camera_left_->setExposure(camera_exposure_);
        }
        break;
    case 3:
    {

            // lc3010_.stop_pattern_sequence(); 
            // lc3010_.init(); 

            camera_left_->switchToInternalTriggerMode(); 
            if(!camera_left_->setExposure(exposure))
            {
                LOG(INFO) << "setExposure Failed!";
            } 
            else
            {  
                LOG(INFO) << "set Exposure: "<<exposure;
            }
            camera_left_->streamOn();
            LOG(INFO) << "Stream On"; 
  
            if(!camera_left_->grap(buff))
            { 
                 LOG(INFO) << "grap generate brightness failed!";
            }
            else
            {
                LOG(INFO) << "grap generate brightness!";
            }

            
            camera_left_->streamOff();
            LOG(INFO) << "Stream Off";
            camera_left_->switchToExternalTriggerMode();
            camera_left_->setExposure(camera_exposure_);

    }
    break; 
    default:
        break;
    }

 

    return true;
}


bool Scan3D::captureRaw01(unsigned char* buff)
{
    projector_->setProjectorExposure(camera_exposure_);
    LOG(INFO) << "setPixelFormat(8)";

    camera_left_->setPixelFormat(8);
    camera_right_->setPixelFormat(8);

    if (!camera_left_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        camera_left_->streamOff();
        camera_right_->streamOff();
        camera_left_->setPixelFormat(12);
        camera_right_->setPixelFormat(12);
        return false;
    }
    if (!camera_right_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        camera_left_->streamOff();
        camera_right_->streamOff();
        camera_left_->setPixelFormat(12);
        camera_right_->setPixelFormat(12);
        return false;
    }



    projector_->project();

    int img_size = image_width_*image_height_;

    unsigned char *img_ptr_left= new unsigned char[image_width_*image_height_];
    unsigned char *img_ptr_right= new unsigned char[image_width_*image_height_];

    // for (int i = 0; i < 12; i += 1)
    // {
    //     if (!camera_left_->grap(img_ptr_left) || !camera_right_->grap(img_ptr_right))
    //     {
    //         camera_left_->streamOff();
    //         camera_right_->streamOff();
            
    //         return false;
    //     }
    // }

    for (int i = 0; i < 18; i++)
    {
        LOG(INFO)<<"grap "<<i<<" image:";
        if (!camera_left_->grap(img_ptr_left))
        {
            camera_left_->streamOff();
            camera_right_->streamOff();
            camera_left_->setPixelFormat(12);
            camera_right_->setPixelFormat(12);
            return false;
        }
        if (!camera_right_->grap(img_ptr_right))
        {
            camera_left_->streamOff();
            camera_right_->streamOff();
            camera_left_->setPixelFormat(12);
            camera_right_->setPixelFormat(12);
            return false;
        }
 
        memcpy(buff+img_size*i, img_ptr_left, img_size);
        memcpy(buff+img_size*(i+18), img_ptr_right, img_size);
    }

    camera_left_->streamOff();
    camera_right_->streamOff();

    //if (1 != generate_brightness_model_)
    //{
    //    captureTextureImage(generate_brightness_model_, generate_brightness_exposure_, img_ptr_left);
    //    memcpy(buff + img_size * 19, img_ptr_left, img_size);
    //}

    delete[] img_ptr_left;
    delete[] img_ptr_right;
    camera_left_->setPixelFormat(12);
    camera_right_->setPixelFormat(12);

    return true;
}

bool Scan3D::captureRaw01_16bit(unsigned short* buff)
{

    if (!camera_left_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        camera_left_->streamOff();
        camera_right_->streamOff();
        return false;
    }
    if (!camera_right_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        camera_left_->streamOff();
        camera_right_->streamOff();
        return false;
    }

    projector_->project();

    int img_size = image_width_*image_height_*sizeof(unsigned short);

    unsigned short *img_ptr_left= new unsigned short[img_size];
    unsigned short *img_ptr_right= new unsigned short[img_size];

    for (int i = 0; i < 18; i++)
    {
        LOG(INFO)<<"grap "<<i<<" image:";
        if (!camera_left_->grap(img_ptr_left))
        {
            camera_left_->streamOff();
            camera_right_->streamOff();
            return false;
        }
        if (!camera_right_->grap(img_ptr_right))
        {
            camera_left_->streamOff();
            camera_right_->streamOff();
            return false;
        }
 
        memcpy(buff+img_size*i, img_ptr_left, img_size);
        memcpy(buff+img_size*(i+18), img_ptr_right, img_size);
  
    }

    camera_left_->streamOff();
    camera_right_->streamOff();

    delete[] img_ptr_left;
    delete[] img_ptr_right;

    return true;
}


bool Scan3D::captureRaw03(unsigned char* buff)
{
    projector_->setProjectorExposure(camera_exposure_);
    LOG(INFO) << "setPixelFormat(8)";

    camera_left_->setPixelFormat(8);
    camera_right_->setPixelFormat(8);

    if (!camera_left_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        camera_rgb_->streamOff();
        camera_left_->streamOff();
        camera_right_->streamOff();
        camera_left_->setPixelFormat(12);
        camera_right_->setPixelFormat(12);
        return false;
    }
    if (!camera_right_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        camera_rgb_->streamOff();
        camera_left_->streamOff();
        camera_right_->streamOff();
        camera_left_->setPixelFormat(12);
        camera_right_->setPixelFormat(12);
        return false;
    }


    int img_size = image_width_*image_height_;

    unsigned char *img_ptr_left= new unsigned char[image_width_*image_height_];
    unsigned char *img_ptr_right= new unsigned char[image_width_*image_height_];

    projector_->project();


    // for (int i = 0; i < 12; i += 1)
    // {
    //     if (!camera_left_->grap(img_ptr_left) || !camera_right_->grap(img_ptr_right))
    //     {
    //         camera_left_->streamOff();
    //         camera_right_->streamOff();
            
    //         return false;
    //     }
    // }

    for (int i = 0; i < 18; i++)
    {
        LOG(INFO)<<"grap "<<i<<" image:";
        if (!camera_left_->grap(img_ptr_left))
        {
            camera_left_->streamOff();
            camera_right_->streamOff();
            camera_left_->setPixelFormat(12);
            camera_right_->setPixelFormat(12);
            return false;
        }
        if (!camera_right_->grap(img_ptr_right))
        {
            camera_left_->streamOff();
            camera_right_->streamOff();
            camera_left_->setPixelFormat(12);
            camera_right_->setPixelFormat(12);
            return false;
        }
 
        memcpy(buff+img_size*i, img_ptr_left, img_size);
        memcpy(buff+img_size*(i+18), img_ptr_right, img_size);
  
    }
    
    if (camera_rgb_->streamOn())
    {
        camera_rgb_->grap(buff + (36 * img_size));
        camera_rgb_->grap(buff + (36 * img_size));
        camera_rgb_->streamOff();
    }

    camera_left_->streamOff();
    camera_right_->streamOff();

    //if (1 != generate_brightness_model_)
    //{
    //    captureTextureImage(generate_brightness_model_, generate_brightness_exposure_, img_ptr_left);
    //    memcpy(buff + img_size * 19, img_ptr_left, img_size);
    //}

    delete[] img_ptr_left;
    delete[] img_ptr_right;
    camera_left_->setPixelFormat(12);
    camera_right_->setPixelFormat(12);

    return true;
}

std::vector<cv::Mat> generateGrayCodeTest(std::vector<int> period, int width, int height) {
    std::vector<cv::Mat> ret;
    if (period.empty()) return ret;
    cv::Mat img(height, width, CV_8UC3, cv::Scalar::all(0));
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

    //for (int m = 0; m < 4; ++m) {
    //    for (int i = 0; i < img.rows; ++i) {
    //        int range = (img.cols + 3) / 4;
    //        for (int j = -3; j < range; ++j) {
    //            for (int n = 0; n < 4; ++n) {
    //                int idx_x = 4 * j + m + n;
    //                if (idx_x >= img.cols || idx_x < 0) continue;
    //                img.at<cv::Vec3b>(i, 4 * j + m + n) = (j % 2 == 0) ? cv::Vec3b(255, 255, 255) : cv::Vec3b(0, 0, 0);
    //            }
    //        }
    //    }
    //    ret.push_back(img.clone());
    //    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    //    img = 255 - img;
    //    cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    //    ret.push_back(img.clone());
    //}

    for (int m = 0; m < 8; ++m) {
        for (int i = 0; i < img.rows; ++i) {
            int range = (img.cols + 7) / 8 + 1;
            for (int j = -3; j < range; ++j) {
                for (int n = 0; n < 8; ++n) {
                    int idx_x = 8 * j + m + n;
                    if (idx_x >= img.cols || idx_x < 0) continue;
                    img.at<cv::Vec3b>(i, idx_x) = (j % 2 == 0) ? cv::Vec3b(255, 255, 255) : cv::Vec3b(0, 0, 0);
                }
            }
        }
        ret.push_back(img.clone());
        //cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
        //img = 255 - img;
        //cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
        //ret.push_back(img.clone());
    }
    return ret;
}

int Scan3D::captureFrame01()
{
    projector_->setProjectorExposure(camera_exposure_);
    LOG(INFO) << "start init basic memory";
    cuda_init_basic_memory();
    LOG(INFO) << "finish init basic memory";

    LOG(INFO) << "Stream On:";
    if (!camera_left_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        camera_left_->streamOff();
        camera_right_->streamOff();
        return DF_ERROR_CAMERA_STREAM;
    }
    if (!camera_right_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        camera_left_->streamOff();
        camera_right_->streamOff();
        return DF_ERROR_CAMERA_STREAM;
    }

    /****************************cudaStreamCreate***********************/

    cudaError_t result_e;
    
    LOG(INFO) << "projector start";
    projector_->project();

    LOG(INFO) << "grap start";


    for (int j = 0; j < 14; j += 1)
    {

        int i = 0;
        if (j < 8)
        {
            i = j + 6;
        }
        else
        {
            i = j - 8;
        }

        LOG(INFO) << "grap " << i << " image:"; 

        if (!camera_left_->grap(host_img_left_[i]) || !camera_right_->grap(host_img_right_[i]))
        {
            camera_left_->streamOff();
            camera_right_->streamOff();
            
            return DF_ERROR_CAMERA_GRAP;
        }

        LOG(INFO) << "finished!";

        cudaStream_t streamNowLeft = i % 2 == 0 ? stream1_ : stream2_;
        cudaStream_t streamNowRight = i % 2 == 0 ? stream3_ : stream4_;

        if (i < 4)
        {
            streamNowLeft = stream2_;
            streamNowRight = stream4_;
        }
        streamNowRight = streamNowLeft;

        cuda_copy_pattern_to_memory(host_img_left_[i], i, streamNowLeft);
        cuda_copy_pattern_to_memory(host_img_right_[i], i + MAX_PATTERNS_NUMBER, streamNowRight);
    }

    cudaDeviceSynchronize();
    for (int i = 0; i < 14; i += 1)
    {
        cudaDeviceSynchronize();
        LOG(INFO) << "grap " << i << " image:"; 
        LOG(INFO) << "finished!";

        cudaStream_t streamNowLeft = i % 2 == 0 ? stream1_ : stream2_;
        cudaStream_t streamNowRight = i % 2 == 0 ? stream3_ : stream4_;

        if (i < 4)
        {
            streamNowLeft = stream2_;
            streamNowRight = stream4_;
        }
        streamNowRight = streamNowLeft;

        if (i == 3)
        {
            // 计算四步相移
            cuda_four_step_phase_shift_16bit(streamNowLeft, streamNowRight);
            continue;
        }

        if (i == 5)
        {
            // 计算threshold
            cuda_decode_gray_code_one_by_one_16bit(-1, streamNowLeft, streamNowRight);
            continue;
        }

        if (i > 5)
        {
            // 格雷码的移位解码
            cuda_decode_gray_code_one_by_one_16bit(i - 6, streamNowLeft, streamNowRight);
            if (i == 13)
            {
                cuda_code_phase_rectify(streamNowLeft, streamNowRight);
            }
            continue;
        }
    }
    
    camera_left_->streamOff();
    camera_right_->streamOff();
    
    cudaDeviceSynchronize();

    LOG(INFO) << "grap end";

    LOG(INFO) << "sort start";

    cuda_code_phase_unwrap(0);//展开

    cuda_fix_four_step_code_shift(0);

    cuda_code_statistics(0);
    cuda_code_statistics_to_index(0);
    cuda_pixels_sort_by_code(0);
    cuda_code_statistics_to_index(0);
    cuda_pixels_shear_by_monotonicity(0);
    cuda_matching(0);
    cuda_disp_to_depth(0);

    cudaDeviceSynchronize();

    LOG(INFO) << "sort end";



    LOG(INFO) << "Stream Off";
    
    removeOutlierBaseDepthFilter();
    removeOutlierBaseRadiusFilter();
    //if (1 != generate_brightness_model_)
    //{
    //    captureTextureImage(generate_brightness_model_, generate_brightness_exposure_,buff_brightness_);
    //}


    cuda_copy_depth_from_memory(buff_depth_);
    cuda_copy_brightness_from_memory(buff_brightness_);

    return DF_SUCCESS;
}

int Scan3D::captureFrame04()
{
    if (!camera_rgb_->streamOn())
    {
        LOG(INFO) << "rgb camera stream on error! ";
        camera_rgb_->streamOff();
        return DF_ERROR_CAMERA_STREAM;
    }
    projector_->setProjectorExposure(camera_exposure_);
    LOG(INFO) << "start init basic memory";
    cuda_init_basic_memory();
    LOG(INFO) << "finish init basic memory";

    LOG(INFO) << "Stream On:";
    if (!camera_left_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        camera_left_->streamOff();
        camera_right_->streamOff();
        return DF_ERROR_CAMERA_STREAM;
    }
    if (!camera_right_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        camera_left_->streamOff();
        camera_right_->streamOff();
        return DF_ERROR_CAMERA_STREAM;
    }

    /****************************cudaStreamCreate***********************/

    cudaError_t result_e;

    
    LOG(INFO) << "projector start";
    projector_->project();

    LOG(INFO) << "grap start";

    for (int j = 0; j < 18; j += 1)
    {

        int i = 0;
        if (j < 8)
        {
            i = j + 6;
        }
        else if (j < 12)
        {
            i = j - 8;
        }
        else if (j < 16)
        {
            i = j + 2;
        }
        else
        {
            i = j - 12;
        }


        LOG(INFO) << "grap " << i << " image:"; 

        if (!camera_left_->grap(host_img_left_[i]) || !camera_right_->grap(host_img_right_[i]))
        {
            camera_left_->streamOff();
            camera_right_->streamOff();
            
            return DF_ERROR_CAMERA_GRAP;
        }

        LOG(INFO) << "finished!";

        cudaStream_t streamNowLeft = i % 2 == 0 ? stream1_ : stream2_;
        cudaStream_t streamNowRight = i % 2 == 0 ? stream3_ : stream4_;

        if (i < 4)
        {
            streamNowLeft = stream2_;
            streamNowRight = stream4_;
        }
        streamNowRight = streamNowLeft;

        cuda_copy_pattern_to_memory(host_img_left_[i], i, streamNowLeft);
        cuda_copy_pattern_to_memory(host_img_right_[i], i + MAX_PATTERNS_NUMBER, streamNowRight);
    }

    cudaDeviceSynchronize();
    for (int i = 0; i < 18; i += 1)
    {
        cudaDeviceSynchronize();
        LOG(INFO) << "grap " << i << " image:"; 
        LOG(INFO) << "finished!";

        cudaStream_t streamNowLeft = i % 2 == 0 ? stream1_ : stream2_;
        cudaStream_t streamNowRight = i % 2 == 0 ? stream3_ : stream4_;

        if (i < 4)
        {
            streamNowLeft = stream2_;
            streamNowRight = stream4_;
        }
        streamNowRight = streamNowLeft;

        if (i == 3)
        {
            // 计算四步相移
            cuda_eight_step_phase_shift_16bit(streamNowLeft, streamNowRight);
            continue;
        }

        if (i == 5)
        {
            // 计算threshold
            cuda_decode_gray_code_one_by_one_16bit(-1, streamNowLeft, streamNowRight);
            continue;
        }

        if (i > 5)
        {
            // 格雷码的移位解码
            cuda_decode_gray_code_one_by_one_16bit(i - 6, streamNowLeft, streamNowRight);
            if (i == 13)
            {
                cuda_code_phase_rectify(streamNowLeft, streamNowRight);
            }
            continue;
        }
    }
    
    camera_left_->streamOff();
    camera_right_->streamOff();

    camera_rgb_->grap(buff_color_brightness_);
    camera_rgb_->grap(buff_color_brightness_);
    camera_rgb_->streamOff();
    
    cudaDeviceSynchronize();

    LOG(INFO) << "grap end";

    LOG(INFO) << "sort start";

    cuda_code_phase_unwrap(0);//展开

    // cuda_fix_four_step_code_shift(0);

    cuda_code_statistics(0);
    cuda_code_statistics_to_index(0);
    cuda_pixels_sort_by_code(0);
    cuda_code_statistics_to_index(0);
    cuda_pixels_shear_by_monotonicity(0);
    cuda_matching(0);
    cuda_disp_to_depth(0);

    cudaDeviceSynchronize();

    LOG(INFO) << "sort end";



    LOG(INFO) << "Stream Off";
    
    removeOutlierBaseDepthFilter();
    removeOutlierBaseRadiusFilter();
    //if (1 != generate_brightness_model_)
    //{
    //    captureTextureImage(generate_brightness_model_, generate_brightness_exposure_,buff_brightness_);
    //}


    cuda_copy_depth_from_memory(buff_depth_);
    cuda_copy_brightness_from_memory(buff_brightness_);


    return DF_SUCCESS;
}

int Scan3D::captureFrame05()
{
    if (!camera_rgb_->streamOn())
    {
        LOG(INFO) << "rgb camera stream on error! ";
        camera_rgb_->streamOff();
        return DF_ERROR_CAMERA_STREAM;
    }
    projector_->setProjectorExposure(camera_exposure_);
    LOG(INFO) << "start init basic memory";
    cuda_init_basic_memory();
    LOG(INFO) << "finish init basic memory";

    LOG(INFO) << "Stream On:";
    if (!camera_left_->streamOn())
    {
        LOG(INFO) << "camera_left Stream On Error";
        camera_left_->streamOff();
        camera_right_->streamOff();
        return DF_ERROR_CAMERA_STREAM;
    }
    if (!camera_right_->streamOn())
    {
        LOG(INFO) << "camera_right Stream On Error";
        camera_left_->streamOff();
        camera_right_->streamOff();
        return DF_ERROR_CAMERA_STREAM;
    }

    /****************************cudaStreamCreate***********************/

    cudaError_t result_e;

    
    LOG(INFO) << "projector start";
    projector_->project();

    LOG(INFO) << "grap start";

    for (int j = 0; j < 18; j += 1)
    {

        int i = 0;
        if (j < 8)
        {
            i = j + 6;
        }
        else if (j < 12)
        {
            i = j - 8;
        }
        else if (j < 16)
        {
            i = j + 2;
        }
        else
        {
            i = j - 12;
        }


        LOG(INFO) << "grap " << i << " image:"; 

        if (!camera_left_->grap(host_img_left_[i]) || !camera_right_->grap(host_img_right_[i]))
        {
            camera_left_->streamOff();
            camera_right_->streamOff();
            
            return DF_ERROR_CAMERA_GRAP;
        }

        LOG(INFO) << "finished!";

        cudaStream_t streamNowLeft = i % 2 == 0 ? stream1_ : stream2_;
        cudaStream_t streamNowRight = i % 2 == 0 ? stream3_ : stream4_;

        if (i < 4)
        {
            streamNowLeft = stream2_;
            streamNowRight = stream4_;
        }
        streamNowRight = streamNowLeft;

        cuda_copy_pattern_to_memory(host_img_left_[i], i, streamNowLeft);
        cuda_copy_pattern_to_memory(host_img_right_[i], i + MAX_PATTERNS_NUMBER, streamNowRight);
    }

    cudaDeviceSynchronize();
    for (int i = 0; i < 18; i += 1)
    {
        cudaDeviceSynchronize();
        LOG(INFO) << "grap " << i << " image:"; 
        LOG(INFO) << "finished!";

        cudaStream_t streamNowLeft = i % 2 == 0 ? stream1_ : stream2_;
        cudaStream_t streamNowRight = i % 2 == 0 ? stream3_ : stream4_;

        if (i < 4)
        {
            streamNowLeft = stream2_;
            streamNowRight = stream4_;
        }
        streamNowRight = streamNowLeft;

        if (i == 3)
        {
            // 计算四步相移
            cuda_eight_step_phase_shift_16bit(streamNowLeft, streamNowRight);
            continue;
        }

        if (i == 5)
        {
            // 计算threshold
            cuda_decode_gray_code_one_by_one_16bit(-1, streamNowLeft, streamNowRight);
            continue;
        }

        if (i > 5)
        {
            // 格雷码的移位解码
            cuda_decode_gray_code_one_by_one_16bit(i - 6, streamNowLeft, streamNowRight);
            if (i == 13)
            {
                cuda_code_phase_rectify(streamNowLeft, streamNowRight);
            }
            continue;
        }
    }
    
    camera_left_->streamOff();
    camera_right_->streamOff();

    camera_rgb_->grap(buff_color_brightness_);
    camera_rgb_->grap(buff_color_brightness_);
    camera_rgb_->streamOff();

    cv::Mat rgbImageTemp(rgb_image_height_, rgb_image_width_, CV_8UC3, buff_color_brightness_);
    cv::Mat resizeTemp;
    cv::resize(rgbImageTemp, resizeTemp, cv::Size(rgbImageTemp.cols / 2, rgbImageTemp.rows / 2));
    // resizeTemp.copyTo(rgbImageTemp);
    memcpy(buff_color_brightness_, resizeTemp.data, rgb_image_height_ * rgb_image_width_ * 3 / 4);
    
    cudaDeviceSynchronize();

    LOG(INFO) << "grap end";

    LOG(INFO) << "sort start";

    cuda_code_phase_unwrap(0);//展开

    // cuda_fix_four_step_code_shift(0);

    cuda_code_statistics(0);
    cuda_code_statistics_to_index(0);
    cuda_pixels_sort_by_code(0);
    cuda_code_statistics_to_index(0);
    cuda_pixels_shear_by_monotonicity(0);
    cuda_matching(0);
    cuda_disp_to_depth(0);

    cudaDeviceSynchronize();

    LOG(INFO) << "sort end";



    LOG(INFO) << "Stream Off";
    
    removeOutlierBaseDepthFilter();
    removeOutlierBaseRadiusFilter();
    //if (1 != generate_brightness_model_)
    //{
    //    captureTextureImage(generate_brightness_model_, generate_brightness_exposure_,buff_brightness_);
    //}


    cuda_copy_depth_from_memory(buff_depth_);
    cuda_copy_brightness_from_memory(buff_brightness_);


    return DF_SUCCESS;
}

int Scan3D::captureColorBrightness()
{
    if (!camera_rgb_->streamOn())
    {
        LOG(INFO) << "rgb camera stream on error! ";
        camera_rgb_->streamOff();
        return DF_ERROR_CAMERA_STREAM;
    }
    cuda_init_basic_memory();
    LOG(INFO) << "finish init basic memory";

    camera_rgb_->grap(buff_color_brightness_);
    camera_rgb_->grap(buff_color_brightness_);
    camera_rgb_->streamOff();
    LOG(INFO) << "Stream Off";

    cv::Mat rgbImageTemp(rgb_image_height_, rgb_image_width_, CV_8UC3, buff_color_brightness_);
    cv::Mat resizeTemp;
    cv::resize(rgbImageTemp, resizeTemp, cv::Size(rgbImageTemp.cols / 2, rgbImageTemp.rows / 2));
    // resizeTemp.copyTo(rgbImageTemp);
    memcpy(buff_color_brightness_, resizeTemp.data, rgb_image_height_ * rgb_image_width_ * 3 / 4);

    return DF_SUCCESS;
}

int Scan3D::captureFrame08()
{
    // projector_->setProjectorWorkingMode(1);

    // sleep(1);

    projector_->setProjectorExposure(camera_exposure_);
    LOG(INFO) << "start init basic memory";
    cuda_init_basic_memory();
    LOG(INFO) << "finish init basic memory";

    LOG(INFO) << "Stream On:";
    if (!camera_left_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        camera_left_->streamOff();
        camera_right_->streamOff();
        return DF_ERROR_CAMERA_STREAM;
    }
    if (!camera_right_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        camera_left_->streamOff();
        camera_right_->streamOff();
        return DF_ERROR_CAMERA_STREAM;
    }

    /****************************cudaStreamCreate***********************/

    cudaError_t result_e;

    LOG(INFO) << "projector start";
    projector_->project();

    LOG(INFO) << "grap start";

    for (int j = 0; j < 18; j += 1)
    {

        int i = 0;
        if (j < 8)
        {
            i = j + 6;
        }
        else if (j < 12)
        {
            i = j - 8;
        }
        else if (j < 16)
        {
            i = j + 2;
        }
        else
        {
            i = j - 12;
        }

        LOG(INFO) << "grap " << i << " image:"; 

        if (!camera_left_->grap(host_img_left_[i]) || !camera_right_->grap(host_img_right_[i]))
        {
            camera_left_->streamOff();
            camera_right_->streamOff();
            return DF_ERROR_CAMERA_GRAP;
        }

        LOG(INFO) << "finished!";

        cudaStream_t streamNowLeft = i % 2 == 0 ? stream1_ : stream2_;
        cudaStream_t streamNowRight = i % 2 == 0 ? stream3_ : stream4_;

        if (i < 4)
        {
            streamNowLeft = stream2_;
            streamNowRight = stream4_;
        }
        streamNowRight = streamNowLeft;

        cuda_copy_pattern_to_memory(host_img_left_[i], i, streamNowLeft);
        cuda_copy_pattern_to_memory(host_img_right_[i], i + MAX_PATTERNS_NUMBER, streamNowRight);
    }

    cudaDeviceSynchronize();
    for (int i = 0; i < 18; i += 1)
    {
        cudaDeviceSynchronize();
        LOG(INFO) << "grap " << i << " image:"; 
        LOG(INFO) << "finished!";

        cudaStream_t streamNowLeft = i % 2 == 0 ? stream1_ : stream2_;
        cudaStream_t streamNowRight = i % 2 == 0 ? stream3_ : stream4_;

        if (i < 4)
        {
            streamNowLeft = stream2_;
            streamNowRight = stream4_;
        }
        streamNowRight = streamNowLeft;

        if (i == 4)
        {
            // 计算八步相移
            cuda_eight_step_phase_shift_16bit(streamNowLeft, streamNowRight);
            continue;
        }

        if (i == 5)
        {
            // 计算threshold
            cuda_decode_gray_code_one_by_one_16bit(-1, streamNowLeft, streamNowRight);
            continue;
        }

        if (i > 5)
        {
            // 格雷码的移位解码
            cuda_decode_gray_code_one_by_one_16bit(i - 6, streamNowLeft, streamNowRight);
            if (i == 13)
            {
                cuda_code_phase_rectify(streamNowLeft, streamNowRight);
                break;
            }
            continue;
        }
    }
    
    camera_left_->streamOff();
    camera_right_->streamOff();

    // if (!camera_rgb_->streamOn())
    // {
    //     LOG(INFO) << "rgb camera stream on error! ";
    //     camera_rgb_->streamOff();
    // }
    // else
    // {
    //     camera_rgb_->grap(buff_color_brightness_);
    //     camera_rgb_->grap(buff_color_brightness_);
    //     camera_rgb_->streamOff();
    // }
    cudaDeviceSynchronize();

    LOG(INFO) << "grap end";

    LOG(INFO) << "sort start";

    cuda_code_phase_unwrap(0);//展开

    // cuda_fix_eight_step_code_shift(0);

    cuda_code_statistics(0);
    cuda_code_statistics_to_index(0);
    cuda_pixels_sort_by_code(0);
    cuda_code_statistics_to_index(0);
    cuda_pixels_shear_by_monotonicity(0);
    cuda_matching(0);
    cuda_disp_to_depth(0);

    cudaDeviceSynchronize();

    LOG(INFO) << "sort end";



    LOG(INFO) << "Stream Off";
    
    removeOutlierBaseDepthFilter();
    removeOutlierBaseRadiusFilter();
    //if (1 != generate_brightness_model_)
    //{
    //    captureTextureImage(generate_brightness_model_, generate_brightness_exposure_,buff_brightness_);
    //}


    cuda_copy_depth_from_memory(buff_depth_);
    cuda_copy_brightness_from_memory(buff_brightness_);

    return DF_SUCCESS;
}

int Scan3D::captureFrame02()
{
    LOG(INFO) << "start init basic memory";
    cuda_init_basic_memory();
    LOG(INFO) << "finish init basic memory";

    LOG(INFO) << "Stream On:";
    if (!camera_left_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        return DF_ERROR_CAMERA_STREAM;
    }
    if (!camera_right_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        return DF_ERROR_CAMERA_STREAM;
    }

    /****************************cudaStreamCreate***********************/

    cudaError_t result_e;
    
    unsigned char* test_left = new unsigned char[image_width_ * image_height_];
    unsigned char* test_right = new unsigned char[image_width_ * image_height_];

    //projector_->setProjectorWorkingMode(0);

    LOG(INFO) << "projector start";
    projector_->project();

    LOG(INFO) << "grap start";

    for (int j = 0; j < 14; j += 1)
    {

        int i = 0;
        if (j < 8)
        {
            i = j + 6;
        }
        else
        {
            i = j - 8;
        }

        LOG(INFO) << "grap " << i << " image:"; 

        if (!camera_left_->grap(host_img_left_[i]) || !camera_right_->grap(host_img_right_[i]))
        {
            camera_left_->streamOff();
            camera_right_->streamOff();
            
            return DF_ERROR_CAMERA_GRAP;
        }

        LOG(INFO) << "finished!";

        cudaStream_t streamNowLeft = i % 2 == 0 ? stream1_ : stream2_;
        cudaStream_t streamNowRight = i % 2 == 0 ? stream3_ : stream4_;

        if (i < 4)
        {
            streamNowLeft = stream2_;
            streamNowRight = stream4_;
        }
        streamNowRight = streamNowLeft;

        for (int nr = 0; nr < image_height_; nr += 1)
        {
            for (int nc = 0; nc < image_width_; nc += 1)
            {
                test_left[nr * image_width_ + nc] = (host_img_left_[i][nr * image_width_ + nc]) >> 4;
                test_right[nr * image_width_ + nc] = (host_img_right_[i][nr * image_width_ + nc]) >> 4;
            }
        }

        cuda_copy_pattern_to_memory(test_left, i, streamNowLeft);
        cuda_copy_pattern_to_memory(test_right, i + MAX_PATTERNS_NUMBER, streamNowRight);
    }

    cudaDeviceSynchronize();
    for (int i = 0; i < 14; i += 1)
    {
        cudaDeviceSynchronize();
        LOG(INFO) << "grap " << i << " image:"; 
        LOG(INFO) << "finished!";

        cudaStream_t streamNowLeft = i % 2 == 0 ? stream1_ : stream2_;
        cudaStream_t streamNowRight = i % 2 == 0 ? stream3_ : stream4_;

        if (i < 4)
        {
            streamNowLeft = stream2_;
            streamNowRight = stream4_;
        }
        streamNowRight = streamNowLeft;

        if (i == 3)
        {
            // 计算四步相移
            cuda_four_step_phase_shift(streamNowLeft, streamNowRight);
            continue;
        }

        if (i == 5)
        {
            // 计算threshold
            //memcpy(buff_brightness_, host_img_left_, image_height_ * image_width_);
            cuda_decode_gray_code_one_by_one(-1, streamNowLeft, streamNowRight);
            continue;
        }

        if (i > 5)
        {
            // 格雷码的移位解码
            cuda_decode_gray_code_one_by_one(i - 6, streamNowLeft, streamNowRight);
            if (i == 13)
            {
                cuda_code_phase_rectify(streamNowLeft, streamNowRight);
            }
            continue;
        }
    }
    
    camera_left_->streamOff();
    camera_right_->streamOff();
    
    cudaDeviceSynchronize();

    LOG(INFO) << "grap end";

    LOG(INFO) << "sort start";

    cuda_code_phase_unwrap(0);//展开

    cuda_fix_four_step_code_shift(0);

    cuda_code_statistics(0);
    cuda_code_statistics_to_index(0);
    cuda_pixels_sort_by_code(0);
    cuda_code_statistics_to_index(0);
    cuda_pixels_shear_by_monotonicity(0);
    cuda_matching(0);
    cuda_disp_to_depth(0);

    cudaDeviceSynchronize();

    LOG(INFO) << "sort end";



    LOG(INFO) << "Stream Off";
    
    removeOutlierBaseDepthFilter();
    removeOutlierBaseRadiusFilter();

    cuda_copy_depth_from_memory(buff_depth_);
    cuda_copy_brightness_from_memory(buff_brightness_);

    delete[] test_left;
    delete[] test_right;

    return DF_SUCCESS;
}

int Scan3D::captureFrame03()
{
    projector_->setProjectorExposure(camera_exposure_);
    LOG(INFO) << "start init basic memory";
    cuda_init_basic_memory();
    LOG(INFO) << "finish init basic memory";

    LOG(INFO) << "Stream On:";
    if (!camera_left_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        camera_left_->streamOff();
        camera_right_->streamOff();
        return DF_ERROR_CAMERA_STREAM;
    }
    if (!camera_right_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        camera_left_->streamOff();
        camera_right_->streamOff();
        return DF_ERROR_CAMERA_STREAM;
    }

    /****************************cudaStreamCreate***********************/

    cudaError_t result_e;
    
    LOG(INFO) << "projector start";
    projector_->project();

    LOG(INFO) << "grap start";

    for (int j = 0; j < 14; j += 1)
    {

        int i = 0;
        if (j < 8)
        {
            i = j + 6;
        }
        else
        {
            i = j - 8;
        }

        LOG(INFO) << "grap " << i << " image:"; 

        if (!camera_left_->grap(host_img_left_[i]) || !camera_right_->grap(host_img_right_[i]))
        {
            camera_left_->streamOff();
            camera_right_->streamOff();
            
            return DF_ERROR_CAMERA_GRAP;
        }

        LOG(INFO) << "finished!";

        cudaStream_t streamNowLeft = i % 2 == 0 ? stream1_ : stream2_;
        cudaStream_t streamNowRight = i % 2 == 0 ? stream3_ : stream4_;

        if (i < 4)
        {
            streamNowLeft = stream2_;
            streamNowRight = stream4_;
        }
        streamNowRight = streamNowLeft;

        cuda_copy_pattern_to_memory(host_img_left_[i], i, streamNowLeft);
        cuda_copy_pattern_to_memory(host_img_right_[i], i + MAX_PATTERNS_NUMBER, streamNowRight);
    }

    cudaDeviceSynchronize();
    for (int i = 0; i < 14; i += 1)
    {
        cudaDeviceSynchronize();
        LOG(INFO) << "grap " << i << " image:"; 
        LOG(INFO) << "finished!";

        cudaStream_t streamNowLeft = i % 2 == 0 ? stream1_ : stream2_;
        cudaStream_t streamNowRight = i % 2 == 0 ? stream3_ : stream4_;

        if (i < 4)
        {
            streamNowLeft = stream2_;
            streamNowRight = stream4_;
        }
        streamNowRight = streamNowLeft;

        if (i == 3)
        {
            // 计算四步相移
            cuda_four_step_phase_shift_16bit(streamNowLeft, streamNowRight);
            continue;
        }

        if (i == 5)
        {
            // 计算threshold
            cuda_decode_gray_code_one_by_one_16bit(-1, streamNowLeft, streamNowRight);
            continue;
        }

        if (i > 5)
        {
            // 格雷码的移位解码
            cuda_decode_gray_code_one_by_one_16bit(i - 6, streamNowLeft, streamNowRight);
            if (i == 13)
            {
                cuda_code_phase_rectify(streamNowLeft, streamNowRight);
            }
            continue;
        }
    }
    
    camera_left_->streamOff();
    camera_right_->streamOff();

    if (!camera_rgb_->streamOn())
    {
        LOG(INFO) << "rgb camera stream on error! ";
        camera_rgb_->streamOff();
    }
    else
    {
        camera_rgb_->grap(buff_color_brightness_);
        camera_rgb_->grap(buff_color_brightness_);
        camera_rgb_->streamOff();
    }
    
    cudaDeviceSynchronize();

    LOG(INFO) << "grap end";

    LOG(INFO) << "sort start";

    cuda_code_phase_unwrap(0);//展开

    cuda_fix_four_step_code_shift(0);

    cuda_code_statistics(0);
    cuda_code_statistics_to_index(0);
    cuda_pixels_sort_by_code(0);
    cuda_code_statistics_to_index(0);
    cuda_pixels_shear_by_monotonicity(0);
    cuda_matching(0);
    cuda_disp_to_depth_and_color_map(0);

    cudaDeviceSynchronize();

    LOG(INFO) << "sort end";



    LOG(INFO) << "Stream Off";
    
    removeOutlierBaseDepthFilter();
    removeOutlierBaseRadiusFilter();
    //if (1 != generate_brightness_model_)
    //{
    //    captureTextureImage(generate_brightness_model_, generate_brightness_exposure_,buff_brightness_);
    //}

    cuda_copy_depth2rgb_map_from_memory((ushort2*)buff_depth_color_map_);
    cuda_copy_depth_from_memory(buff_depth_);
    cuda_copy_brightness_from_memory(buff_brightness_);

    return DF_SUCCESS;
}

bool Scan3D::setLaserBrightness(int brightness)
{
    return projector_->setProjectorCurrent(brightness);
}

bool Scan3D::setParamLaserCurrent(int val)
{
    if (val < 0 || val > 1024)
    {
        return false;
    }

    laser_current_ = val;

    return true;
}

int Scan3D::captureFrame01HDR()
{
    cuda_init_basic_memory_hdr();
    int hdr_count;
    int exposure_time[5];
    int brightness[5];

    read_hdr_param_from_file("./HDR_params.xml", hdr_count, exposure_time, brightness);

    if (hdr_count < 1)
    {
        return DF_FAILED;
    }

    LOG(INFO) << "Stream On:";
    if (!camera_left_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        return DF_ERROR_CAMERA_STREAM;
    }
    if (!camera_right_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        return DF_ERROR_CAMERA_STREAM;
    }

    /****************************cudaStreamCreate***********************/

    cudaError_t result_e;

    LOG(INFO) << "grap start";

    for (int i = 0; i < hdr_count - 1; i += 1)
    {
        projector_->project();
        for (int j = 0; j < 4; j += 1)
        {
            LOG(INFO) << "grap " << j << " image:";
            if (!camera_left_->grap(host_img_left_[j]) || !camera_right_->grap(host_img_right_[j]))
            {
                camera_left_->streamOff();
                camera_right_->streamOff();

                return DF_ERROR_CAMERA_GRAP;
            }

            LOG(INFO) << "finished!";

            cudaStream_t streamNowLeft = j % 2 == 0 ? stream1_ : stream2_;
            cudaStream_t streamNowRight = j % 2 == 0 ? stream3_ : stream4_;

            if (j < 4)
            {
                streamNowLeft = stream2_;
                streamNowRight = stream4_;
            }

            cuda_copy_pattern_to_memory(host_img_left_[j], j, streamNowLeft);
            cuda_copy_pattern_to_memory(host_img_right_[j], j + MAX_PATTERNS_NUMBER, streamNowRight);

            if (j == 3)
            {
                cuda_four_step_phase_shift(streamNowLeft, streamNowRight, i);
            }
        }
        

    }
    
    projector_->project();

    for (int i = 0; i < 14; i += 1)
    {

        LOG(INFO) << "grap " << i << " image:"; 

        if (!camera_left_->grap(host_img_left_[i]) || !camera_right_->grap(host_img_right_[i]))
        {
            camera_left_->streamOff();
            camera_right_->streamOff();
            
            return DF_ERROR_CAMERA_GRAP;
        }

        LOG(INFO) << "finished!";

        cudaStream_t streamNowLeft = i % 2 == 0 ? stream1_ : stream2_;
        cudaStream_t streamNowRight = i % 2 == 0 ? stream3_ : stream4_;

        if (i < 4)
        {
            streamNowLeft = stream2_;
            streamNowRight = stream4_;
        }

        cuda_copy_pattern_to_memory(host_img_left_[i], i, streamNowLeft);
        cuda_copy_pattern_to_memory(host_img_right_[i], i + MAX_PATTERNS_NUMBER, streamNowRight);

        if (i == 3)
        {
            // 计算四步相移
            cuda_four_step_phase_shift(streamNowLeft, streamNowRight, hdr_count - 1);
            cuda_hdr_sort_phase(hdr_count);
            continue;
        }

        if (i == 5)
        {
            // 计算threshold
            //memcpy(buff_brightness_, host_img_left_, image_height_ * image_width_);
            cuda_decode_gray_code_one_by_one(-1, streamNowLeft, streamNowRight);
            continue;
        }

        if (i > 5)
        {
            // 格雷码的移位解码
            cuda_decode_gray_code_one_by_one(i - 6, streamNowLeft, streamNowRight);
            if (i == 13)
            {
                cuda_code_phase_rectify(streamNowLeft, streamNowRight);
            }
            continue;
        }
    }
    
    camera_left_->streamOff();
    camera_right_->streamOff();
    
    cudaDeviceSynchronize();

    LOG(INFO) << "grap end";

    LOG(INFO) << "sort start";

    cuda_code_phase_unwrap(0);//展开
    cuda_code_statistics(0);
    cuda_code_statistics_to_index(0);
    cuda_pixels_sort_by_code(0);
    cuda_code_statistics_to_index(0);
    cuda_pixels_shear_by_monotonicity(0);
    cuda_matching(0);
    cuda_disp_to_depth(0);

    cudaDeviceSynchronize();

    LOG(INFO) << "sort end";



    LOG(INFO) << "Stream Off";
    
    removeOutlierBaseDepthFilter();
    removeOutlierBaseRadiusFilter();
    cuda_copy_depth_from_memory(buff_depth_);
    cuda_copy_brightness_from_memory(buff_brightness_);

    LOG(INFO) << "start init basic memory";

    LOG(INFO) << "finish init basic memory";

    return DF_SUCCESS;
}

int Scan3D::captureFrame01MixedHDR()
{
    int frame_status = DF_SUCCESS;

    int laser_current_before = laser_current_;
    int camera_exposure_before = camera_exposure_;

    int hdr_count;
    int exposure_time[5];
    int brightness[5];

    read_hdr_param_from_file("./HDR_params.xml", hdr_count, exposure_time, brightness);

    if (hdr_count < 1)
    {
        return DF_FAILED;
    }

    LOG(INFO) << "grap start";

    for (int i = 0; i < hdr_count; i += 1)
    {
        int ret = setParamExposure(camera_exposure_list_[i]);
        LOG(INFO) << "camera exposure:" << camera_exposure_list_[i];
        laser_current_ = laser_current_list_[i];
        ret = captureFrame01();
        if (DF_SUCCESS != ret)
        {
            LOG(ERROR) << "captureFrame01 code: " << ret;
            frame_status = ret;

            setParamExposure(camera_exposure_before);
            laser_current_ = laser_current_before;

            return ret;
        }
        // gpu中拷贝数据
        cuda_copy_result_to_hdr(i,5);
    }
    
    cuda_merge_hdr_data(hdr_num_, buff_depth_, buff_brightness_);  

    setParamExposure(camera_exposure_before);
    laser_current_ = laser_current_before;

    LOG(INFO) << "start init basic memory";
    cuda_init_basic_memory_hdr();

    LOG(INFO) << "finish init basic memory";

    return DF_SUCCESS;
}

int Scan3D::captureFrame08MixedHDR()
{
    int frame_status = DF_SUCCESS;

    int laser_current_before = laser_current_;
    int camera_exposure_before = camera_exposure_;

    int hdr_count;
    int exposure_time[5];
    int brightness[5];

    read_hdr_param_from_file("./HDR_params.xml", hdr_count, exposure_time, brightness);

    if (hdr_count < 1)
    {
        return DF_FAILED;
    }

    LOG(INFO) << "grap start";

    for (int i = 0; i < hdr_count; i += 1)
    {
        int ret = setParamExposure(camera_exposure_list_[i]);
        LOG(INFO) << "camera exposure:" << camera_exposure_list_[i];
        laser_current_ = laser_current_list_[i];
        ret = captureFrame08();
        if (DF_SUCCESS != ret)
        {
            LOG(ERROR) << "captureFrame08 code: " << ret;
            frame_status = ret;

            setParamExposure(camera_exposure_before);
            laser_current_ = laser_current_before;

            return ret;
        }
        // gpu中拷贝数据
        cuda_copy_result_to_hdr(i,5);
    }
    
    cuda_merge_hdr_data(hdr_num_, buff_depth_, buff_brightness_);  

    setParamExposure(camera_exposure_before);
    laser_current_ = laser_current_before;

    LOG(INFO) << "start init basic memory";
    cuda_init_basic_memory_hdr();

    LOG(INFO) << "finish init basic memory";

    return DF_SUCCESS;
}

int Scan3D::captureFrame08Repetition(int repetition_count)
{
    // projector_->setProjectorWorkingMode(1);

    // sleep(1);

    projector_->setProjectorExposure(camera_exposure_);
    LOG(INFO) << "start init basic memory";
    cuda_init_basic_memory();
    cuda_clear_repetition_capture_cache();
    LOG(INFO) << "finish init basic memory";

    LOG(INFO) << "Stream On:";
    if (!camera_left_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        camera_left_->streamOff();
        camera_right_->streamOff();
        return DF_ERROR_CAMERA_STREAM;
    }
    if (!camera_right_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        camera_left_->streamOff();
        camera_right_->streamOff();
        return DF_ERROR_CAMERA_STREAM;
    }

    /****************************cudaStreamCreate***********************/

    cudaError_t result_e;
    
    for (int r = 0; r < repetition_count; r += 1)
    {
        LOG(INFO) << "projector start";
        projector_->project();

        LOG(INFO) << "grap start";

        for (int j = 0; j < 18; j += 1)
        {

            int i = 0;
            if (j < 8)
            {
                i = j + 6;
            }
            else if (j < 12)
            {
                i = j - 8;
            }
            else if (j < 16)
            {
                i = j + 2;
            }
            else
            {
                i = j - 12;
            }

            LOG(INFO) << "grap " << i << " image:"; 

            if (!camera_left_->grap(host_img_left_[i]) || !camera_right_->grap(host_img_right_[i]))
            {
                camera_left_->streamOff();
                camera_right_->streamOff();
                
                return DF_ERROR_CAMERA_GRAP;
            }

            LOG(INFO) << "finished!";

            cudaStream_t streamNowLeft = i % 2 == 0 ? stream1_ : stream2_;
            cudaStream_t streamNowRight = i % 2 == 0 ? stream3_ : stream4_;

            if (i < 4)
            {
                streamNowLeft = stream2_;
                streamNowRight = stream4_;
            }
            streamNowRight = streamNowLeft;

            cuda_copy_repetition_pattern_to_memory(host_img_left_[i], i, streamNowLeft);
            cuda_copy_repetition_pattern_to_memory(host_img_right_[i], i + MAX_PATTERNS_NUMBER, streamNowRight);
        }
    }

    cuda_normalize_repetition_patterns(repetition_count);

    cudaDeviceSynchronize();
    for (int i = 0; i < 18; i += 1)
    {
        cudaDeviceSynchronize();
        LOG(INFO) << "grap " << i << " image:"; 
        LOG(INFO) << "finished!";

        cudaStream_t streamNowLeft = i % 2 == 0 ? stream1_ : stream2_;
        cudaStream_t streamNowRight = i % 2 == 0 ? stream3_ : stream4_;

        if (i < 4)
        {
            streamNowLeft = stream2_;
            streamNowRight = stream4_;
        }
        streamNowRight = streamNowLeft;

        if (i == 4)
        {
            // 计算八步相移
            cuda_eight_step_phase_shift_16bit(streamNowLeft, streamNowRight);
            continue;
        }

        if (i == 5)
        {
            // 计算threshold
            cuda_decode_gray_code_one_by_one_16bit(-1, streamNowLeft, streamNowRight);
            continue;
        }

        if (i > 5)
        {
            // 格雷码的移位解码
            cuda_decode_gray_code_one_by_one_16bit(i - 6, streamNowLeft, streamNowRight);
            if (i == 13)
            {
                cuda_code_phase_rectify(streamNowLeft, streamNowRight);
                break;
            }
            continue;
        }
    }
    
    camera_left_->streamOff();
    camera_right_->streamOff();

    if (!camera_rgb_->streamOn())
    {
        LOG(INFO) << "rgb camera stream on error! ";
        camera_rgb_->streamOff();
    }
    else
    {
        camera_rgb_->grap(buff_color_brightness_);
        camera_rgb_->grap(buff_color_brightness_);
        camera_rgb_->streamOff();
    }
    cudaDeviceSynchronize();

    LOG(INFO) << "grap end";

    LOG(INFO) << "sort start";

    cuda_code_phase_unwrap(0);//展开

    // cuda_fix_eight_step_code_shift(0);

    cuda_code_statistics(0);
    cuda_code_statistics_to_index(0);
    cuda_pixels_sort_by_code(0);
    cuda_code_statistics_to_index(0);
    cuda_pixels_shear_by_monotonicity(0);
    cuda_matching(0);
    cuda_disp_to_depth(0);

    cudaDeviceSynchronize();

    LOG(INFO) << "sort end";



    LOG(INFO) << "Stream Off";
    
    removeOutlierBaseDepthFilter();
    removeOutlierBaseRadiusFilter();
    //if (1 != generate_brightness_model_)
    //{
    //    captureTextureImage(generate_brightness_model_, generate_brightness_exposure_,buff_brightness_);
    //}


    cuda_copy_depth_from_memory(buff_depth_);
    cuda_copy_brightness_from_memory(buff_brightness_);

    return DF_SUCCESS;
}


int Scan3D::captureFrame04MixedHDR()
{
    int frame_status = DF_SUCCESS;

    int laser_current_before = laser_current_;
    int camera_exposure_before = camera_exposure_;

    int hdr_count;
    int exposure_time[5];
    int brightness[5];

    read_hdr_param_from_file("./HDR_params.xml", hdr_count, exposure_time, brightness);

    if (hdr_count < 1)
    {
        return DF_FAILED;
    }

    LOG(INFO) << "grap start";

    for (int i = 0; i < hdr_count; i += 1)
    {
        int ret = setParamExposure(camera_exposure_list_[i]);
        LOG(INFO) << "camera exposure:" << camera_exposure_list_[i];
        laser_current_ = laser_current_list_[i];
        ret = captureFrame04();
        if (DF_SUCCESS != ret)
        {
            LOG(ERROR) << "captureFrame04 code: " << ret;
            frame_status = ret;

            setParamExposure(camera_exposure_before);
            laser_current_ = laser_current_before;

            return ret;
        }
        // gpu中拷贝数据
        cuda_copy_result_to_hdr(i,5);
    }
    
    cuda_merge_hdr_data(hdr_num_, buff_depth_, buff_brightness_);  

    setParamExposure(camera_exposure_before);
    laser_current_ = laser_current_before;

    LOG(INFO) << "start init basic memory";
    cuda_init_basic_memory_hdr();

    LOG(INFO) << "finish init basic memory";

    return DF_SUCCESS;
}

int Scan3D::captureFrame04Repetition(int repetition_count)
{
    projector_->setProjectorExposure(camera_exposure_);
    LOG(INFO) << "start init basic memory";
    cuda_init_basic_memory();
    cuda_clear_repetition_capture_cache();
    LOG(INFO) << "finish init basic memory";

    LOG(INFO) << "Stream On:";
    if (!camera_left_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        camera_left_->streamOff();
        camera_right_->streamOff();
        return DF_ERROR_CAMERA_STREAM;
    }
    if (!camera_right_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        camera_left_->streamOff();
        camera_right_->streamOff();
        return DF_ERROR_CAMERA_STREAM;
    }

    /****************************cudaStreamCreate***********************/

    cudaError_t result_e;

    for (int r = 0; r < repetition_count; r += 1)
    {
        LOG(INFO) << "projector start";
        projector_->project();

        LOG(INFO) << "grap start";

        for (int j = 0; j < 18; j += 1)
        {

            int i = 0;
            if (j < 8)
            {
                i = j + 6;
            }
            else if (j < 12)
            {
                i = j - 8;
            }
            else if (j < 16)
            {
                i = j + 2;
            }
            else
            {
                i = j - 12;
            }


            LOG(INFO) << "grap " << i << " image:"; 

            if (!camera_left_->grap(host_img_left_[i]) || !camera_right_->grap(host_img_right_[i]))
            {
                camera_left_->streamOff();
                camera_right_->streamOff();

                return DF_ERROR_CAMERA_GRAP;
            }

            LOG(INFO) << "finished!";

            cudaStream_t streamNowLeft = i % 2 == 0 ? stream1_ : stream2_;
            cudaStream_t streamNowRight = i % 2 == 0 ? stream3_ : stream4_;

            if (i < 4)
            {
                streamNowLeft = stream2_;
                streamNowRight = stream4_;
            }
            streamNowRight = streamNowLeft;

            cuda_copy_repetition_pattern_to_memory(host_img_left_[i], i, streamNowLeft);
            cuda_copy_repetition_pattern_to_memory(host_img_right_[i], i + MAX_PATTERNS_NUMBER, streamNowRight);
        }
    }

    cuda_normalize_repetition_patterns(repetition_count);

    cudaDeviceSynchronize();
    for (int i = 0; i < 18; i += 1)
    {
        cudaDeviceSynchronize();
        LOG(INFO) << "grap " << i << " image:"; 
        LOG(INFO) << "finished!";

        cudaStream_t streamNowLeft = i % 2 == 0 ? stream1_ : stream2_;
        cudaStream_t streamNowRight = i % 2 == 0 ? stream3_ : stream4_;

        if (i < 4)
        {
            streamNowLeft = stream2_;
            streamNowRight = stream4_;
        }
        streamNowRight = streamNowLeft;

        if (i == 3)
        {
            // 计算四步相移
            cuda_eight_step_phase_shift_16bit(streamNowLeft, streamNowRight);
            continue;
        }

        if (i == 5)
        {
            // 计算threshold
            cuda_decode_gray_code_one_by_one_16bit(-1, streamNowLeft, streamNowRight);
            continue;
        }

        if (i > 5)
        {
            // 格雷码的移位解码
            cuda_decode_gray_code_one_by_one_16bit(i - 6, streamNowLeft, streamNowRight);
            if (i == 13)
            {
                cuda_code_phase_rectify(streamNowLeft, streamNowRight);
            }
            continue;
        }
    }
    
    camera_left_->streamOff();
    camera_right_->streamOff();

    if (!camera_rgb_->streamOn())
    {
        LOG(INFO) << "rgb camera stream on error! ";
        camera_rgb_->streamOff();
    }
    else
    {
        camera_rgb_->grap(buff_color_brightness_);
        camera_rgb_->grap(buff_color_brightness_);
        camera_rgb_->streamOff();
    }
    cudaDeviceSynchronize();

    LOG(INFO) << "grap end";

    LOG(INFO) << "sort start";

    cuda_code_phase_unwrap(0);//展开

    // cuda_fix_four_step_code_shift(0);

    cuda_code_statistics(0);
    cuda_code_statistics_to_index(0);
    cuda_pixels_sort_by_code(0);
    cuda_code_statistics_to_index(0);
    cuda_pixels_shear_by_monotonicity(0);
    cuda_matching(0);
    cuda_disp_to_depth(0);

    cudaDeviceSynchronize();

    LOG(INFO) << "sort end";



    LOG(INFO) << "Stream Off";
    
    removeOutlierBaseDepthFilter();
    removeOutlierBaseRadiusFilter();
    //if (1 != generate_brightness_model_)
    //{
    //    captureTextureImage(generate_brightness_model_, generate_brightness_exposure_,buff_brightness_);
    //}


    cuda_copy_depth_from_memory(buff_depth_);
    cuda_copy_brightness_from_memory(buff_brightness_);

    return DF_SUCCESS;
}

bool Scan3D::setParamHdr(int num,std::vector<int> led_list,std::vector<int> exposure_list)
{
    if(led_list.size() != exposure_list.size() || exposure_list.size() < 5 || exposure_list.size() > 6)
    {
        LOG(ERROR) << "setParamHdr() Error! ";
        return false;
    }

    hdr_num_ = num;

    laser_current_list_ = led_list;
    camera_exposure_list_ = exposure_list;

    return true;
}

int Scan3D::getParamMaxCameraExposure()
{
    return max_camera_exposure_;
}


int Scan3D::captureFrameTest(unsigned char* patterns_buf)
{
    LOG(INFO) << "Stream On:";
    if (!camera_left_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        return DF_ERROR_CAMERA_STREAM;
    }
    if (!camera_right_->streamOn())
    {
        LOG(INFO) << "Stream On Error";
        return DF_ERROR_CAMERA_STREAM;
    }

    int img_size = image_width_ * image_height_;

    /****************************cudaStreamCreate***********************/

    cudaError_t result_e;

    projector_->project();

    LOG(INFO) << "grap start";
    for (int i = 0; i < 14; i += 1)
    {

        LOG(INFO) << "grap " << i << " image:"; 

        if (!camera_left_->grap(host_img_left_[i]) || !camera_right_->grap(host_img_right_[i]))
        {
            camera_left_->streamOff();
            camera_right_->streamOff();

            return DF_ERROR_CAMERA_GRAP;
        }

        LOG(INFO) << "finished!";

        //memcpy(patterns_buf + img_size * i, host_img_left_, img_size);
        //memcpy(patterns_buf + img_size * (i + 14), host_img_right_, img_size);

        cudaStream_t streamNowLeft = i % 2 == 0 ? stream1_ : stream2_;
        cudaStream_t streamNowRight = i % 2 == 0 ? stream3_ : stream4_;

        // if (i < 4)
        // {
        //     streamNowLeft = stream2_;
        //     streamNowRight = stream4_;
        // }

        cuda_copy_pattern_to_memory(host_img_left_[i], i, streamNowLeft);
        cuda_copy_pattern_to_memory(host_img_right_[i], i + MAX_PATTERNS_NUMBER, streamNowRight);
        //cudaDeviceSynchronize();

        if (i == 3)
        {
            // 计算四步相移
            cuda_four_step_phase_shift(streamNowLeft, streamNowRight);
            continue;
        }

        if (i == 5)
        {
            // 计算threshold
            //memcpy(buff_brightness_, host_img_left_, image_height_ * image_width_);
            cuda_decode_gray_code_one_by_one(-1, streamNowLeft, streamNowRight);
            continue;
        }

        if (i > 5)
        {
            // 格雷码的移位解码
            cuda_decode_gray_code_one_by_one(i - 6, streamNowLeft, streamNowRight);
            if (i == 13)
            {
                cuda_code_phase_rectify(streamNowLeft, streamNowRight);
            }
            continue;
        }
    }
    
    camera_left_->streamOff();
    camera_right_->streamOff();
    
    cudaDeviceSynchronize();

    LOG(INFO) << "grap end";

    LOG(INFO) << "sort start";

    cuda_code_phase_unwrap(0);//展开
    cuda_code_statistics(0);
    cuda_code_statistics_to_index(0);
    cuda_pixels_sort_by_code(0);
    cuda_code_statistics_to_index(0);
    cuda_pixels_shear_by_monotonicity(0);
    cuda_matching(0);
    cuda_disp_to_depth(0);

    cudaDeviceSynchronize();

    LOG(INFO) << "sort end";



    LOG(INFO) << "Stream Off";
    

    //if (1 != generate_brightness_model_)
    //{
    //    captureTextureImage(generate_brightness_model_, generate_brightness_exposure_,buff_brightness_);
    //}

    removeOutlierBaseDepthFilter();
    removeOutlierBaseRadiusFilter();

    cuda_copy_depth_from_memory(buff_depth_);
    cuda_copy_brightness_from_memory(buff_brightness_);

    for (int i = 0; i < 28; i += 1)
    {
        cuda_copy_brightness_from_memory(patterns_buf + image_height_ * image_width_ * i, i);
    }

    LOG(INFO) << "start init basic memory";
    cuda_init_basic_memory();
    LOG(INFO) << "finish init basic memory";

    return DF_SUCCESS;
}

bool Scan3D::readCalibParam()
{
    std::ifstream ifile; 
    ifile.open("calib_param.txt");

    if(!ifile.is_open())
    {
        return false;
    }

    int n_params = sizeof(calib_param_)/sizeof(float);
    for(int i=0; i<n_params; i++)
    {
	    ifile>>(((float*)(&calib_param_))[i]); 
    }
    ifile.close();
    return true;
}

bool Scan3D::writeCalibParamUser()
{
    std::ofstream ofile;
    ofile.open("calib_param_user.txt");
    int n_params = sizeof(calib_param_user_) / sizeof(float);
    for (int i = 0; i < n_params; i++)
    {
        ofile << (((float*)(&calib_param_user_))[i]) << std::endl;
    }
    ofile.close();
    return DF_SUCCESS;
}


bool Scan3D::loadCalibData()
{

    if(!readCalibParam())
    {
        LOG(INFO)<<"Read Calib Param Error!";  
        return false; 
    }
    else
    {
        
        LOG(INFO)<<"cuda_copy_calib_data:";  
    }

    return true;
}

void Scan3D::copyBrightnessData(unsigned char* &ptr)
{ 
	memcpy(ptr, buff_brightness_, sizeof(unsigned char)*image_height_*image_width_); 
}

void Scan3D::copyColorBrightnessData(unsigned char* &ptr)
{ 
	memcpy(ptr, buff_color_brightness_, sizeof(unsigned char)*rgb_image_height_*rgb_image_width_*3); 
}

void Scan3D::copyResizeColorBrightnessData(unsigned char* &ptr)
{ 
	memcpy(ptr, buff_color_brightness_, sizeof(unsigned char)*rgb_image_height_*rgb_image_width_*3/4); 
}

void Scan3D::copyDepthData(float* &ptr)
{ 
	memcpy(ptr, buff_depth_, sizeof(float)*image_height_*image_width_);
} 

void Scan3D::copyPointcloudData(float* &ptr)
{ 
    // reconstruct_copy_pointcloud_from_cuda_memory(ptr);
}

void Scan3D::copyDepth2ColorData(unsigned short* &ptr)
{
    memcpy(ptr, buff_depth_color_map_, 2*sizeof(unsigned short)*image_height_*image_width_);
}

void Scan3D::getCameraResolution(int &width, int &height)
{
    LOG(INFO) << "image_width: " << image_width_;
    LOG(INFO) << "image_height: " << image_height_;
    width = image_width_;
    height = image_height_;
}

void Scan3D::getRGBCameraResolution(int &width, int &height)
{
    LOG(INFO) << "rgb_image_width: " << rgb_image_width_;
    LOG(INFO) << "rgb_image_height: " << rgb_image_height_;
    width = rgb_image_width_;
    height = rgb_image_height_;
}

void Scan3D::removeOutlierBaseDepthFilter()
{
    if (1 == system_config_settings_machine_.Instance().firwmare_param_.use_depth_filter)
    {
        float depth_threshold = system_config_settings_machine_.Instance().firwmare_param_.depth_filter_threshold * (-0.195) + 20.;
        LOG(INFO) << "depth_filter_threshold: " << depth_threshold;

        depth_filter(depth_threshold / 1000.);
    }
}

void Scan3D::removeOutlierBaseRadiusFilter()
{
    if(1 == system_config_settings_machine_.Instance().firwmare_param_.use_radius_filter)
    {
        float r = system_config_settings_machine_.Instance().firwmare_param_.radius_filter_r;
        int num = system_config_settings_machine_.Instance().firwmare_param_.radius_filter_threshold_num;
        LOG(INFO)<<"radius_filter_r: "<<r;
        LOG(INFO)<<"num: "<<num; 

        cuda_remove_points_base_radius_filter(0.5,r,num);
    }
}


bool Scan3D::read_hdr_param_from_file(const char* param_path, int& hdr_count, int* exposure_time, int* brightness)
{
    std::string file_path(param_path);
    cv::FileStorage fs_in, fs_out;

    if (!fs_in.open(file_path, cv::FileStorage::READ))
    {
        fs_in.release();
        fs_out.open(file_path, cv::FileStorage::WRITE);

        cv::Mat HDR_exposure_list = (cv::Mat_<int>(5, 1) << 3000,
            6000,
            12000,
            24000,
            48000);

        cv::Mat HDR_brightness_list = (cv::Mat_<int>(5, 1) << 1023,
            1023,
            1023,
            1023,
            1023);

        fs_out << "hdr_count" << 1;
        fs_out << "hdr_exposure_list" << HDR_exposure_list;
        fs_out << "hdr_brightness_list" << HDR_brightness_list;
        fs_out.release();

        fs_in.open(file_path, cv::FileStorage::READ);
    }

    cv::Mat brightness_list(5, 1, CV_32S, brightness);
    cv::Mat exposure_list(5, 1, CV_32S, exposure_time);

    fs_in["hdr_count"] >> hdr_count;
    fs_in["hdr_exposure_list"] >> exposure_list;
    fs_in["hdr_brightness_list"] >> brightness_list;

    return true;
}

bool Scan3D::read_calib_param_from_file(const char* param_path, double* intrinsic_l, double* intrinsic_r, double* dist_l, double* dist_r, double* R_ptr, double* T_ptr)
{
    std::string file_path(param_path);
    cv::FileStorage fs_in, fs_out;

    if (!fs_in.open(file_path, cv::FileStorage::READ))
    {
        fs_in.release();
        return false;
    }

    cv::Mat intrinsic_l_mat(3, 3, CV_64F, intrinsic_l);
    cv::Mat intrinsic_r_mat(3, 3, CV_64F, intrinsic_r);
    cv::Mat dist_l_mat(1, 5, CV_64F, dist_l);
    cv::Mat dist_r_mat(1, 5, CV_64F, dist_r);
    cv::Mat R_ptr_mat(3, 3, CV_64F, R_ptr);
    cv::Mat T_ptr_mat(3, 1, CV_64F, T_ptr);

    fs_in["camera_intrinsic_l"] >> intrinsic_l_mat;
    fs_in["camera_dist_l"] >> dist_l_mat;
    fs_in["camera_intrinsic_r"] >> intrinsic_r_mat;
    fs_in["camera_dist_r"] >> dist_r_mat;
    fs_in["R"] >> R_ptr_mat;
    fs_in["T"] >> T_ptr_mat;

    return true;
}

bool Scan3D::read_calib_param_from_file(const char* param_path, double* intrinsic_l, double* intrinsic_r, double* intrinsic_rgb, double* dist_l, double* dist_r, double* dist_rgb, double* R_ptr, double* R_rgb_ptr, double* T_ptr, double* T_rgb_ptr)
{
    std::string file_path(param_path);
    cv::FileStorage fs_in, fs_out;

    if (!fs_in.open(file_path, cv::FileStorage::READ))
    {
        fs_in.release();
        return false;
    }

    cv::Mat intrinsic_l_mat(3, 3, CV_64F, intrinsic_l);
    cv::Mat intrinsic_r_mat(3, 3, CV_64F, intrinsic_r);
    cv::Mat intrinsic_rgb_mat(3, 3, CV_64F, intrinsic_rgb);
    cv::Mat dist_l_mat(1, 5, CV_64F, dist_l);
    cv::Mat dist_r_mat(1, 5, CV_64F, dist_r);
    cv::Mat dist_rgb_mat(1, 5, CV_64F, dist_rgb);
    cv::Mat R_ptr_mat(3, 3, CV_64F, R_ptr);
    cv::Mat T_ptr_mat(3, 1, CV_64F, T_ptr);
    cv::Mat R_rgb_ptr_mat(3, 3, CV_64F, R_rgb_ptr);
    cv::Mat T_rgb_ptr_mat(3, 1, CV_64F, T_rgb_ptr);

    fs_in["camera_intrinsic_l"] >> intrinsic_l_mat;
    fs_in["camera_dist_l"] >> dist_l_mat;
    fs_in["camera_intrinsic_r"] >> intrinsic_r_mat;
    fs_in["camera_dist_r"] >> dist_r_mat;
    fs_in["R"] >> R_ptr_mat;
    fs_in["T"] >> T_ptr_mat;
    fs_in["camera_intrinsic_rgb"] >> intrinsic_rgb_mat;
    fs_in["camera_dist_rgb"] >> dist_rgb_mat;
    fs_in["R_l2rgb"] >> R_rgb_ptr_mat;
    fs_in["T_l2rgb"] >> T_rgb_ptr_mat;

    return true;
}
