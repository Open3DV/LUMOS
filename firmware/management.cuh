#ifndef MEMORY_MANAGEMENT_CUDA_CUH
#define MEMORY_MANAGEMENT_CUDA_CUH
#pragma once
#include <device_launch_parameters.h>
#include <device_functions.h>
#include <cuda_runtime.h>  
#include <cuda_texture_types.h>
#include <texture_types.h>  
#include <iostream>
#include <stdint.h>
#include <vector>
#include <opencv2/core.hpp> 
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "encode_cuda.cuh"
#include "easylogging++.h"
#include "merge_hdr.cuh"


#define MAX_PATTERNS_NUMBER 14
#define MAX_GRAY_CODE_NUMBER 8
#define D_HDR_MAX_NUM 5

__device__ int d_image_width_ = 0;
__device__ int d_image_height_ = 0;
__device__ int d_rgb_image_width_ = 0;
__device__ int d_rgb_image_height_ = 0;

__device__ float d_confidence_ = 100;
__device__ float d_confidence_t = 100;

__device__ float d_camera_gamma_ = 1;

/**********************************************************************/
//basic memory
// 0是左目，1是右目
__device__ unsigned char* d_patterns_list_[2][MAX_PATTERNS_NUMBER];
__device__ unsigned char* d_remap_patterns_list_[2][MAX_PATTERNS_NUMBER];

/**16bit**/
__device__ unsigned short* d_patterns_list_16bit_[2][MAX_PATTERNS_NUMBER];
__device__ unsigned short* d_remap_patterns_list_16bit_[2][MAX_PATTERNS_NUMBER];

/**********************************************************************/
__device__ float* d_pointcloud_map_;
__device__ float* d_depth_map_;
__device__ unsigned char* d_mask_map_;
__device__ float* d_depth_map_temp_;
__device__ unsigned char* d_depth_mask_;
__device__ float* d_disparty_map_;
__device__ unsigned char* d_brightness_map_[2];

/**********************************************************************/
__device__ unsigned char* d_threshold_map_[2];

/**16bit**/
__device__ unsigned short* d_threshold_map_16bit_[2];

__device__ unsigned char* d_noise_mask_map_[2];
__device__ unsigned char* d_disp_mask_map_;
__device__ unsigned char* d_gray_to_bin_decode_map_;
__device__ float* d_Q;

__device__ float* d_intrinsic_rgb;
__device__ float* d_R_l2rgb;
__device__ float* d_T_l2rgb;
__device__ ushort2* d_depth2rgb_map;

__device__ unsigned char* d_phase_map_[2];
__device__ unsigned char* d_code_map_[2];

__device__ unsigned short* d_unwraped_pixels[2]; //用于存储unwrap之后的结果，可以进行插值补空洞
__device__ unsigned short* d_sorted_pixels[2];
__device__ unsigned short* d_num_of_pixels_[2]; //先存储数量，后改为存储起始位置
__device__ unsigned short* d_index_of_pixels_[2]; //先存储数量，后改为存储起始位置
__device__ unsigned short* d_sheared_start_pixels[2]; //用于存储分割起始位置，一个格雷码允许中间出现一次分割
__device__ unsigned short* d_sheared_end_pixels[2]; //用于存储分割终止位置

/**********************************************************************/
//hdr memory

__device__ float* d_hdr_depth_map_list_[D_HDR_MAX_NUM];
__device__ unsigned char* d_hdr_brightness_list_[D_HDR_MAX_NUM];
__device__ float* d_hdr_bright_pixel_sum_list_[D_HDR_MAX_NUM];
__device__ unsigned char* d_hdr_phase_map_[2][D_HDR_MAX_NUM];
__device__ unsigned char* d_hdr_brightness_map_[2][D_HDR_MAX_NUM];


/**********************************************************************/

// remap所需的表一共6个，左右各三个
__device__ short2* d_remap_xy_map[2];
__device__ unsigned short* d_weight_index_map[2];
__device__ short4* d_weight_map;

//__device__ float* d_camera_intrinsic_ = NULL;
//__device__ float* d_project_intrinsic_ = NULL;
//__device__ float* d_camera_distortion_ = NULL;
//__device__ float* d_projector_distortion_ = NULL;
//__device__ float* d_rotation_matrix_ = NULL;
//__device__ float* d_translation_matrix_ = NULL;


//__device__ float d_baseline_ = 0;

/**********************************************************************/
//分配basic内存
bool cuda_set_camera_resolution(int width, int height);

bool cuda_set_camera_resolution(int width, int height, int rgb_width, int rgb_height);

bool cuda_malloc_basic_memory();

bool cuda_free_basic_memory();

bool cuda_init_basic_memory();

bool cuda_init_basic_memory_hdr();

/**********************************************************************/
//copy
bool cuda_copy_pattern_to_memory(unsigned char* pattern_ptr, int serial_flag);

bool cuda_copy_pattern_to_memory(unsigned char* pattern_ptr, int serial_flag, cudaStream_t stream);

bool cuda_copy_pattern_to_memory(unsigned short* pattern_ptr, int serial_flag, cudaStream_t stream);

bool cuda_copy_decode_map_to_memory(unsigned char* decode_map_ptr);

bool cuda_copy_remap_maps_to_memory(short2* remap_xy_map_l, short2* remap_xy_map_r, unsigned short* weight_index_map_l, unsigned short* weight_index_map_r, short4* weight_map);

bool cuda_copy_Q_map_to_memory(float* Q_map);

bool cuda_copy_rgb_transform_to_memory(float* input_rgb_camera_intrinsic, float* input_l2rgb_R, float* input_l2rgb_T);

//void cuda_copy_pointcloud_from_memory(float* pointcloud);

void cuda_copy_depth_from_memory(float* depth);

void cuda_copy_brightness_from_memory(unsigned char* brightness);

void cuda_copy_brightness_from_memory(unsigned char* brightness, int index);

void cuda_copy_brightness_to_memory(unsigned char* brightness);

void cuda_copy_code_from_memory(unsigned char* code);

void cuda_copy_code_to_memory(unsigned char* code);

void cuda_copy_phase_from_memory(unsigned char* phase);

void cuda_copy_phase_to_memory(unsigned char* phase);

void cuda_copy_code_statistics_from_memory(unsigned short* code_statistics);

void cuda_copy_code_sorted_index_from_memory(unsigned short* code_sorted_index);

void cuda_copy_disparty_from_memory(float* disparty);

void cuda_copy_depth2rgb_map_from_memory(ushort2* output_depth2rgb_map);



/**********************************************************************/

bool cuda_decode_gray_code(int serial_flag);

bool cuda_four_step_phase_shift();

bool cuda_four_step_phase_shift(cudaStream_t stream_left, cudaStream_t stream_right);

bool cuda_four_step_phase_shift_16bit(cudaStream_t stream_left, cudaStream_t stream_right);

bool cuda_four_step_phase_shift(cudaStream_t stream_left, cudaStream_t stream_right, int serial_flag);

bool cuda_decode_gray_code_one_by_one(int serial_flag);

bool cuda_decode_gray_code_one_by_one(int serial_flag, cudaStream_t stream_left, cudaStream_t stream_right);

bool cuda_decode_gray_code_one_by_one_16bit(int serial_flag, cudaStream_t stream_left, cudaStream_t stream_right);

bool cuda_code_phase_rectify();

bool cuda_code_phase_rectify(cudaStream_t stream_left, cudaStream_t stream_right);

bool cuda_code_phase_unwrap(int serial_flag);

bool cuda_code_statistics(int serial_flag);

bool cuda_code_statistics_to_index(int serial_flag);

bool cuda_pixels_sort_by_code(int serial_flag);//遍历写入排序之后的pixel，维护起始，终止以及

bool cuda_pixels_shear_by_monotonicity(int serial_flag);//遍历unwrap之后的相位值，根据单调性的判断，找出合适的序列长度

bool cuda_matching(int serial_flag);// 排除对于0的matching

bool cuda_disp_to_depth(int serial_flag);

bool cuda_disp_to_depth_and_color_map(int serial_flag);

void depth_filter(float depth_threshold_val);

bool cuda_hdr_sort_phase(int serial_flag);

void cuda_remove_points_base_radius_filter(float dot_spacing,float radius,int threshold_num);

bool cuda_set_param_confidence(float val);

void cuda_fix_four_step_code_shift(int serial_flag);

bool cuda_copy_result_to_hdr(int serial_flag,int brigntness_serial);

bool cuda_merge_hdr_data(int hdr_num,float* depth_map, unsigned char* brightness);

bool cuda_set_camera_gamma(float gamma);

bool cuda_get_camera_gamma(float& gamma);

//bool 

//bool cuda_compute_phase_shift(int serial_flag);

//bool cuda_unwrap_phase_shift(int serial_flag);

/**********************************************************************/
//reconstruct
//bool cuda_generate_pointcloud_base_table();

/**********************************************************************/

#endif