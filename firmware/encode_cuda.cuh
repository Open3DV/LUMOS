#ifndef ENCODE_CUDA_CUH
#define ENCODE_CUDA_CUH
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
#include "easylogging++.h"

#define DEPTH_DIFF_NUM_THRESHOLD 3

//kernel
__global__ void kernel_decode_gray_code_8bit(int width, int height, unsigned char* decode_map, unsigned char* d_in_dark, unsigned char* d_in_bright, unsigned char* d_in_0, unsigned char* d_in_1, unsigned char* d_in_2, unsigned char* d_in_3, unsigned char* d_in_4, unsigned char* d_in_5, unsigned char* d_in_6, unsigned char* d_in_7, unsigned char* d_out, unsigned char* mask_niose);

__global__ void kernel_decode_gray_code_one_by_one(int width, int height, unsigned char* d_in_threshold, unsigned char* d_in_img, unsigned char* d_out, unsigned char* mask_niose);

__global__ void kernel_decode_gray_code_one_by_one(int width, int height, unsigned short* d_in_threshold, unsigned short* d_in_img, unsigned char* d_out, unsigned char* mask_niose);

__global__ void kernel_4_step_phase_shift_8bit(int width, int height, unsigned char* d_in_0, unsigned char* d_in_1, unsigned char* d_in_2, unsigned char* d_in_3, unsigned char* d_out, unsigned char* mask_noise);

__global__ void kernel_4_step_phase_shift_8bit(int width, int height, unsigned char* d_in_0, unsigned char* d_in_1, unsigned char* d_in_2, unsigned char* d_in_3, unsigned char* d_out, unsigned char* mask_noise, unsigned char* decode_threshold, float d_in_confidence);

__global__ void kernel_4_step_phase_shift_16bit(int width, int height, unsigned short* d_in_0, unsigned short* d_in_1, unsigned short* d_in_2, unsigned short* d_in_3, unsigned char* d_out, unsigned char* mask_noise, unsigned short* decode_threshold, float d_in_confidence);

__global__ void kernel_code_rectify_8bit(int width, int height, unsigned char* d_in_code, unsigned char* d_in_phase, unsigned char* mask_noise);

__global__ void kernel_code_phase_unwrap_8bit(int width, int height, unsigned char* d_in_code, unsigned char* d_in_phase, unsigned short* d_out_unwrap, unsigned char* mask_noise);

__global__ void kernel_decode_threshold_and_mask(int width, int height, float d_in_confidence, unsigned char* d_in_darkness, unsigned char* d_in_brightness, unsigned char* d_threshold, unsigned char* d_mask_noise, unsigned char* d_code);

__global__ void kernel_decode_threshold_and_mask(int width, int height, float d_in_confidence, unsigned char* d_in_darkness, unsigned char* d_in_brightness, unsigned short* d_threshold, unsigned char* d_mask_noise, unsigned char* d_code);

__global__ void kernel_convert_brightness_to_8bit(int width, int height, unsigned short* d_in_brightness_16bit, unsigned char* d_in_brightness, unsigned char* d_code);

__global__ void kernel_gray_code_to_bin_code(int width, int height, unsigned char* d_in_out_code, unsigned char* d_gray_code_to_bin_map, unsigned char* d_in_noise_mask, unsigned char* d_in_test);

__global__ void kernel_code_statistics(int width, int height, unsigned char* d_in_code, unsigned short* d_num_of_pixels_one_code);

__global__ void kernel_code_statistics_to_index(int width, int height, unsigned short* d_num_of_pixels_one_code, unsigned short* d_inedx_of_pixels);

__global__ void kernel_sort_code(int width, int height, unsigned char* d_in_code, unsigned short* d_sorted_pixels, unsigned short* d_num_of_pixels_one_code, unsigned char* d_in_noise_mask);

__global__ void kernel_filter_code_noise(int width, int height, unsigned short* d_in_unwrap_phase, unsigned short* d_in_num_of_pixels, unsigned short* d_in_index_of_pixels, unsigned short* d_in_sorted_pixels);

__global__ void kernel_change_edge(int width, int height, unsigned short* d_in_unwrap_phase, unsigned short* d_in_sorted_pixels, unsigned short* d_in_num_of_pixels, unsigned short* d_in_index_of_pixels);

__global__ void kernel_matching(int width, int height, unsigned short* d_in_sorted_pixels_left, unsigned short* d_in_unwrap_phase_left, unsigned short* d_in_num_of_pixels_left, unsigned short* d_in_index_of_pixels_left, unsigned short* d_in_sorted_pixels_right, unsigned short* d_in_unwrap_phase_right, unsigned short* d_in_num_of_pixels_right, unsigned short* d_in_index_of_pixels_right, float* disparty, unsigned char* disparty_mask);

__global__ void kernel_dispaty_to_xyz(int width, int height, float* d_in_Q, float* d_in_disparty, float* d_out_point_cloud_xyz);

__global__ void kernel_dispaty_to_depth(int width, int height, float* d_in_Q, float* d_in_disparty, float* d_out_depth_map, unsigned char* disparty_mask);

__global__ void kernel_depth_to_pointcloud(int width, int height, float* d_in_Q, float* d_in_depth_map, float* d_out_pointcloud_map);

__global__ void kernel_remap(uchar* src, uchar* dst, short2* map1, ushort* map2, short4* weight, int width, int height);

__global__ void kernel_remap(unsigned short* src, unsigned short* dst, short2* map1, ushort* map2, short4* weight, int width, int height);

__global__ void kernel_depth_filter_step_1(uint32_t img_height, uint32_t img_width, float depth_threshold, float* const depth_map, float* const depth_map_temp, unsigned char* mask_temp);

__global__ void kernel_depth_filter_step_2(uint32_t img_height, uint32_t img_width, float depth_threshold, float* const depth_map, float* const depth_map_temp, unsigned char* mask_temp);

__global__ void kernel_filter_radius_outlier_removal(uint32_t img_height, uint32_t img_width,float* const point_cloud_map,unsigned char* remove_mask, float dot_spacing_2, float r_2,int threshold);

__global__ void kernel_removal_points_base_mask(uint32_t img_height, uint32_t img_width,float* const point_cloud_map,float* const depth_map,uchar* remove_mask);

__global__ void kernel_fix_unwrap_phase(int width, int height, unsigned short* d_in_unwrap_phase);

#endif