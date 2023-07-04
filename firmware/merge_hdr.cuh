#ifndef MERGE_HDR_CUDA_CUH
#define MERGE_HDR_CUDA_CUH
#pragma once 
#include <device_launch_parameters.h>
#include <device_functions.h>
#include <cuda_runtime.h>  
#include <cuda_texture_types.h>
#include <stdint.h>

__global__ void cuda_count_sum_pixel(const unsigned char* brightness,uint32_t img_height, uint32_t img_width, float* sum_pixels);

__global__ void cuda_merge_hdr_depth_6(const float*  depth_map_0,const float*  depth_map_1,const float*  depth_map_2,
	const float*  depth_map_3,const float*  depth_map_4,const float*  depth_map_5,
	const unsigned char* brightness_0,const unsigned char* brightness_1,const unsigned char* brightness_2,
	const unsigned char* brightness_3,const unsigned char* brightness_4,const unsigned char* brightness_5,
	uint32_t img_height, uint32_t img_width, float* const depth_map,unsigned char * const brightness);


__global__ void cuda_merge_hdr_depth_5(const float*  depth_map_0,const float*  depth_map_1,const float*  depth_map_2,
	const float*  depth_map_3,const float*  depth_map_4,
	const unsigned char* brightness_0,const unsigned char* brightness_1,const unsigned char* brightness_2,
	const unsigned char* brightness_3,const unsigned char* brightness_4,
	uint32_t img_height, uint32_t img_width, float* const depth_map,unsigned char * const brightness);

__global__ void cuda_merge_hdr_depth_4(const float*  depth_map_0,const float*  depth_map_1,const float*  depth_map_2,const float*  depth_map_3,
	const unsigned char* brightness_0,const unsigned char* brightness_1,const unsigned char* brightness_2,const unsigned char* brightness_3,
	uint32_t img_height, uint32_t img_width, float* const depth_map,unsigned char * const brightness);

__global__ void cuda_merge_hdr_depth_3(const float*  depth_map_0,const float*  depth_map_1,const float*  depth_map_2,const unsigned char* brightness_0,const unsigned char* brightness_1,
	const unsigned char* brightness_2,uint32_t img_height, uint32_t img_width, float* const depth_map,unsigned char * const brightness);

__global__ void cuda_merge_hdr_depth_2(const float*  depth_map_0,const float*  depth_map_1,const unsigned char* brightness_0,const unsigned char* brightness_1,
	uint32_t img_height, uint32_t img_width, float* const depth_map,unsigned char * const brightness);


__global__ void cuda_merge_hdr_2(const unsigned char*  phase_map_0,const unsigned char*  phase_map_1,const unsigned char* brightness_0,const unsigned char* brightness_1, uint32_t img_height, uint32_t img_width, unsigned char* const phase_map_out);

__global__ void cuda_merge_hdr_3(const unsigned char*  phase_map_0,const unsigned char*  phase_map_1,const unsigned char*  phase_map_2,const unsigned char* brightness_0,const unsigned char* brightness_1,
	const unsigned char* brightness_2,uint32_t img_height, uint32_t img_width, unsigned char* const phase_map_out);

__global__ void cuda_merge_hdr_4(const unsigned char*  phase_map_0,const unsigned char*  phase_map_1,const unsigned char*  phase_map_2,const unsigned char*  phase_map_3,
	const unsigned char* brightness_0,const unsigned char* brightness_1,const unsigned char* brightness_2,const unsigned char* brightness_3,
	uint32_t img_height, uint32_t img_width, unsigned char* const phase_map_out);

__global__ void cuda_merge_hdr_5(const unsigned char*  phase_map_0,const unsigned char*  phase_map_1,const unsigned char*  phase_map_2,
	const unsigned char*  phase_map_3,const unsigned char*  phase_map_4,
	const unsigned char* brightness_0,const unsigned char* brightness_1,const unsigned char* brightness_2,
	const unsigned char* brightness_3,const unsigned char* brightness_4,
	uint32_t img_height, uint32_t img_width, unsigned char* const phase_map_out);

__global__ void cuda_merge_hdr_6(const unsigned char*  phase_map_0,const unsigned char*  phase_map_1,const unsigned char*  phase_map_2,
	const unsigned char*  phase_map_3,const unsigned char*  phase_map_4,const unsigned char*  phase_map_5,
	const unsigned char* brightness_0,const unsigned char* brightness_1,const unsigned char* brightness_2,
	const unsigned char* brightness_3,const unsigned char* brightness_4,const unsigned char* brightness_5,
	uint32_t img_height, uint32_t img_width, unsigned char* const phase_map_out);

#endif