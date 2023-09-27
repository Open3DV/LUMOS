#include "management.cuh"

#define CHECK(call)\
{\
  const cudaError_t error=call;\
  if(error!=cudaSuccess)\
  {\
      printf("ERROR: %s:%d,",__FILE__,__LINE__);\
      printf("code:%d,reason:%s\n",error,cudaGetErrorString(error));\
      exit(1);\
  }\
}

int h_image_width_ = 0;
int h_image_height_ = 0;
int h_rgb_image_width_ = 0;
int h_rgb_image_height_ = 0;

dim3 threadsPerBlock(16, 16);
dim3 blocksPerGrid((d_image_width_ + threadsPerBlock.x - 1) / threadsPerBlock.x,
	(d_image_height_ + threadsPerBlock.y - 1) / threadsPerBlock.y);

bool cuda_set_camera_resolution(int width, int height)
{
	h_image_width_ = width;
	h_image_height_ = height;

	d_image_width_ = width;
	d_image_height_ = height;

	cudaError_t error_code = cudaMemcpyToSymbol(d_image_width_, &width, sizeof(int));
	if (error_code != cudaSuccess)
	{
		return false;
	}

	error_code = cudaMemcpyToSymbol(d_image_height_, &height, sizeof(int));

	if (error_code != cudaSuccess)
	{
		return false;
	}


	blocksPerGrid.x = (width + threadsPerBlock.x - 1) / threadsPerBlock.x;
	blocksPerGrid.y = (height + threadsPerBlock.y - 1) / threadsPerBlock.y;

	LOG(INFO) << "init d_image_width_: " << d_image_width_;
	LOG(INFO) << "init d_image_height_: " << d_image_height_;
	LOG(INFO) << "blocksPerGrid.x: " << blocksPerGrid.x;
	LOG(INFO) << "blocksPerGrid.y: " << blocksPerGrid.y;

	return true;
}

bool cuda_set_camera_resolution(int width, int height, int rgb_width, int rgb_height)
{
	h_image_width_ = width;
	h_image_height_ = height;

	d_image_width_ = width;
	d_image_height_ = height;

	h_rgb_image_width_ = rgb_width;
	h_rgb_image_height_ = rgb_height;

	d_rgb_image_width_ = rgb_width;
	d_rgb_image_height_ = rgb_height;

	cudaError_t error_code = cudaMemcpyToSymbol(d_image_width_, &width, sizeof(int));
	if (error_code != cudaSuccess)
	{
		return false;
	}

	error_code = cudaMemcpyToSymbol(d_image_height_, &height, sizeof(int));
	if (error_code != cudaSuccess)
	{
		return false;
	}

	error_code = cudaMemcpyToSymbol(d_rgb_image_width_, &rgb_width, sizeof(int));
	if (error_code != cudaSuccess)
	{
		return false;
	}

	error_code = cudaMemcpyToSymbol(d_rgb_image_height_, &rgb_height, sizeof(int));
	if (error_code != cudaSuccess)
	{
		return false;
	}


	blocksPerGrid.x = (width + threadsPerBlock.x - 1) / threadsPerBlock.x;
	blocksPerGrid.y = (height + threadsPerBlock.y - 1) / threadsPerBlock.y;

	LOG(INFO) << "init d_image_width_: " << d_image_width_;
	LOG(INFO) << "init d_image_height_: " << d_image_height_;	
	LOG(INFO) << "init d_rgb_image_width_: " << d_rgb_image_width_;
	LOG(INFO) << "init d_rgb_image_height_: " << d_rgb_image_height_;
	LOG(INFO) << "blocksPerGrid.x: " << blocksPerGrid.x;
	LOG(INFO) << "blocksPerGrid.y: " << blocksPerGrid.y;

	return true;
}

bool cuda_malloc_basic_memory()
{
	for (int i = 0; i < MAX_PATTERNS_NUMBER; i++)
	{
		cudaMalloc((void**)&d_patterns_list_[0][i], d_image_height_ * d_image_width_ * sizeof(unsigned char));
	}
	for (int i = 0; i < MAX_PATTERNS_NUMBER; i++)
	{
		cudaMalloc((void**)&d_patterns_list_[1][i], d_image_height_ * d_image_width_ * sizeof(unsigned char));
	}

	for (int i = 0; i < MAX_PATTERNS_NUMBER; i++)
	{
		cudaMalloc((void**)&d_remap_patterns_list_[0][i], d_image_height_ * d_image_width_ * sizeof(unsigned char));
	}
	for (int i = 0; i < MAX_PATTERNS_NUMBER; i++)
	{
		cudaMalloc((void**)&d_remap_patterns_list_[1][i], d_image_height_ * d_image_width_ * sizeof(unsigned char));
	}

	for (int i = 0; i < MAX_PATTERNS_NUMBER; i++)
	{
		cudaMalloc((void**)&d_remap_patterns_list_16bit_[0][i], d_image_height_ * d_image_width_ * sizeof(unsigned short));
	}
	for (int i = 0; i < MAX_PATTERNS_NUMBER; i++)
	{
		cudaMalloc((void**)&d_remap_patterns_list_16bit_[1][i], d_image_height_ * d_image_width_ * sizeof(unsigned short));
	}

	for (int i = 0; i < MAX_PATTERNS_NUMBER; i++)
	{
		cudaMalloc((void**)&d_patterns_list_16bit_[0][i], d_image_height_ * d_image_width_ * sizeof(unsigned short));
	}
	for (int i = 0; i < MAX_PATTERNS_NUMBER; i++)
	{
		cudaMalloc((void**)&d_patterns_list_16bit_[1][i], d_image_height_ * d_image_width_ * sizeof(unsigned short));
	}

	cudaMalloc((void**)&d_brightness_map_[0], d_image_height_ * d_image_width_ * sizeof(unsigned char));
	cudaMalloc((void**)&d_brightness_map_[1], d_image_height_ * d_image_width_ * sizeof(unsigned char));

	for (int i = 0; i < D_HDR_MAX_NUM; i++)
	{
		cudaMalloc((void**)&d_hdr_brightness_map_[0][i], d_image_height_ * d_image_width_ * sizeof(unsigned char));
		cudaMalloc((void**)&d_hdr_brightness_map_[1][i], d_image_height_ * d_image_width_ * sizeof(unsigned char));

		cudaMalloc((void**)&d_hdr_bright_pixel_sum_list_[i], 1 * sizeof(float));
		cudaMalloc((void**)&d_hdr_brightness_list_[i], d_image_height_ * d_image_width_ * sizeof(unsigned char));
		cudaMalloc((void**)&d_hdr_depth_map_list_[i], d_image_height_ * d_image_width_ * sizeof(float));

		cudaMalloc((void**)&d_hdr_phase_map_[0][i], d_image_height_ * d_image_width_ * sizeof(unsigned char));
		cudaMalloc((void**)&d_hdr_phase_map_[1][i], d_image_height_ * d_image_width_ * sizeof(unsigned char));
	}

	cudaMalloc((void**)&d_depth_map_, d_image_height_ * d_image_width_ * sizeof(float));
	cudaMalloc((void**)&d_depth_map_temp_, d_image_height_ * d_image_width_ * sizeof(float));
	cudaMalloc((void**)&d_mask_map_, d_image_height_ * d_image_width_ * sizeof(unsigned char));
	cudaMalloc((void**)&d_depth_mask_, d_image_height_ * d_image_width_ * sizeof(unsigned char));
	cudaMalloc((void**)&d_disparty_map_, d_image_height_ * d_image_width_ * sizeof(float));
	cudaMalloc((void**)&d_pointcloud_map_, 3 * d_image_height_ * d_image_width_ * sizeof(float));


	cudaMalloc((void**)&d_noise_mask_map_[0], d_image_height_ * d_image_width_ * sizeof(unsigned char));
	cudaMalloc((void**)&d_noise_mask_map_[1], d_image_height_ * d_image_width_ * sizeof(unsigned char));

	cudaMalloc((void**)&d_disp_mask_map_, d_image_height_ * d_image_width_ * sizeof(unsigned char));

	cudaMalloc((void**)&d_threshold_map_[0], d_image_height_ * d_image_width_ * sizeof(unsigned char));
	cudaMalloc((void**)&d_threshold_map_[1], d_image_height_ * d_image_width_ * sizeof(unsigned char));

	cudaMalloc((void**)&d_threshold_map_16bit_[0], d_image_height_ * d_image_width_ * sizeof(unsigned short));
	cudaMalloc((void**)&d_threshold_map_16bit_[1], d_image_height_ * d_image_width_ * sizeof(unsigned short));

	cudaMalloc((void**)&d_phase_map_[0], d_image_height_ * d_image_width_ * sizeof(unsigned char));
	cudaMalloc((void**)&d_phase_map_[1], d_image_height_ * d_image_width_ * sizeof(unsigned char));

	cudaMalloc((void**)&d_code_map_[0], d_image_height_ * d_image_width_ * sizeof(unsigned char));
	cudaMalloc((void**)&d_code_map_[1], d_image_height_ * d_image_width_ * sizeof(unsigned char));

	cudaMalloc((void**)&d_gray_to_bin_decode_map_, 256 * sizeof(unsigned char));

	cudaMalloc((void**)&d_remap_xy_map[0], d_image_height_ * d_image_width_ * sizeof(short2));
	cudaMalloc((void**)&d_remap_xy_map[1], d_image_height_ * d_image_width_ * sizeof(short2));
	cudaMalloc((void**)&d_weight_index_map[0], d_image_height_ * d_image_width_ * sizeof(unsigned short));
	cudaMalloc((void**)&d_weight_index_map[1], d_image_height_ * d_image_width_ * sizeof(unsigned short));
	cudaMalloc((void**)&d_weight_map, 1024 * sizeof(short4));

	cudaMalloc((void**)&d_Q, 16 * sizeof(float));

	cudaMalloc((void**)&d_intrinsic_rgb, 3 * 3 * sizeof(float));
	cudaMalloc((void**)&d_R_l2rgb, 3 * 3 * sizeof(float));
	cudaMalloc((void**)&d_T_l2rgb, 3 * 1 * sizeof(float));
	cudaMalloc((void**)&d_depth2rgb_map, d_image_height_ * d_image_width_ * sizeof(ushort2));

	cudaMalloc((void**)&d_unwraped_pixels[0], d_image_height_ * d_image_width_ * sizeof(unsigned short));
	cudaMalloc((void**)&d_unwraped_pixels[1], d_image_height_ * d_image_width_ * sizeof(unsigned short));

	cudaMalloc((void**)&d_sorted_pixels[0], d_image_height_ * d_image_width_ * sizeof(unsigned short));
	cudaMalloc((void**)&d_sorted_pixels[1], d_image_height_ * d_image_width_ * sizeof(unsigned short));

	cudaMalloc((void**)&d_num_of_pixels_[0], d_image_height_ * 256 * sizeof(unsigned short));
	cudaMalloc((void**)&d_num_of_pixels_[1], d_image_height_ * 256 * sizeof(unsigned short));

	cudaMalloc((void**)&d_index_of_pixels_[0], d_image_height_ * 256 * sizeof(unsigned short));
	cudaMalloc((void**)&d_index_of_pixels_[1], d_image_height_ * 256 * sizeof(unsigned short));

	cudaMalloc((void**)&d_sheared_start_pixels[0], d_image_height_ * d_image_width_ * sizeof(unsigned short));
	cudaMalloc((void**)&d_sheared_start_pixels[1], d_image_height_ * d_image_width_ * sizeof(unsigned short));

	cudaMalloc((void**)&d_sheared_end_pixels[0], d_image_height_ * d_image_width_ * sizeof(unsigned short));
	cudaMalloc((void**)&d_sheared_end_pixels[1], d_image_height_ * d_image_width_ * sizeof(unsigned short));


	LOG(INFO) << "cudaMalloc finished!";
	LOG(INFO) << "d_image_height_: " << d_image_height_;
	LOG(INFO) << "d_image_width_: " << d_image_width_;

	cudaDeviceSynchronize();
	return true;
}

bool cuda_free_basic_memory()
{
	for (int i = 0; i < MAX_PATTERNS_NUMBER; i++)
	{
		cudaFree(d_patterns_list_[0][i]);
	}
	for (int i = 0; i < MAX_PATTERNS_NUMBER; i++)
	{
		cudaFree(d_patterns_list_[1][i]);
	}	
	for (int i = 0; i < MAX_PATTERNS_NUMBER; i++)
	{
		cudaFree(d_remap_patterns_list_[0][i]);
	}
	for (int i = 0; i < MAX_PATTERNS_NUMBER; i++)
	{
		cudaFree(d_remap_patterns_list_[1][i]);
	}	
	
	cudaFree(d_brightness_map_[0]);
	cudaFree(d_depth_map_);
	cudaFree(d_depth_map_temp_);
	cudaFree(d_mask_map_);
	cudaFree(d_depth_mask_);
	cudaFree(d_pointcloud_map_);

	cudaFree(d_noise_mask_map_[0]);
	cudaFree(d_noise_mask_map_[1]);

	cudaFree(d_disp_mask_map_);

	cudaFree(d_threshold_map_[0]);
	cudaFree(d_gray_to_bin_decode_map_);
	cudaFree(d_Q);

	cudaFree(d_intrinsic_rgb);
	cudaFree(d_R_l2rgb);
	cudaFree(d_T_l2rgb);
	cudaFree(d_depth2rgb_map);

	cudaFree(d_phase_map_[0]);

	cudaFree(d_unwraped_pixels[0]);
	cudaFree(d_sorted_pixels[0]);
	cudaFree(d_num_of_pixels_[0]);
	cudaFree(d_index_of_pixels_[0]);
	cudaFree(d_sheared_start_pixels[0]);
	cudaFree(d_sheared_end_pixels[0]);

	cudaFree(d_remap_xy_map[0]);
	cudaFree(d_remap_xy_map[1]);
	cudaFree(d_weight_index_map[0]);
	cudaFree(d_weight_index_map[1]);
	cudaFree(d_weight_map);

	return true;
}

bool cuda_init_basic_memory()
{
	cudaMemset(d_depth_map_, 0, d_image_height_ * d_image_width_ * sizeof(float));
	cudaMemset(d_mask_map_, 0, d_image_height_ * d_image_width_ * sizeof(unsigned char));
	cudaMemset(d_disparty_map_, 0, d_image_height_ * d_image_width_ * sizeof(float));
	cudaMemset(d_pointcloud_map_, 0, 3 * d_image_height_ * d_image_width_ * sizeof(float));

	cudaMemset(d_noise_mask_map_[0], 0, d_image_height_ * d_image_width_ * sizeof(unsigned char));
	cudaMemset(d_noise_mask_map_[1], 0, d_image_height_ * d_image_width_ * sizeof(unsigned char));

	cudaMemset(d_disp_mask_map_, 0, d_image_height_ * d_image_width_ * sizeof(unsigned char));
	
	cudaMemset(d_phase_map_[0], 0, d_image_height_ * d_image_width_ * sizeof(unsigned char));
	cudaMemset(d_phase_map_[1], 0, d_image_height_ * d_image_width_ * sizeof(unsigned char));
	
	cudaMemset(d_unwraped_pixels[0], 0, d_image_height_ * d_image_width_ * sizeof(unsigned short));
	cudaMemset(d_unwraped_pixels[1], 0, d_image_height_ * d_image_width_ * sizeof(unsigned short));

	cudaMemset(d_num_of_pixels_[0], 0, d_image_height_ * 256 * sizeof(unsigned short));
	cudaMemset(d_num_of_pixels_[1], 0, d_image_height_ * 256 * sizeof(unsigned short));
}

void cuda_clear_repetition_capture_cache()
{
	// 初始化unsigned short格式的remap之后的左右目结果
	for (int i = 0; i < MAX_PATTERNS_NUMBER; i += 1)
	{
		CHECK(cudaMemset(d_patterns_list_16bit_[0][i],0,d_image_height_*d_image_width_ * sizeof(unsigned short)));
		CHECK(cudaMemset(d_patterns_list_16bit_[1][i],0,d_image_height_*d_image_width_ * sizeof(unsigned short)));
	}
}

bool cuda_init_basic_memory_hdr()
{
	cudaMemset(d_depth_map_, 0, d_image_height_ * d_image_width_ * sizeof(float));
	cudaMemset(d_mask_map_, 0, d_image_height_ * d_image_width_ * sizeof(unsigned char));
	cudaMemset(d_disparty_map_, 0, d_image_height_ * d_image_width_ * sizeof(float));
	cudaMemset(d_pointcloud_map_, 0, 3 * d_image_height_ * d_image_width_ * sizeof(float));

	cudaMemset(d_noise_mask_map_[0], 0, d_image_height_ * d_image_width_ * sizeof(unsigned char));
	cudaMemset(d_noise_mask_map_[1], 0, d_image_height_ * d_image_width_ * sizeof(unsigned char));
	
	cudaMemset(d_disp_mask_map_, 0, d_image_height_ * d_image_width_ * sizeof(unsigned char));

	cudaMemset(d_phase_map_[0], 0, d_image_height_ * d_image_width_ * sizeof(unsigned char));
	cudaMemset(d_phase_map_[1], 0, d_image_height_ * d_image_width_ * sizeof(unsigned char));
	
	cudaMemset(d_unwraped_pixels[0], 0, d_image_height_ * d_image_width_ * sizeof(unsigned short));
	cudaMemset(d_unwraped_pixels[1], 0, d_image_height_ * d_image_width_ * sizeof(unsigned short));

	cudaMemset(d_num_of_pixels_[0], 0, d_image_height_ * 256 * sizeof(unsigned short));
	cudaMemset(d_num_of_pixels_[1], 0, d_image_height_ * 256 * sizeof(unsigned short));

	for (int i = 0; i < D_HDR_MAX_NUM; i += 1)
	{
		cudaMemset(d_hdr_brightness_map_[0][i], 0, d_image_height_ * d_image_width_ * sizeof(unsigned char));
		cudaMemset(d_hdr_brightness_map_[1][i], 0, d_image_height_ * d_image_width_ * sizeof(unsigned char));

		cudaMemset(d_hdr_brightness_map_[1][i], 0, d_image_height_ * d_image_width_ * sizeof(unsigned char));
		cudaMemset(d_hdr_brightness_map_[1][i], 0, d_image_height_ * d_image_width_ * sizeof(unsigned char));
		cudaMemset(d_hdr_brightness_map_[1][i], 0, d_image_height_ * d_image_width_ * sizeof(unsigned char));

	}
}

/********************************************************************/
bool cuda_copy_pattern_to_memory(unsigned char* pattern_ptr, int serial_flag)
{
	if (serial_flag > MAX_PATTERNS_NUMBER)
	{
		return false;
	}

	CHECK(cudaMemcpyAsync(d_patterns_list_[0][serial_flag], pattern_ptr, d_image_height_ * d_image_width_ * sizeof(unsigned char), cudaMemcpyHostToDevice));
}

bool cuda_copy_pattern_to_memory(unsigned char* pattern_ptr, int serial_flag, cudaStream_t stream)
{
	if (serial_flag < MAX_PATTERNS_NUMBER)
	{
		CHECK(cudaMemcpyAsync(d_remap_patterns_list_[0][serial_flag], pattern_ptr, d_image_height_ * d_image_width_ * sizeof(unsigned char), cudaMemcpyHostToDevice, stream));
		kernel_remap << <blocksPerGrid, threadsPerBlock, 0, stream >> > (d_remap_patterns_list_[0][serial_flag], d_patterns_list_[0][serial_flag], d_remap_xy_map[0], d_weight_index_map[0], d_weight_map, d_image_width_, d_image_height_);
	}
	else if (serial_flag < 2 * MAX_PATTERNS_NUMBER)
	{
		LOG(INFO) << "serial_flag - MAX_PATTERNS_NUMBER: " << serial_flag - MAX_PATTERNS_NUMBER;
		CHECK(cudaMemcpyAsync(d_remap_patterns_list_[1][serial_flag - MAX_PATTERNS_NUMBER], pattern_ptr, d_image_height_ * d_image_width_ * sizeof(unsigned char), cudaMemcpyHostToDevice, stream));
		kernel_remap << <blocksPerGrid, threadsPerBlock, 0, stream >> > (d_remap_patterns_list_[1][serial_flag - MAX_PATTERNS_NUMBER], d_patterns_list_[1][serial_flag - MAX_PATTERNS_NUMBER], d_remap_xy_map[1], d_weight_index_map[1], d_weight_map, d_image_width_, d_image_height_);
	}
	else
	{
		return false;
	}

}

bool cuda_copy_pattern_to_memory(unsigned short* pattern_ptr, int serial_flag, cudaStream_t stream)
{
	if (serial_flag < MAX_PATTERNS_NUMBER)
	{
		CHECK(cudaMemcpyAsync(d_remap_patterns_list_16bit_[0][serial_flag], pattern_ptr, d_image_height_ * d_image_width_ * sizeof(unsigned short), cudaMemcpyHostToDevice, stream));
		kernel_remap << <blocksPerGrid, threadsPerBlock, 0, stream >> > (d_remap_patterns_list_16bit_[0][serial_flag], d_patterns_list_16bit_[0][serial_flag], d_remap_xy_map[0], d_weight_index_map[0], d_weight_map, d_image_width_, d_image_height_);
	}
	else if (serial_flag < 2 * MAX_PATTERNS_NUMBER)
	{
		LOG(INFO) << "serial_flag - MAX_PATTERNS_NUMBER: " << serial_flag - MAX_PATTERNS_NUMBER;
		CHECK(cudaMemcpyAsync(d_remap_patterns_list_16bit_[1][serial_flag - MAX_PATTERNS_NUMBER], pattern_ptr, d_image_height_ * d_image_width_ * sizeof(unsigned short), cudaMemcpyHostToDevice, stream));
		kernel_remap << <blocksPerGrid, threadsPerBlock, 0, stream >> > (d_remap_patterns_list_16bit_[1][serial_flag - MAX_PATTERNS_NUMBER], d_patterns_list_16bit_[1][serial_flag - MAX_PATTERNS_NUMBER], d_remap_xy_map[1], d_weight_index_map[1], d_weight_map, d_image_width_, d_image_height_);
	}
	else
	{
		return false;
	}
	
	return true;

}

bool cuda_copy_repetition_pattern_to_memory(unsigned short* pattern_ptr, int serial_flag, cudaStream_t stream)
{
	if (serial_flag < MAX_PATTERNS_NUMBER)
	{
		CHECK(cudaMemcpyAsync(d_remap_patterns_list_16bit_[0][serial_flag], pattern_ptr, d_image_height_ * d_image_width_ * sizeof(unsigned short), cudaMemcpyHostToDevice, stream));
		kernel_remap_repetition_mode << <blocksPerGrid, threadsPerBlock, 0, stream >> > (d_remap_patterns_list_16bit_[0][serial_flag], d_patterns_list_16bit_[0][serial_flag], d_remap_xy_map[0], d_weight_index_map[0], d_weight_map, d_image_width_, d_image_height_);
	}
	else if (serial_flag < 2 * MAX_PATTERNS_NUMBER)
	{
		LOG(INFO) << "serial_flag - MAX_PATTERNS_NUMBER: " << serial_flag - MAX_PATTERNS_NUMBER;
		CHECK(cudaMemcpyAsync(d_remap_patterns_list_16bit_[1][serial_flag - MAX_PATTERNS_NUMBER], pattern_ptr, d_image_height_ * d_image_width_ * sizeof(unsigned short), cudaMemcpyHostToDevice, stream));
		kernel_remap_repetition_mode << <blocksPerGrid, threadsPerBlock, 0, stream >> > (d_remap_patterns_list_16bit_[1][serial_flag - MAX_PATTERNS_NUMBER], d_patterns_list_16bit_[1][serial_flag - MAX_PATTERNS_NUMBER], d_remap_xy_map[1], d_weight_index_map[1], d_weight_map, d_image_width_, d_image_height_);
	}
	else
	{
		return false;
	}
	
	return true;

}

bool cuda_normalize_repetition_patterns(int count)
{
	for (int i = 0; i < MAX_PATTERNS_NUMBER; i += 1)
	{
		kernel_normalize_repetition_patterns << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list_16bit_[0][i], d_patterns_list_16bit_[0][i], count, d_image_width_, d_image_height_);
		kernel_normalize_repetition_patterns << <blocksPerGrid, threadsPerBlock >> > (d_patterns_list_16bit_[1][i], d_patterns_list_16bit_[1][i], count, d_image_width_, d_image_height_);
	}
}

bool cuda_copy_decode_map_to_memory(unsigned char* decode_map_ptr)
{
	CHECK(cudaMemcpyAsync(d_gray_to_bin_decode_map_, decode_map_ptr, 256 * 1 * sizeof(unsigned char), cudaMemcpyHostToDevice));
}

bool cuda_copy_remap_maps_to_memory(short2* remap_xy_map_l, short2* remap_xy_map_r, unsigned short* weight_index_map_l, unsigned short* weight_index_map_r, short4* weight_map)
{
	CHECK(cudaMemcpyAsync(d_remap_xy_map[0], remap_xy_map_l, d_image_height_ * d_image_width_ * sizeof(short2), cudaMemcpyHostToDevice));

	CHECK(cudaMemcpyAsync(d_remap_xy_map[1], remap_xy_map_r, d_image_height_ * d_image_width_ * sizeof(short2), cudaMemcpyHostToDevice));

	CHECK(cudaMemcpyAsync(d_weight_index_map[0], weight_index_map_l, d_image_height_ * d_image_width_ * sizeof(unsigned short), cudaMemcpyHostToDevice));

	CHECK(cudaMemcpyAsync(d_weight_index_map[1], weight_index_map_r, d_image_height_ * d_image_width_ * sizeof(unsigned short), cudaMemcpyHostToDevice));

	CHECK(cudaMemcpyAsync(d_weight_map, weight_map, 1024 * sizeof(short4), cudaMemcpyHostToDevice));
}

bool cuda_copy_Q_map_to_memory(float* Q_map)
{
	CHECK(cudaMemcpyAsync(d_Q, Q_map, 16 * sizeof(float), cudaMemcpyHostToDevice));
}

bool cuda_copy_rgb_transform_to_memory(float* input_rgb_camera_intrinsic, float* input_l2rgb_R, float* input_l2rgb_T)
{
	CHECK(cudaMemcpyAsync(d_intrinsic_rgb, input_rgb_camera_intrinsic, 3 * 3 * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpyAsync(d_R_l2rgb, input_l2rgb_R, 3 * 3 * sizeof(float), cudaMemcpyHostToDevice));
	CHECK(cudaMemcpyAsync(d_T_l2rgb, input_l2rgb_T, 3 * 1 * sizeof(float), cudaMemcpyHostToDevice));
}

void cuda_copy_depth_from_memory(float* depth)
{
	CHECK(cudaMemcpy(depth, d_depth_map_, d_image_height_ * d_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
}

void cuda_copy_brightness_from_memory(unsigned char* brightness)
{
	CHECK(cudaMemcpy(brightness, d_patterns_list_[0][5], d_image_height_ * d_image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
}

void cuda_copy_brightness_from_memory(unsigned char* brightness, int index)
{
	if (index < 14)
	{
		CHECK(cudaMemcpy(brightness, d_patterns_list_[0][index], d_image_height_ * d_image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
	}
	else
	{
		CHECK(cudaMemcpy(brightness, d_patterns_list_[1][index - 14], d_image_height_ * d_image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
	}
}

void cuda_copy_brightness_to_memory(unsigned char* brightness)
{
	CHECK(cudaMemcpyAsync(d_brightness_map_[0], brightness, d_image_height_ * d_image_width_ * sizeof(unsigned char), cudaMemcpyHostToDevice));
}

void cuda_copy_code_from_memory(unsigned char* code)
{
	CHECK(cudaMemcpy(code, d_code_map_[1], d_image_height_ * d_image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
}

void cuda_copy_code_to_memory(unsigned char* code)
{
	CHECK(cudaMemcpyAsync(d_code_map_[0], code, d_image_height_ * d_image_width_ * sizeof(unsigned char), cudaMemcpyHostToDevice));
}

void cuda_copy_phase_from_memory(unsigned char* phase)
{
	CHECK(cudaMemcpy(phase, d_phase_map_[1], d_image_height_ * d_image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
}

void cuda_copy_phase_to_memory(unsigned char* phase)
{
	CHECK(cudaMemcpyAsync(d_phase_map_[0], phase, d_image_height_ * d_image_width_ * sizeof(unsigned char), cudaMemcpyHostToDevice));
}

void cuda_copy_code_statistics_from_memory(unsigned short* code_statistics)
{
	CHECK(cudaMemcpy(code_statistics, d_index_of_pixels_[0], d_image_height_ * 256 * sizeof(unsigned short), cudaMemcpyDeviceToHost));
}

void cuda_copy_code_sorted_index_from_memory(unsigned short* code_sorted_index)
{
	CHECK(cudaMemcpy(code_sorted_index, d_unwraped_pixels[0], d_image_height_ * h_image_width_ * sizeof(unsigned short), cudaMemcpyDeviceToHost));
}

void cuda_copy_disparty_from_memory(float* disparty)
{
	CHECK(cudaMemcpy(disparty, d_disparty_map_, d_image_height_ * h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
}

void cuda_copy_depth2rgb_map_from_memory(ushort2* output_depth2rgb_map)
{
	CHECK(cudaMemcpy(output_depth2rgb_map, d_depth2rgb_map, d_image_height_ * d_image_width_ * sizeof(ushort2), cudaMemcpyDeviceToHost));
}

/********************************************************************/

bool cuda_decode_gray_code(int serial_flag)
{
	cudaDeviceSynchronize();

	switch (serial_flag)
	{
	case 8:
	{
		LOG(INFO) << "start kernel_decode_gray_code_8bit()";
		kernel_decode_gray_code_8bit << <blocksPerGrid, threadsPerBlock >> > (d_image_width_, d_image_height_, d_gray_to_bin_decode_map_, d_patterns_list_[0][4], d_patterns_list_[0][5], d_patterns_list_[0][6], d_patterns_list_[0][7], d_patterns_list_[0][8], d_patterns_list_[0][9], d_patterns_list_[0][10], d_patterns_list_[0][11], d_patterns_list_[0][12], d_patterns_list_[0][13], d_code_map_[0], d_noise_mask_map_[0]);
		cudaDeviceSynchronize();
		LOG(INFO) << "end kernel_decode_gray_code_8bit()";

		LOG(INFO) << "start kernel_4_step_phase_shift_8bit()";
		kernel_4_step_phase_shift_8bit << <blocksPerGrid, threadsPerBlock >> > (d_image_width_, d_image_height_, d_patterns_list_[0][1], d_patterns_list_[0][2], d_patterns_list_[0][3], d_patterns_list_[0][0], d_phase_map_[0], d_noise_mask_map_[0]);
		cudaDeviceSynchronize();
		LOG(INFO) << "end kernel_4_step_phase_shift_8bit()";

		LOG(INFO) << "start kernel_code_rectify_8bit()";
		kernel_code_rectify_8bit << <blocksPerGrid, threadsPerBlock >> > (d_image_width_, d_image_width_, d_code_map_[0], d_phase_map_[0], d_noise_mask_map_[0]);
		cudaDeviceSynchronize();
		LOG(INFO) << "end kernel_code_rectify_8bit()";
	}
	break;
	default:
		break;
	}
	return true;
}

bool cuda_decode_gray_code_one_by_one(int serial_flag)
{
	switch (serial_flag)
	{
	case -1:
	{
		kernel_decode_threshold_and_mask << <blocksPerGrid, threadsPerBlock >> > (d_image_width_, d_image_height_, d_confidence_, d_patterns_list_[0][4], d_patterns_list_[0][5], d_threshold_map_[0], d_noise_mask_map_[0], d_code_map_[0]);
	}
	break;
	case 0:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock >> > (d_image_width_, d_image_height_, d_threshold_map_[0], d_patterns_list_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);

	}
	break;
	case 1:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock >> > (d_image_width_, d_image_height_, d_threshold_map_[0], d_patterns_list_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);
	}
	break;
	case 2:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock >> > (d_image_width_, d_image_height_, d_threshold_map_[0], d_patterns_list_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);
	}
	break;
	case 3:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock >> > (d_image_width_, d_image_height_, d_threshold_map_[0], d_patterns_list_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);
	}
	break;
	case 4:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock >> > (d_image_width_, d_image_height_, d_threshold_map_[0], d_patterns_list_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);
	}
	break;
	case 5:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock >> > (d_image_width_, d_image_height_, d_threshold_map_[0], d_patterns_list_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);
	}
	break;
	case 6:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock >> > (d_image_width_, d_image_height_, d_threshold_map_[0], d_patterns_list_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);
	}
	break;
	case 7:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock >> > (d_image_width_, d_image_height_, d_threshold_map_[0], d_patterns_list_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);

		//kernel_gray_code_to_bin_code << <blocksPerGrid, threadsPerBlock >> > (d_image_width_, d_image_height_, d_code_map_[0], d_gray_to_bin_decode_map_, d_noise_mask_map_[0]);
	}
	break;
	default:
		break;
	}
	return true;
}

bool cuda_decode_gray_code_one_by_one(int serial_flag, cudaStream_t stream_left, cudaStream_t stream_right)
{
	switch (serial_flag)
	{
	case -1:
	{
		kernel_decode_threshold_and_mask << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_confidence_, d_patterns_list_[0][4], d_patterns_list_[0][5], d_threshold_map_[0], d_noise_mask_map_[0], d_code_map_[0]);
		kernel_decode_threshold_and_mask << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_confidence_, d_patterns_list_[1][4], d_patterns_list_[1][5], d_threshold_map_[1], d_noise_mask_map_[1], d_code_map_[1]);
	}
	break;
	case 0:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_threshold_map_[0], d_patterns_list_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_threshold_map_[1], d_patterns_list_[1][serial_flag + 6], d_code_map_[1], d_noise_mask_map_[1]);

	}
	break;
	case 1:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_threshold_map_[0], d_patterns_list_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_threshold_map_[1], d_patterns_list_[1][serial_flag + 6], d_code_map_[1], d_noise_mask_map_[1]);
	}
	break;
	case 2:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_threshold_map_[0], d_patterns_list_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_threshold_map_[1], d_patterns_list_[1][serial_flag + 6], d_code_map_[1], d_noise_mask_map_[1]);
	}
	break;
	case 3:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_threshold_map_[0], d_patterns_list_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_threshold_map_[1], d_patterns_list_[1][serial_flag + 6], d_code_map_[1], d_noise_mask_map_[1]);
	}
	break;
	case 4:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_threshold_map_[0], d_patterns_list_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_threshold_map_[1], d_patterns_list_[1][serial_flag + 6], d_code_map_[1], d_noise_mask_map_[1]);
	}
	break;
	case 5:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_threshold_map_[0], d_patterns_list_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_threshold_map_[1], d_patterns_list_[1][serial_flag + 6], d_code_map_[1], d_noise_mask_map_[1]);
	}
	break;
	case 6:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_threshold_map_[0], d_patterns_list_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_threshold_map_[1], d_patterns_list_[1][serial_flag + 6], d_code_map_[1], d_noise_mask_map_[1]);
	}
	break;
	case 7:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_threshold_map_[0], d_patterns_list_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_threshold_map_[1], d_patterns_list_[1][serial_flag + 6], d_code_map_[1], d_noise_mask_map_[1]);

		kernel_gray_code_to_bin_code << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_code_map_[0], d_gray_to_bin_decode_map_, d_noise_mask_map_[0], d_patterns_list_[0][4]);
		kernel_gray_code_to_bin_code << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_code_map_[1], d_gray_to_bin_decode_map_, d_noise_mask_map_[1], d_patterns_list_[1][4]);
	}
	break;
	default:
		break;
	}
	return true;
}

bool cuda_decode_gray_code_one_by_one_16bit(int serial_flag, cudaStream_t stream_left, cudaStream_t stream_right)
{
	switch (serial_flag)
	{
	case -1:
	{
		// kernel_decode_threshold_and_mask << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_confidence_, d_patterns_list_[0][4], d_patterns_list_[0][5], d_threshold_map_16bit_[0], d_noise_mask_map_[0], d_code_map_[0]);
		// kernel_decode_threshold_and_mask << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_confidence_, d_patterns_list_[1][4], d_patterns_list_[1][5], d_threshold_map_16bit_[1], d_noise_mask_map_[1], d_code_map_[1]);
		LOG(INFO) << "d_camera_gamma_: " << d_camera_gamma_;
		kernel_convert_brightness_to_8bit << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_patterns_list_16bit_[0][4], d_patterns_list_[0][5], d_code_map_[0], d_camera_gamma_);
		kernel_convert_brightness_to_8bit << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_patterns_list_16bit_[1][4], d_patterns_list_[1][5], d_code_map_[1], d_camera_gamma_);


	}
	break;
	case 0:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_threshold_map_16bit_[0], d_patterns_list_16bit_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_threshold_map_16bit_[1], d_patterns_list_16bit_[1][serial_flag + 6], d_code_map_[1], d_noise_mask_map_[1]);

	}
	break;
	case 1:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_threshold_map_16bit_[0], d_patterns_list_16bit_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_threshold_map_16bit_[1], d_patterns_list_16bit_[1][serial_flag + 6], d_code_map_[1], d_noise_mask_map_[1]);
	}
	break;
	case 2:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_threshold_map_16bit_[0], d_patterns_list_16bit_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_threshold_map_16bit_[1], d_patterns_list_16bit_[1][serial_flag + 6], d_code_map_[1], d_noise_mask_map_[1]);
	}
	break;
	case 3:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_threshold_map_16bit_[0], d_patterns_list_16bit_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_threshold_map_16bit_[1], d_patterns_list_16bit_[1][serial_flag + 6], d_code_map_[1], d_noise_mask_map_[1]);
	}
	break;
	case 4:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_threshold_map_16bit_[0], d_patterns_list_16bit_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_threshold_map_16bit_[1], d_patterns_list_16bit_[1][serial_flag + 6], d_code_map_[1], d_noise_mask_map_[1]);
	}
	break;
	case 5:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_threshold_map_16bit_[0], d_patterns_list_16bit_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_threshold_map_16bit_[1], d_patterns_list_16bit_[1][serial_flag + 6], d_code_map_[1], d_noise_mask_map_[1]);
	}
	break;
	case 6:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_threshold_map_16bit_[0], d_patterns_list_16bit_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_threshold_map_16bit_[1], d_patterns_list_16bit_[1][serial_flag + 6], d_code_map_[1], d_noise_mask_map_[1]);
	}
	break;
	case 7:
	{
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_threshold_map_16bit_[0], d_patterns_list_16bit_[0][serial_flag + 6], d_code_map_[0], d_noise_mask_map_[0]);
		kernel_decode_gray_code_one_by_one << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_threshold_map_16bit_[1], d_patterns_list_16bit_[1][serial_flag + 6], d_code_map_[1], d_noise_mask_map_[1]);

		kernel_gray_code_to_bin_code << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_code_map_[0], d_gray_to_bin_decode_map_, d_noise_mask_map_[0], d_patterns_list_[0][4]);
		kernel_gray_code_to_bin_code << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_code_map_[1], d_gray_to_bin_decode_map_, d_noise_mask_map_[1], d_patterns_list_[1][4]);
	}
	break;
	default:
		break;
	}
	return true;
}

bool cuda_four_step_phase_shift()
{
	kernel_4_step_phase_shift_8bit << <blocksPerGrid, threadsPerBlock >> > (d_image_width_, d_image_height_, d_patterns_list_[0][1], d_patterns_list_[0][2], d_patterns_list_[0][3], d_patterns_list_[0][0], d_phase_map_[0], d_noise_mask_map_[0]);
	kernel_4_step_phase_shift_8bit << <blocksPerGrid, threadsPerBlock >> > (d_image_width_, d_image_height_, d_patterns_list_[1][1], d_patterns_list_[1][2], d_patterns_list_[1][3], d_patterns_list_[1][0], d_phase_map_[1], d_noise_mask_map_[1]);
	return true;
}

bool cuda_four_step_phase_shift(cudaStream_t stream_left, cudaStream_t stream_right)
{
	kernel_4_step_phase_shift_8bit << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_patterns_list_[0][0], d_patterns_list_[0][1], d_patterns_list_[0][2], d_patterns_list_[0][3], d_phase_map_[0], d_noise_mask_map_[0], d_threshold_map_[0], d_confidence_);
	kernel_4_step_phase_shift_8bit << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_patterns_list_[1][0], d_patterns_list_[1][1], d_patterns_list_[1][2], d_patterns_list_[1][3], d_phase_map_[1], d_noise_mask_map_[1], d_threshold_map_[1], d_confidence_);
	return true;
}

bool cuda_four_step_phase_shift_16bit(cudaStream_t stream_left, cudaStream_t stream_right)
{
	kernel_4_step_phase_shift_16bit << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_patterns_list_16bit_[0][0], d_patterns_list_16bit_[0][1], d_patterns_list_16bit_[0][2], d_patterns_list_16bit_[0][3], d_phase_map_[0], d_noise_mask_map_[0], d_threshold_map_16bit_[0], d_confidence_);
	kernel_4_step_phase_shift_16bit << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_patterns_list_16bit_[1][0], d_patterns_list_16bit_[1][1], d_patterns_list_16bit_[1][2], d_patterns_list_16bit_[1][3], d_phase_map_[1], d_noise_mask_map_[1], d_threshold_map_16bit_[1], d_confidence_);
	return true;
}

bool cuda_eight_step_phase_shift_16bit(cudaStream_t stream_left, cudaStream_t stream_right)
{
	kernel_8_step_phase_shift_16bit << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_patterns_list_16bit_[0][0], d_patterns_list_16bit_[0][1], d_patterns_list_16bit_[0][2], d_patterns_list_16bit_[0][3], d_patterns_list_16bit_[0][14], d_patterns_list_16bit_[0][15], d_patterns_list_16bit_[0][16], d_patterns_list_16bit_[0][17], d_phase_map_[0], d_noise_mask_map_[0], d_threshold_map_16bit_[0], d_confidence_);
	kernel_8_step_phase_shift_16bit << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_patterns_list_16bit_[1][0], d_patterns_list_16bit_[1][1], d_patterns_list_16bit_[1][2], d_patterns_list_16bit_[1][3], d_patterns_list_16bit_[1][14], d_patterns_list_16bit_[1][15], d_patterns_list_16bit_[1][16], d_patterns_list_16bit_[1][17], d_phase_map_[1], d_noise_mask_map_[1], d_threshold_map_16bit_[1], d_confidence_);
	return true;
}

bool cuda_four_step_phase_shift(cudaStream_t stream_left, cudaStream_t stream_right, int serial_flag)
{
	kernel_4_step_phase_shift_8bit << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_patterns_list_[0][1], d_patterns_list_[0][2], d_patterns_list_[0][3], d_patterns_list_[0][0], d_hdr_phase_map_[0][serial_flag], d_hdr_brightness_map_[0][serial_flag]);
	kernel_4_step_phase_shift_8bit << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_patterns_list_[1][1], d_patterns_list_[1][2], d_patterns_list_[1][3], d_patterns_list_[1][0], d_hdr_phase_map_[1][serial_flag], d_hdr_brightness_map_[1][serial_flag]);
	return true;
}

bool cuda_code_phase_rectify()
{
	kernel_code_rectify_8bit << <blocksPerGrid, threadsPerBlock >> > (d_image_width_, d_image_height_, d_code_map_[0], d_phase_map_[0], d_noise_mask_map_[0]);
	return true;
}

bool cuda_code_phase_rectify(cudaStream_t stream_left, cudaStream_t stream_right)
{
	kernel_code_rectify_8bit << <blocksPerGrid, threadsPerBlock, 0, stream_left >> > (d_image_width_, d_image_height_, d_code_map_[0], d_phase_map_[0], d_noise_mask_map_[0]);
	kernel_code_rectify_8bit << <blocksPerGrid, threadsPerBlock, 0, stream_right >> > (d_image_width_, d_image_height_, d_code_map_[1], d_phase_map_[1], d_noise_mask_map_[1]);
	return true;
}

bool cuda_code_phase_unwrap(int serial_flag)
{
	kernel_code_phase_unwrap_8bit << <blocksPerGrid, threadsPerBlock>> > (d_image_width_, d_image_height_, d_code_map_[0], d_phase_map_[0], d_unwraped_pixels[0], d_noise_mask_map_[0]);
	kernel_code_phase_unwrap_8bit << <blocksPerGrid, threadsPerBlock>> > (d_image_width_, d_image_height_, d_code_map_[1], d_phase_map_[1], d_unwraped_pixels[1], d_noise_mask_map_[1]);
	return true;
}

bool cuda_code_statistics(int serial_flag)
{
	dim3 threadsPerBlockTemp(4, 8);
	dim3 blocksPerGridTemp((32 + threadsPerBlockTemp.x - 1) / (threadsPerBlockTemp.x), (48 + threadsPerBlockTemp.y - 1) / threadsPerBlockTemp.y);

	kernel_code_statistics << <blocksPerGridTemp, threadsPerBlockTemp>> > (d_image_width_, d_image_height_, d_code_map_[0], d_num_of_pixels_[0]);
	kernel_code_statistics << <blocksPerGridTemp, threadsPerBlockTemp>> > (d_image_width_, d_image_height_, d_code_map_[1], d_num_of_pixels_[1]);

	return true;

}

bool cuda_code_statistics_to_index(int serial_flag)
{
	dim3 threadsPerBlockTemp(4, 8);
	dim3 blocksPerGridTemp((32 + threadsPerBlockTemp.x - 1) / (threadsPerBlockTemp.x), (48 + threadsPerBlockTemp.y - 1) / threadsPerBlockTemp.y);

	kernel_code_statistics_to_index << <blocksPerGridTemp, threadsPerBlockTemp >> > (d_image_width_, d_image_height_, d_num_of_pixels_[0], d_index_of_pixels_[0]);
	kernel_code_statistics_to_index << <blocksPerGridTemp, threadsPerBlockTemp >> > (d_image_width_, d_image_height_, d_num_of_pixels_[1], d_index_of_pixels_[1]);

	return true;
}

bool cuda_pixels_sort_by_code(int serial_flag)
{
	dim3 threadsPerBlockTemp(4, 8);
	dim3 blocksPerGridTemp((32 + threadsPerBlockTemp.x - 1) / (threadsPerBlockTemp.x), (48 + threadsPerBlockTemp.y - 1) / threadsPerBlockTemp.y);

	kernel_sort_code << <blocksPerGridTemp, threadsPerBlockTemp >> > (d_image_width_, d_image_height_, d_code_map_[0], d_sorted_pixels[0], d_index_of_pixels_[0], d_noise_mask_map_[0]);
	kernel_sort_code << <blocksPerGridTemp, threadsPerBlockTemp >> > (d_image_width_, d_image_height_, d_code_map_[1], d_sorted_pixels[1], d_index_of_pixels_[1], d_noise_mask_map_[0]);

	return true;
}

bool cuda_pixels_shear_by_monotonicity(int serial_flag)
{
	dim3 threadsPerBlockTemp(4, 8);
	dim3 blocksPerGridTemp((256 + threadsPerBlockTemp.x - 1) / (threadsPerBlockTemp.x), (d_image_width_ + threadsPerBlockTemp.y - 1) / threadsPerBlockTemp.y);
	//kernel_filter_code_noise << <blocksPerGridTemp, threadsPerBlockTemp >> > (d_image_width_, d_image_height_, d_unwraped_pixels[0], d_num_of_pixels_[0], d_index_of_pixels_[0], d_sorted_pixels[0]);
	kernel_filter_code_noise << <blocksPerGridTemp, threadsPerBlockTemp >> > (d_image_width_, d_image_height_, d_unwraped_pixels[1], d_num_of_pixels_[1], d_index_of_pixels_[1], d_sorted_pixels[1]);

	return true;
}

bool cuda_matching(int serial_flag)
{
	dim3 threadsPerBlockTemp(4, 8);
	dim3 blocksPerGridTemp((256 + threadsPerBlockTemp.x - 1) / (threadsPerBlockTemp.x), (d_image_width_ + threadsPerBlockTemp.y - 1) / threadsPerBlockTemp.y);
	kernel_matching << <blocksPerGridTemp, threadsPerBlockTemp >> > (d_image_width_, d_image_height_, d_sorted_pixels[0], d_unwraped_pixels[0], d_num_of_pixels_[0], d_index_of_pixels_[0], d_sorted_pixels[1], d_unwraped_pixels[1], d_num_of_pixels_[1], d_index_of_pixels_[1], d_disparty_map_, d_disp_mask_map_);

	return true;
}

bool cuda_disp_to_depth(int serial_flag)
{
	kernel_dispaty_to_depth << <blocksPerGrid, threadsPerBlock >> > (d_image_width_, d_image_height_, d_Q, d_disparty_map_, d_depth_map_, d_disp_mask_map_);
	return true;
}

bool cuda_disp_to_depth_and_color_map(int serial_flag)
{
	kernel_dispaty_to_depth_and_color_map << <blocksPerGrid, threadsPerBlock >> > (d_image_width_, d_image_height_, d_rgb_image_width_, d_rgb_image_height_, d_Q, d_intrinsic_rgb, d_R_l2rgb, d_T_l2rgb, d_disparty_map_, d_depth_map_, d_depth2rgb_map, d_disp_mask_map_);
	return true;
}

void depth_filter(float depth_threshold_val)
{
	dim3 threadsPerBlock_p(4, 8);
	dim3 blocksPerGrid_p;
	if (1200 == h_image_height_)
	{
		blocksPerGrid_p.x = (40 + threadsPerBlock_p.x - 1) / threadsPerBlock_p.x;
		blocksPerGrid_p.y = (30 + threadsPerBlock_p.y - 1) / threadsPerBlock_p.y;
	}
	else if (2048 == h_image_height_)
	{
		blocksPerGrid_p.x = (64 + threadsPerBlock_p.x - 1) / threadsPerBlock_p.x;
		blocksPerGrid_p.y = (32 + threadsPerBlock_p.y - 1) / threadsPerBlock_p.y;
	}
	else if (1536 == h_image_height_)
	{
		blocksPerGrid_p.x = (32 + threadsPerBlock_p.x - 1) / threadsPerBlock_p.x;
		blocksPerGrid_p.y = (48 + threadsPerBlock_p.y - 1) / threadsPerBlock_p.y;
	}
	else if (1240 == h_image_height_)
	{
		blocksPerGrid_p.x = (32 + threadsPerBlock_p.x - 1) / threadsPerBlock_p.x;
		blocksPerGrid_p.y = (39 + threadsPerBlock_p.y - 1) / threadsPerBlock_p.y;
	}
	LOG(INFO) << "depth filter start ";
	kernel_depth_filter_step_1 << < blocksPerGrid_p, threadsPerBlock_p >> > (d_image_height_, h_image_width_, depth_threshold_val, d_depth_map_, d_depth_map_temp_, d_depth_mask_);//
	kernel_depth_filter_step_2 << < blocksPerGrid_p, threadsPerBlock_p >> > (d_image_height_, h_image_width_, depth_threshold_val, d_depth_map_, d_depth_map_temp_, d_depth_mask_);
	cudaDeviceSynchronize();
	LOG(INFO) << "depth filter end";
}

bool cuda_hdr_sort_phase(int serial_flag)
{
	// 五个模式

	std::vector<int> id;

	for (int i = 0; i < serial_flag; i += 1)
	{
		id.push_back(serial_flag - 1 - i);
	}

	switch (serial_flag)
	{
	case 1:
	{
		CHECK(cudaMemcpy(d_phase_map_[0], d_hdr_phase_map_[0][0], 1 * h_image_height_ * h_image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToDevice));
		CHECK(cudaMemcpy(d_phase_map_[1], d_hdr_phase_map_[1][0], 1 * h_image_height_ * h_image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToDevice));
	}
	break;
	case 2:
	{
		cuda_merge_hdr_2 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_phase_map_[0][id[0]], d_hdr_phase_map_[0][id[1]], d_hdr_brightness_map_[0][id[0]],
			d_hdr_brightness_map_[0][id[1]], h_image_height_, h_image_width_, d_phase_map_[0]);
		cuda_merge_hdr_2 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_phase_map_[1][id[0]], d_hdr_phase_map_[1][id[1]], d_hdr_brightness_map_[1][id[0]],
			d_hdr_brightness_map_[1][id[1]], h_image_height_, h_image_width_, d_phase_map_[1]);

	}
	break;
	case 3:
	{
		cuda_merge_hdr_3 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_phase_map_[0][id[0]], d_hdr_phase_map_[0][id[1]], d_hdr_phase_map_[0][id[2]], d_hdr_brightness_map_[0][id[0]],
			d_hdr_brightness_map_[0][id[1]], d_hdr_brightness_map_[0][id[2]], h_image_height_, h_image_width_, d_phase_map_[0]);
		cuda_merge_hdr_3 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_phase_map_[1][id[0]], d_hdr_phase_map_[1][id[1]], d_hdr_phase_map_[1][id[2]], d_hdr_brightness_map_[1][id[0]],
			d_hdr_brightness_map_[1][id[1]], d_hdr_brightness_map_[1][id[2]], h_image_height_, h_image_width_, d_phase_map_[1]);

	}
	break;
	case 4:
	{
		cuda_merge_hdr_4 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_phase_map_[0][id[0]], d_hdr_phase_map_[0][id[1]], d_hdr_phase_map_[0][id[2]], d_hdr_phase_map_[0][id[3]], d_hdr_brightness_map_[0][id[0]],
			d_hdr_brightness_map_[0][id[1]], d_hdr_brightness_map_[0][id[2]], d_hdr_brightness_map_[0][id[3]], h_image_height_, h_image_width_, d_phase_map_[0]);
		cuda_merge_hdr_4 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_phase_map_[1][id[0]], d_hdr_phase_map_[1][id[1]], d_hdr_phase_map_[1][id[2]], d_hdr_phase_map_[1][id[3]], d_hdr_brightness_map_[1][id[0]],
			d_hdr_brightness_map_[1][id[1]], d_hdr_brightness_map_[1][id[2]], d_hdr_brightness_map_[1][id[3]], h_image_height_, h_image_width_, d_phase_map_[1]);

	}
	break;
	case 5:
	{
		cuda_merge_hdr_5 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_phase_map_[0][id[0]], d_hdr_phase_map_[0][id[1]], d_hdr_phase_map_[0][id[2]], d_hdr_phase_map_[0][id[3]], d_hdr_phase_map_[0][id[4]], d_hdr_brightness_map_[0][id[0]],
			d_hdr_brightness_map_[0][id[1]], d_hdr_brightness_map_[0][id[2]], d_hdr_brightness_map_[0][id[3]], d_hdr_brightness_map_[0][id[4]], h_image_height_, h_image_width_, d_phase_map_[0]);
		cuda_merge_hdr_5 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_phase_map_[1][id[0]], d_hdr_phase_map_[1][id[1]], d_hdr_phase_map_[1][id[2]], d_hdr_phase_map_[1][id[3]], d_hdr_phase_map_[1][id[4]], d_hdr_brightness_map_[1][id[0]],
			d_hdr_brightness_map_[1][id[1]], d_hdr_brightness_map_[1][id[2]], d_hdr_brightness_map_[1][id[3]], d_hdr_brightness_map_[1][id[4]], h_image_height_, h_image_width_, d_phase_map_[1]);

	}
	break;

	default:
		return false;
	}

}

void cuda_remove_points_base_radius_filter(float dot_spacing,float radius,int threshold_num)
{

	// cv::Mat pointcloud(1200, 1920, CV_32FC3, cv::Scalar(0));
	// CHECK(cudaMemcpy(pointcloud.data, d_point_cloud_map_, 3 * h_image_height_ * h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
	// std::vector<cv::Mat> channels;
	// cv::split(pointcloud, channels);
	// cv::imwrite("depth_f.tiff", channels[2]);

	cudaDeviceSynchronize();
	LOG(INFO)<<"kernel_depth_to_pointcloud:"; 
	kernel_depth_to_pointcloud << <blocksPerGrid, threadsPerBlock >> > (h_image_width_,h_image_height_,d_Q,d_depth_map_,d_pointcloud_map_);

	cudaDeviceSynchronize();

	LOG(INFO) << "remove_base_radius_filter start:";

	// //相机像素为5.4um、焦距12mm。dot_spacing = 5.4*distance/12000 mm，典型值0.54mm（1200）

	float d2 = dot_spacing * dot_spacing;
	float r2 = radius * radius;

	kernel_filter_radius_outlier_removal<<<blocksPerGrid, threadsPerBlock>>>(h_image_height_, h_image_width_, d_pointcloud_map_, d_mask_map_, d2, r2, threshold_num);
	cudaDeviceSynchronize();

	LOG(INFO) << "remove start:";
	kernel_removal_points_base_mask<<<blocksPerGrid, threadsPerBlock>>>(h_image_height_, h_image_width_, d_pointcloud_map_, d_depth_map_, d_mask_map_);

	cudaDeviceSynchronize();

	LOG(INFO)<<"remove_base_radius_filter finished!";
}

bool cuda_set_param_confidence(float val)
{
	d_confidence_ = val;
	
	cudaError_t error_code = cudaMemcpyToSymbol(d_confidence_, &val, sizeof(float));

	LOG(INFO) << "cuda_set_param_confidence: " << d_confidence_;

	if(error_code!= cudaSuccess)
	{
		return false;
	}

	return true;
}

void cuda_fix_four_step_code_shift(int serial_flag)
{
	dim3 threadsPerBlockTemp(32);
	dim3 blocksPerGridTemp(6, 8);

	kernel_fix_unwrap_phase<<<blocksPerGridTemp, threadsPerBlockTemp>>>(h_image_width_, h_image_height_, d_unwraped_pixels[0]);
	kernel_fix_unwrap_phase<<<blocksPerGridTemp, threadsPerBlockTemp>>>(h_image_width_, h_image_height_, d_unwraped_pixels[1]);

}

void cuda_fix_eight_step_code_shift(int serial_flag)
{
	dim3 threadsPerBlockTemp(32);
	dim3 blocksPerGridTemp(6, 8);

	kernel_fix_eight_step_unwrap_phase<<<blocksPerGridTemp, threadsPerBlockTemp>>>(h_image_width_, h_image_height_, d_unwraped_pixels[0]);
	kernel_fix_eight_step_unwrap_phase<<<blocksPerGridTemp, threadsPerBlockTemp>>>(h_image_width_, h_image_height_, d_unwraped_pixels[1]);

}

bool cuda_copy_result_to_hdr(int serial_flag,int brigntness_serial)
{
	CHECK(cudaMemcpyAsync(d_hdr_brightness_list_[serial_flag], d_patterns_list_[0][brigntness_serial], 1 * d_image_height_*d_image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToDevice));

	CHECK(cudaMemcpyAsync(d_hdr_depth_map_list_[serial_flag], d_depth_map_, 1 * d_image_height_*d_image_width_ * sizeof(float), cudaMemcpyDeviceToDevice)); 

	float val  = 0;
	CHECK(cudaMemcpyAsync(d_hdr_bright_pixel_sum_list_[serial_flag], &val, sizeof(float), cudaMemcpyHostToDevice)); 
 	cuda_count_sum_pixel << <blocksPerGrid, threadsPerBlock >> > (d_hdr_brightness_list_[serial_flag],d_image_height_,d_image_width_,d_hdr_bright_pixel_sum_list_[serial_flag]);
 
	LOG(INFO)<<"cuda_copy_result_to_hdr: "<<serial_flag;
	return true;
}

bool cuda_merge_hdr_data(int hdr_num,float* depth_map, unsigned char* brightness)
{
	
	LOG(INFO)<<"sum pixels ";
	float sum_pixels_list[6];  

    for(int i= 0;i<hdr_num;i++)
    { 
		CHECK(cudaMemcpy(&sum_pixels_list[i], d_hdr_bright_pixel_sum_list_[i], 1* sizeof(float), cudaMemcpyDeviceToHost));
    }
 
 
	std::vector<float> param_list;
	std::vector<int> id; 
	std::vector<bool> flag_list;

	for (int i = 0; i < hdr_num; i++)
	{ 
        param_list.push_back(sum_pixels_list[i]);
		id.push_back(0);
		flag_list.push_back(true);
    } 
   	std::sort(param_list.begin(),param_list.end(),std::greater<float>());
 
 
	for (int i = 0; i < hdr_num; i++)
	{ 
		
		for(int j= 0;j< hdr_num;j++)
		{
			if(param_list[i] == sum_pixels_list[j])
			{
				if(flag_list[j])
				{ 
					id[i] = j;
					flag_list[j] = false; 
					break;
				}
			}
		}
		 
    } 

 
	for (int i = 0; i < hdr_num; i++)
	{ 
        LOG(INFO)<<"sum pixels "<<i<<": "<<sum_pixels_list[i]<<" _ "<<id[i];
    }
 

	switch(hdr_num)
	{
		case 1:
		{

			CHECK(cudaMemcpy(depth_map, d_hdr_depth_map_list_[0], 1 * h_image_height_*h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_list_[0], 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
		} 
		break;
		case 2:
		{
			cuda_merge_hdr_depth_2 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_depth_map_list_[id[0]],d_hdr_depth_map_list_[id[1]], d_hdr_brightness_list_[id[0]], 
				d_hdr_brightness_list_[id[1]], h_image_height_, h_image_width_, d_depth_map_,d_patterns_list_[0][5]);

				
			CHECK(cudaMemcpy(depth_map, d_depth_map_, 1 * h_image_height_*h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));

		}
		break;
		case 3:
		{
			cuda_merge_hdr_depth_3 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_depth_map_list_[id[0]],d_hdr_depth_map_list_[id[1]],d_hdr_depth_map_list_[id[2]], d_hdr_brightness_list_[id[0]], 
				d_hdr_brightness_list_[id[1]], d_hdr_brightness_list_[id[2]], h_image_height_, h_image_width_, d_depth_map_,d_patterns_list_[0][5]);
				
			CHECK(cudaMemcpy(depth_map, d_depth_map_, 1 * h_image_height_*h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));

		}
		break;
		case 4:
		{
			cuda_merge_hdr_depth_4 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_depth_map_list_[id[0]],d_hdr_depth_map_list_[id[1]],d_hdr_depth_map_list_[id[2]],d_hdr_depth_map_list_[id[3]],
				 d_hdr_brightness_list_[id[0]], d_hdr_brightness_list_[id[1]], d_hdr_brightness_list_[id[2]], d_hdr_brightness_list_[id[3]], 
				h_image_height_, h_image_width_, d_depth_map_,d_patterns_list_[0][5]);
				
			CHECK(cudaMemcpy(depth_map, d_depth_map_, 1 * h_image_height_*h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));

		}
		break;
		case 5:
		{
			cuda_merge_hdr_depth_5 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_depth_map_list_[id[0]],d_hdr_depth_map_list_[id[1]],d_hdr_depth_map_list_[id[2]],
				d_hdr_depth_map_list_[id[3]],d_hdr_depth_map_list_[id[4]],
				 d_hdr_brightness_list_[id[0]], d_hdr_brightness_list_[id[1]], d_hdr_brightness_list_[id[2]], d_hdr_brightness_list_[id[3]], d_hdr_brightness_list_[id[4]], 
				h_image_height_, h_image_width_, d_depth_map_,d_patterns_list_[0][5]);
				
			CHECK(cudaMemcpy(depth_map, d_depth_map_, 1 * h_image_height_*h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));

		}
		break;
		case 6:
		{
			cuda_merge_hdr_depth_6 << <blocksPerGrid, threadsPerBlock >> > (d_hdr_depth_map_list_[id[0]],d_hdr_depth_map_list_[id[1]],d_hdr_depth_map_list_[id[2]],
				d_hdr_depth_map_list_[id[3]],d_hdr_depth_map_list_[id[4]],d_hdr_depth_map_list_[id[5]],
				 d_hdr_brightness_list_[id[0]], d_hdr_brightness_list_[id[1]], d_hdr_brightness_list_[id[2]], d_hdr_brightness_list_[id[3]], d_hdr_brightness_list_[id[4]], 
				 d_hdr_brightness_list_[id[5]], 
				h_image_height_, h_image_width_, d_depth_map_,d_patterns_list_[0][5]);
				
			CHECK(cudaMemcpy(depth_map, d_depth_map_, 1 * h_image_height_*h_image_width_ * sizeof(float), cudaMemcpyDeviceToHost));
			// CHECK(cudaMemcpy(brightness, d_hdr_brightness_, 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));

		}
		break;

		default:
		 		return false;

	}

 	// CHECK(cudaMemcpy(brightness, d_hdr_brightness_list_[id[0]], 1*image_height_*image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
 	CHECK(cudaMemcpy(brightness, d_hdr_brightness_list_[hdr_num-1], 1*h_image_height_*h_image_width_ * sizeof(unsigned char), cudaMemcpyDeviceToHost));
	LOG(INFO)<<"DHR Finished!";

	return true;
}

bool cuda_set_camera_gamma(float gamma)
{
	d_camera_gamma_ = gamma;
	cudaError_t error_code = cudaMemcpyToSymbol(d_camera_gamma_, &gamma, sizeof(float));
	if (error_code != cudaSuccess)
	{
		LOG(ERROR) << "cudaMemcpyToSymbol Failed! ";
		return false;
	}
	LOG(INFO) << "d_camera_gamma_: " << d_camera_gamma_;
	return true;
}

bool cuda_get_camera_gamma(float& gamma)
{
	gamma = d_camera_gamma_;
	cudaError_t error_code = cudaMemcpyFromSymbol(&gamma, d_camera_gamma_, sizeof(float));
	if (error_code != cudaSuccess)
	{
		LOG(ERROR) << "cudaMemcpyFromSymbol Failed! ";
		return false;
	}
	LOG(INFO) << "d_camera_gamma_: " << d_camera_gamma_;
	return true;
}

bool cuda_gnerate_depth2rgb_color_map()
{
	// 核函数运行之后得到depth2rgb_map
	// 函数输入：三通道点云
	// 重载depth的生成函数，增加这个map的输出
	// 就没有必要重新从depth得到点云

}
