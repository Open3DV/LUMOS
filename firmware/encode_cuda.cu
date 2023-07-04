#include "encode_cuda.cuh"



__global__ void kernel_decode_gray_code_8bit(int width, int height, unsigned char* decode_map, unsigned char* d_in_dark, unsigned char* d_in_bright, unsigned char* d_in_0, unsigned char* d_in_1, unsigned char* d_in_2, unsigned char* d_in_3, unsigned char* d_in_4, unsigned char* d_in_5, unsigned char* d_in_6, unsigned char* d_in_7, unsigned char* d_out, unsigned char* mask_niose)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;

	if (idx < width && idy < height)
	{
		if (d_in_bright[offset] - d_in_dark[offset] < 20)
		{
			d_out[offset] = 0;
			mask_niose[offset] = 255;
			return;
		}

		unsigned char threshold = (d_in_dark[offset] + d_in_bright[offset]) / 2;

		d_out[offset] = 0;
		d_out[offset] += (d_in_0[offset] > threshold) ? 1 : 0;

		d_out[offset] = (d_out[offset] << 1) + ((d_in_1[offset] > threshold) ? 1 : 0);
		d_out[offset] = (d_out[offset] << 1) + ((d_in_2[offset] > threshold) ? 1 : 0);
		d_out[offset] = (d_out[offset] << 1) + ((d_in_3[offset] > threshold) ? 1 : 0);
		d_out[offset] = (d_out[offset] << 1) + ((d_in_4[offset] > threshold) ? 1 : 0);
		d_out[offset] = (d_out[offset] << 1) + ((d_in_5[offset] > threshold) ? 1 : 0);
		d_out[offset] = (d_out[offset] << 1) + ((d_in_6[offset] > threshold) ? 1 : 0);
		d_out[offset] = (d_out[offset] << 1) + ((d_in_7[offset] > threshold) ? 1 : 0);
		d_out[offset] = decode_map[d_out[offset]];

	}
}

__global__ void kernel_decode_gray_code_one_by_one(int width, int height, unsigned char* d_in_threshold, unsigned char* d_in_img, unsigned char* d_out, unsigned char* mask_niose)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;

	if (idx < width && idy < height && !(mask_niose[offset]))
	{
		d_out[offset] = (d_out[offset] << 1) + (d_in_img[offset] > d_in_threshold[offset]);
	}
}

__global__ void kernel_decode_gray_code_one_by_one(int width, int height, unsigned short* d_in_threshold, unsigned short* d_in_img, unsigned char* d_out, unsigned char* mask_niose)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;

	if (idx < width && idy < height && !(mask_niose[offset]))
	{
		d_out[offset] = (d_out[offset] << 1) + (d_in_img[offset] > d_in_threshold[offset]);
	}
}

__global__ void kernel_4_step_phase_shift_8bit(int width, int height, unsigned char* d_in_0, unsigned char* d_in_1, unsigned char* d_in_2, unsigned char* d_in_3, unsigned char* d_out, unsigned char* mask_noise)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;

	if (idx < width && idy < height && !(mask_noise[offset]))
	{
		int over_num = 0;

		over_num = (d_in_0[offset] >= 250) + (d_in_1[offset] >= 250) + (d_in_2[offset] >= 250) + (d_in_3[offset] >= 250);

		float a = (float)d_in_2[offset] - (float)d_in_0[offset];
		float b = (float)d_in_3[offset] - (float)d_in_1[offset];


		d_out[offset] = (atan2(a, b) + CV_PI) * 40.7436654315252f * (sqrt(a * a + b * b) > 2) * (over_num < 2);
		mask_noise[offset] = 255 * (over_num > 1);

	}
}

__global__ void kernel_4_step_phase_shift_8bit(int width, int height, unsigned char* d_in_0, unsigned char* d_in_1, unsigned char* d_in_2, unsigned char* d_in_3, unsigned char* d_out, unsigned char* mask_noise, unsigned char* decode_threshold, float d_in_confidence)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;

	if (idx < width && idy < height && !(mask_noise[offset]))
	{
		int over_num = 0;

		over_num = (d_in_0[offset] >= 240) + (d_in_1[offset] >= 240) + (d_in_2[offset] >= 240) + (d_in_3[offset] >= 240);

		float a = (float)d_in_2[offset] - (float)d_in_0[offset];
		float b = (float)d_in_3[offset] - (float)d_in_1[offset];

		float r = sqrt(a * a + b * b);

		decode_threshold[offset] = ((int)d_in_0[offset] + (int)d_in_1[offset] + (int)d_in_2[offset] + (int)d_in_3[offset]) / 4;

		d_out[offset] = (atan2(a, b) + CV_PI) * 40.7436654315252f * (r >= d_in_confidence) * (over_num < 2);
		d_in_3[offset] = d_out[offset];
		mask_noise[offset] = 255 * ((over_num > 1) || (r < d_in_confidence));

	}
}

__global__ void kernel_4_step_phase_shift_16bit(int width, int height, unsigned short* d_in_0, unsigned short* d_in_1, unsigned short* d_in_2, unsigned short* d_in_3, unsigned char* d_out, unsigned char* mask_noise, unsigned short* decode_threshold, float d_in_confidence)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;

	if (idx < width && idy < height && !(mask_noise[offset]))
	{
		int over_num = 0;

		over_num = (d_in_0[offset] >= 4085) + (d_in_1[offset] >= 4085) + (d_in_2[offset] >= 4085) + (d_in_3[offset] >= 4085);

		float a = (float)d_in_2[offset] - (float)d_in_0[offset];
		float b = (float)d_in_3[offset] - (float)d_in_1[offset];

		float r = sqrt(a * a + b * b);

		decode_threshold[offset] = ((int)d_in_0[offset] + (int)d_in_1[offset] + (int)d_in_2[offset] + (int)d_in_3[offset]) / 4;

		d_out[offset] = (atan2(a, b) + CV_PI) * 40.7436654315252f * (r >= d_in_confidence) * (over_num < 2);
		d_in_3[offset] = d_out[offset];
		mask_noise[offset] = 255 * ((over_num > 1) || (r < d_in_confidence));

	}
}


__global__ void kernel_code_rectify_8bit(int width, int height, unsigned char* d_in_code, unsigned char* d_in_phase, unsigned char* mask_noise)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;

	if (idx < width && idy < height)
	{
		d_in_code[offset] += !(d_in_code[offset] % 2) * (d_in_phase[offset] < 96) * (d_in_code[offset] < 255) + (d_in_code[offset] % 2) * (d_in_phase[offset] > 160) * (d_in_code[offset] > 0) * (-1);
		if (mask_noise[offset])
		{
			d_in_code[offset] = 0;
		}
	}
}

__global__ void kernel_code_phase_unwrap_8bit(int width, int height, unsigned char* d_in_code, unsigned char* d_in_phase, unsigned short* d_out_unwrap, unsigned char* mask_noise)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;

	if (idx < width && idy < height && !mask_noise[offset])
	{
		d_out_unwrap[offset] = ((int)d_in_code[offset] + 1) / 2 * 256 + d_in_phase[offset];
	}
}

__global__ void kernel_decode_threshold_and_mask(int width, int height, float d_in_confidence, unsigned char* d_in_darkness, unsigned char* d_in_brightness, unsigned char* d_threshold, unsigned char* d_mask_noise, unsigned char* d_code)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;

	// if (idx < width && idy < height)
	// {
	// 	d_code[offset] = 0;
	// 	d_threshold[offset] = (d_in_darkness[offset] + d_in_brightness[offset]) / 2;
	// 	if (d_threshold[offset] < d_in_confidence)
	// 	{
	// 		d_mask_noise[offset] = 255;
	// 		d_threshold[offset] = 255;
	// 	}

	// }
	if (idx < width && idy < height)
	{
		d_code[offset] = 0;
		//int isGood = d_threshold[offset] > d_in_confidence;
		//int isGood = 1;
		int temp = (d_threshold[offset] * 2);
		d_in_brightness[offset] = temp > 255 ? 255 : temp;
		//d_threshold[offset] = ((d_in_darkness[offset] + d_in_brightness[offset]) / 2) * isGood;
		//d_mask_noise[offset] = 255 * !isGood;

	}
}

__global__ void kernel_decode_threshold_and_mask(int width, int height, float d_in_confidence, unsigned char* d_in_darkness, unsigned char* d_in_brightness, unsigned short* d_threshold, unsigned char* d_mask_noise, unsigned char* d_code)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;

	// if (idx < width && idy < height)
	// {
	// 	d_code[offset] = 0;
	// 	d_threshold[offset] = (d_in_darkness[offset] + d_in_brightness[offset]) / 2;
	// 	if (d_threshold[offset] < d_in_confidence)
	// 	{
	// 		d_mask_noise[offset] = 255;
	// 		d_threshold[offset] = 255;
	// 	}

	// }
	if (idx < width && idy < height)
	{
		d_code[offset] = 0;
		//int isGood = d_threshold[offset] > d_in_confidence;
		//int isGood = 1;
		int temp = d_threshold[offset] >> 3;
		d_in_brightness[offset] = temp > 255 ? 255 : temp;
		//d_threshold[offset] = ((d_in_darkness[offset] + d_in_brightness[offset]) / 2) * isGood;
		//d_mask_noise[offset] = 255 * !isGood;

	}
}

__global__ void kernel_gray_code_to_bin_code(int width, int height, unsigned char* d_in_out_code, unsigned char* d_gray_code_to_bin_map, unsigned char* d_in_noise_mask, unsigned char* d_in_test)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;

	if (idx < width && idy < height && !(d_in_noise_mask[offset]))
	{
		d_in_out_code[offset] = d_gray_code_to_bin_map[d_in_out_code[offset]];
		d_in_test[offset] = d_in_out_code[offset];
	}
}

__global__ void kernel_code_statistics(int width, int height, unsigned char* d_in_code, unsigned short* d_num_of_pixels_one_code)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset_y = idy * gridDim.x * blockDim.x + idx;

	if (offset_y < height)
	{
		unsigned int offset = offset_y * width;
		for (int i = 0; i < width; i += 1)
		{
			d_num_of_pixels_one_code[offset_y * 256 + d_in_code[offset]] += 1;
			offset += 1;
		}
	}
}

__global__ void kernel_code_statistics_to_index(int width, int height, unsigned short* d_num_of_pixels_one_code, unsigned short* d_inedx_of_pixels)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset_y = idy * gridDim.x * blockDim.x + idx;

	if (offset_y < height)
	{
		int offset = offset_y * 256;
		d_inedx_of_pixels[offset] = 0;
		for (int i = 1; i < 256; i += 1)
		{
			offset = offset_y * 256 + i;
			d_inedx_of_pixels[offset] = d_inedx_of_pixels[offset - 1] + d_num_of_pixels_one_code[offset - 1];
		}
	}
}

__global__ void kernel_sort_code(int width, int height, unsigned char* d_in_code, unsigned short* d_sorted_pixels, unsigned short* d_num_of_pixels_one_code, unsigned char* d_in_noise_mask)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset_y = idy * gridDim.x * blockDim.x + idx;
	const unsigned int offset_img = offset_y * width;
	const unsigned int offset_256 = offset_y * 256;

	if (offset_y < height)
	{
		for (int i = 0; i < width; i += 1)
		{
			// 1.offsety+i就是当前的像素
			// 2.把当前的像素根据查询出来的code，插入到目标map，index+=1
			// if (d_in_noise_mask[offset_img + i])
			// {
			// 	d_in_code[offset_img + i] = 0;
			// 	continue;
			// }

			unsigned int code_now = d_in_code[offset_img + i];
			d_sorted_pixels[offset_img + d_num_of_pixels_one_code[offset_256 + code_now]] = i;
			d_num_of_pixels_one_code[offset_256 + code_now] += 1;
		}
	}
}

__global__ void kernel_filter_code_noise(int width, int height, unsigned short* d_in_unwrap_phase, unsigned short* d_in_num_of_pixels, unsigned short* d_in_index_of_pixels, unsigned short* d_in_sorted_pixels)
{
	// 核函数的并行是基于图像高 * 256来及进行循环
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * 256 + idx;
	const unsigned int offset_img = idy * width;

	if (idy < height && idx < 256 && d_in_num_of_pixels[offset] > 1 && d_in_index_of_pixels[offset] != 0)
	{
		short max_monotonicity_segmentation_num = 1;
		short max_monotonicity_segmentation_num_now = 1;
		short max_monotonicity_segmentation_index = -1;
		short max_monotonicity_segmentation_index_now = d_in_index_of_pixels[offset];
		bool include_left = false;
		bool include_right = false;
		// 先判断左点是否满足
		if (d_in_num_of_pixels[offset] < 2)
		{
			return;
		}
		//if (d_in_index_of_pixels[offset] > 0)
		//{
		//	if (d_in_unwrap_phase[offset_img + d_in_index_of_pixels[offset]] - d_in_unwrap_phase[offset_img + d_in_index_of_pixels[offset] - 1] < 255 && d_in_unwrap_phase[offset_img + d_in_index_of_pixels[offset]] > d_in_unwrap_phase[offset_img + d_in_index_of_pixels[offset] - 1])//右减左小于255，且右大于左
		//	{
		//		include_left == true;
		//	}
		//}
		//if (d_in_index_of_pixels[offset] + d_in_num_of_pixels[offset] - 1 < width && )
		//{
		//	if (d_in_unwrap_phase[offset_img + d_in_index_of_pixels[offset]] - d_in_unwrap_phase[offset_img + d_in_index_of_pixels[offset] - 1] < 255 && d_in_unwrap_phase[offset_img + d_in_index_of_pixels[offset]] > d_in_unwrap_phase[offset_img + d_in_index_of_pixels[offset] - 1])//右减左小于255，且右大于左
		//	{
		//		include_left == true;
		//	}
		//}

		//int img_x = d_in_sorted_pixels[offset_img + d_in_index_of_pixels[offset]];
		bool sheared = false;

		for (int i = 0; i < d_in_num_of_pixels[offset] - 1; i += 1)//两两比较
		{
			int index_img = offset_img + d_in_sorted_pixels[offset_img + d_in_index_of_pixels[offset] + i];
			int index_img_add_1 = offset_img + d_in_sorted_pixels[offset_img + d_in_index_of_pixels[offset] + i + 1];
			if (d_in_unwrap_phase[index_img_add_1] > d_in_unwrap_phase[index_img] - 64 && index_img_add_1 - index_img < 3)
			{
				max_monotonicity_segmentation_num_now += 1;
			}
			else
			{
				max_monotonicity_segmentation_index_now = d_in_index_of_pixels[offset] + i + 1;
				max_monotonicity_segmentation_num_now = 1;
			}

			if (max_monotonicity_segmentation_num_now > max_monotonicity_segmentation_num)
			{
				max_monotonicity_segmentation_index = max_monotonicity_segmentation_index_now;
				max_monotonicity_segmentation_num = max_monotonicity_segmentation_num_now;
				sheared = true;
			}
		}
		if (sheared)
		{
			d_in_num_of_pixels[offset] = max_monotonicity_segmentation_num;
			d_in_index_of_pixels[offset] = max_monotonicity_segmentation_index;
		}

	}
}

__global__ void kernel_change_edge(int width, int height, unsigned short* d_in_unwrap_phase, unsigned short* d_in_sorted_pixels, unsigned short* d_in_num_of_pixels, unsigned short* d_in_index_of_pixels)
{
	// 核函数的并行是基于图像高 * 256来及进行循环
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * 256 + idx;
	const unsigned int offset_img = idy * width;

	if (idy < height && idx < 256)
	{
		bool include_left = false;
		bool include_right = false;

		int index_left_1 = d_in_sorted_pixels[d_in_index_of_pixels[offset]] - 1;

		if (index_left_1 > -1)//判断左侧的index大于0，要从sorted里去找
		{
			if (d_in_unwrap_phase[index_left_1 + 1] - d_in_unwrap_phase[index_left_1] > 0 && d_in_unwrap_phase[index_left_1 + 1] - d_in_unwrap_phase[index_left_1] < 255)
			{
				include_left = true;
			}
		}
		
		int index_right_1 = d_in_sorted_pixels[d_in_index_of_pixels[offset] + d_in_num_of_pixels[offset] - 1] + 1;

		if (index_right_1 < width)//判断左侧的index大于0，要从sorted里去找
		{
			if (d_in_unwrap_phase[index_right_1] - d_in_unwrap_phase[index_right_1 - 1] > 0 && d_in_unwrap_phase[index_right_1] - d_in_unwrap_phase[index_right_1 - 1] < 255)
			{
				include_right = true;
			}
		}

		if (include_left)
		{
			d_in_index_of_pixels[offset] -= 1;
			d_in_num_of_pixels[offset] += 1;
		}
		if (include_right)
		{
			d_in_num_of_pixels[offset] += 1;
		}

	}
}

__global__ void kernel_matching_(int width, int height, unsigned short* d_in_sorted_pixels_left, unsigned short* d_in_unwrap_phase_left, unsigned short* d_in_num_of_pixels_left, unsigned short* d_in_index_of_pixels_left, unsigned short* d_in_sorted_pixels_right, unsigned short* d_in_unwrap_phase_right, unsigned short* d_in_num_of_pixels_right, unsigned short* d_in_index_of_pixels_right, float* disparty, unsigned char* disparty_mask)
{
	// 核函数的并行是基于图像高 * 256来及进行循环
	// 先一个
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * 256 + idx;
	const unsigned int offset_img = idy * width;

	if (idy < height && idx < 256 && d_in_num_of_pixels_left[offset] > 0 && d_in_num_of_pixels_right[offset] > 0 && idx > 0)
	{
		bool include_left = true;
		bool include_right = true;
		float left_phase;
		float right_phase_1;
		float right_phase;


		int left_x = d_in_sorted_pixels_left[offset_img + d_in_index_of_pixels_left[offset]];
		int right_x_1 = d_in_sorted_pixels_right[offset_img + d_in_index_of_pixels_right[offset] - 1];
		int right_x = d_in_sorted_pixels_right[offset_img + d_in_index_of_pixels_right[offset]];

		int j = 1;
		for (int i = 0; i < d_in_num_of_pixels_left[offset]; i += 1)
		{
			left_x = d_in_sorted_pixels_left[offset_img + d_in_index_of_pixels_left[offset] + i];
			left_phase = d_in_unwrap_phase_left[offset_img + left_x];

			for (j = 1; j < d_in_num_of_pixels_right[offset]; j += 1)
			{
				right_x_1 = d_in_sorted_pixels_right[offset_img + d_in_index_of_pixels_right[offset] + j - 1];
				right_x = d_in_sorted_pixels_right[offset_img + d_in_index_of_pixels_right[offset] + j];
				right_phase_1 = d_in_unwrap_phase_right[offset_img + right_x_1];
				right_phase = d_in_unwrap_phase_right[offset_img + right_x];

				if (left_phase <= right_phase && left_phase >= right_phase_1 || left_phase == right_phase || left_phase == right_phase_1)
				{
					disparty[offset_img + left_x] = left_x - (right_x_1 + (left_phase - right_phase_1) / (right_phase - right_phase_1) * (right_x - right_x_1));
					disparty_mask[offset_img + left_x] = 255;
					//disparty[offset_img + left_x] = 255;
					continue;
				}
			}
		}

		// 新增判断，识别缝隙

		left_x = d_in_sorted_pixels_left[offset_img + d_in_index_of_pixels_left[offset]];
		right_x = d_in_sorted_pixels_right[offset_img + d_in_index_of_pixels_right[offset]];




		if (right_x > 0 && include_left)
		{
			right_phase_1 = d_in_unwrap_phase_right[offset_img + right_x - 1];
			right_phase = d_in_unwrap_phase_right[offset_img + right_x];
			left_phase = d_in_unwrap_phase_left[offset_img + left_x];
			if (left_phase > right_phase_1 && left_phase < right_phase && right_phase - right_phase_1 < 256)
			{
				disparty[offset_img + left_x] = left_x - (right_x - 1 + (left_phase - right_phase_1) / (right_phase - right_phase_1));
				disparty_mask[offset_img + left_x] = 255;
			}
		}

		left_x = d_in_sorted_pixels_left[offset_img + d_in_index_of_pixels_left[offset] + d_in_num_of_pixels_left[offset] - 1];
		right_x = d_in_sorted_pixels_right[offset_img + d_in_index_of_pixels_right[offset] + d_in_num_of_pixels_right[offset] - 1];

		if (right_x + 1 < width && include_right)
		{
			right_phase_1 = d_in_unwrap_phase_right[offset_img + right_x + 1];//右1
			right_phase = d_in_unwrap_phase_right[offset_img + right_x];
			left_phase = d_in_unwrap_phase_left[offset_img + left_x];
			if (left_phase < right_phase_1 && left_phase > right_phase && right_phase_1 - right_phase < 255)
			{

				disparty[offset_img + left_x] = left_x - (right_x + (left_phase - right_phase) / (right_phase_1 - right_phase));
				disparty_mask[offset_img + left_x] = 255;

			}
		}

	}
}

__global__ void kernel_matching(int width, int height, unsigned short* d_in_sorted_pixels_left, unsigned short* d_in_unwrap_phase_left, unsigned short* d_in_num_of_pixels_left, unsigned short* d_in_index_of_pixels_left, unsigned short* d_in_sorted_pixels_right, unsigned short* d_in_unwrap_phase_right, unsigned short* d_in_num_of_pixels_right, unsigned short* d_in_index_of_pixels_right, float* disparty, unsigned char* disparty_mask)
{
	// 核函数的并行是基于图像高 * 256来及进行循环
	// 先一个
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * 256 + idx;
	const unsigned int offset_img = idy * width;

	// 比起寻找目标点的右侧，不如寻找相邻的格雷码有无

	if (idy < height && idx < 256 && d_in_num_of_pixels_left[offset] > 0 && d_in_num_of_pixels_right[offset] > 0 && idx > 0)
	{
		bool include_left = false;
		bool include_right = false;
		float left_phase;
		float right_phase_1;
		float right_phase;

		// 主要是需要处理右侧的点，右侧的点朝左、朝右要扩展；
		int num_left = d_in_num_of_pixels_left[offset];
		if (num_left > 30)
		{
			num_left = 30;
		}
		int num_right = d_in_num_of_pixels_right[offset];

		int more_num = 1;

		if (num_right > 30 - more_num * 2)
		{
			num_right = 30 - more_num * 2;
		}
		// 拷贝数据到本地的数组：相位、x坐标
		unsigned short left_phase_list[30];
		unsigned short right_phase_list[30];
		unsigned short left_x_list[30];
		unsigned short right_x_list[30];

		

		for (int i = 0; i < num_left; i += 1)
		{
			left_x_list[i] = d_in_sorted_pixels_left[offset_img + d_in_index_of_pixels_left[offset] + i];
			left_phase_list[i] = d_in_unwrap_phase_left[offset_img + left_x_list[i]];
		}
		
		// for (int i = 0; i < more_num; i += 1)
		// {
			
		// } 
		right_x_list[0] = d_in_sorted_pixels_right[offset_img + d_in_index_of_pixels_right[offset]] - 1;

		if (right_x_list[0] < 1)
		{
			right_x_list[0] = 1;
		}

		right_phase_list[0] = d_in_unwrap_phase_right[offset_img + right_x_list[0]];

		for (int i = 0; i < num_right; i += 1)
		{
			right_x_list[i + 1] = d_in_sorted_pixels_right[offset_img + d_in_index_of_pixels_right[offset] + i];
			right_phase_list[i + 1] = d_in_unwrap_phase_right[offset_img + right_x_list[i + 1]];
		}

		right_x_list[num_right + 1] = right_x_list[num_right] + 1;

		if (right_x_list[num_right + 1] > width - 2)
		{
			right_x_list[num_right + 1] = width - 1;
		}

		right_phase_list[num_right + 1] = d_in_unwrap_phase_right[offset_img + right_x_list[num_right + 1]];

		int left_x;
		int right_x_1;
		int right_x;

		int j = 1;
		for (int i = 0; i < num_left; i += 1)
		{
			left_x = left_x_list[i];
			left_phase = left_phase_list[i];

			for (j = 1; j < num_right + 2; j += 1)
			{
				right_x_1 = right_x_list[j - 1];
				right_x = right_x_list[j];
				right_phase_1 = right_phase_list[j - 1];
				right_phase = right_phase_list[j];

				if (left_phase <= right_phase && left_phase >= right_phase_1 || left_phase == right_phase || left_phase == right_phase_1)
				{
					disparty[offset_img + left_x] = left_x - (right_x_1 + (left_phase - right_phase_1) / (right_phase - right_phase_1) * (right_x - right_x_1));
					disparty_mask[offset_img + left_x] = 255;
					break;
				}
			}
		}

		// 新增判断，识别缝隙

		// left_x = d_in_sorted_pixels_left[offset_img + d_in_index_of_pixels_left[offset]];
		// right_x = d_in_sorted_pixels_right[offset_img + d_in_index_of_pixels_right[offset]];




		// if (right_x > 0)
		// {
		// 	right_phase_1 = d_in_unwrap_phase_right[offset_img + right_x - 1];
		// 	right_phase = d_in_unwrap_phase_right[offset_img + right_x];
		// 	left_phase = d_in_unwrap_phase_left[offset_img + left_x];
		// 	if (left_phase > right_phase_1 && left_phase < right_phase && right_phase - right_phase_1 < 256)
		// 	{
		// 		disparty[offset_img + left_x] = left_x - (right_x - 1 + (left_phase - right_phase_1) / (right_phase - right_phase_1));
		// 		disparty_mask[offset_img + left_x] = 255;
		// 	}
		// }

		// left_x = d_in_sorted_pixels_left[offset_img + d_in_index_of_pixels_left[offset] + num_left - 1];
		// right_x = d_in_sorted_pixels_right[offset_img + d_in_index_of_pixels_right[offset] + num_right - 1];

		// if (right_x + 1 < width)
		// {
		// 	right_phase_1 = d_in_unwrap_phase_right[offset_img + right_x + 1];//右1
		// 	right_phase = d_in_unwrap_phase_right[offset_img + right_x];
		// 	left_phase = d_in_unwrap_phase_left[offset_img + left_x];
		// 	if (left_phase < right_phase_1 && left_phase > right_phase && right_phase_1 - right_phase < 255)
		// 	{

		// 		disparty[offset_img + left_x] = left_x - (right_x + (left_phase - right_phase) / (right_phase_1 - right_phase));
		// 		disparty_mask[offset_img + left_x] = 255;

		// 	}
		// }

	}
}

__global__ void kernel_dispaty_to_depth(int width, int height, float* d_in_Q, float* d_in_disparty, float* d_out_depth_map, unsigned char* disparty_mask)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;

	if (idx < width && idy < height && disparty_mask[offset] == 255)
	{
		float f = d_in_Q[11];
		float w = d_in_Q[14] * d_in_disparty[offset] + d_in_Q[15];

		if (w > 0)
		{
			d_out_depth_map[offset] = f / w;
		}
	}
}

__global__ void kernel_depth_to_pointcloud(int width, int height, float* d_in_Q, float* d_in_depth_map, float* d_out_pointcloud_map)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * width + idx;

	if (idx < width && idy < height)
	{
		float f = d_in_Q[11];
		float u0 = -d_in_Q[3];
		float v0 = -d_in_Q[7];
		float z = d_in_depth_map[offset];

		if (z > 0)
		{
			d_out_pointcloud_map[offset * 3] = z * (idx - u0) / f;
			d_out_pointcloud_map[offset * 3 + 1] = z * (idy - v0) / f;
			d_out_pointcloud_map[offset * 3 + 2] = z;
		}
	}
}

__global__ void kernel_remap(uchar* src, uchar* dst, short2* map1, ushort* map2, short4* weight, int width, int height)
{
	int idx_x = blockDim.x * blockIdx.x + threadIdx.x;
	int idx_y = blockDim.y * blockIdx.y + threadIdx.y;
	if (idx_x >= width || idx_y >= height) return;
	int idx = idx_y * width + idx_x;
	short2 xy = map1[idx];
	ushort a = map2[idx] & 1023;
	short4 w = weight[a];
	int x = xy.x, y = xy.y;
	int x0 = x >= 0 && x < width ? x : -1;
	int x1 = (x + 1) >= 0 && (x + 1) < width ? x + 1 : -1;
	int y0 = y >= 0 && y < height ? y : -1;
	int y1 = (y + 1) >= 0 && (y + 1) < height ? y + 1 : -1;
	uchar v0 = x0 >= 0 && y0 >= 0 ? src[y0 * width + x0] : 0;
	uchar v1 = x1 >= 0 && y0 >= 0 ? src[y0 * width + x1] : 0;
	uchar v2 = x0 >= 0 && y1 >= 0 ? src[y1 * width + x0] : 0;
	uchar v3 = x1 >= 0 && y1 >= 0 ? src[y1 * width + x1] : 0;
	dst[idx] = (int(w.x * v0 + w.y * v1 + w.z * v2 + w.w * v3) + (1 << 14)) >> 15;
}

__global__ void kernel_remap(unsigned short* src, unsigned short* dst, short2* map1, ushort* map2, short4* weight, int width, int height)
{
	int idx_x = blockDim.x * blockIdx.x + threadIdx.x;
	int idx_y = blockDim.y * blockIdx.y + threadIdx.y;
	if (idx_x >= width || idx_y >= height) return;
	int idx = idx_y * width + idx_x;
	short2 xy = map1[idx];
	ushort a = map2[idx] & 1023;
	short4 w = weight[a];
	int x = xy.x, y = xy.y;
	int x0 = x >= 0 && x < width ? x : -1;
	int x1 = (x + 1) >= 0 && (x + 1) < width ? x + 1 : -1;
	int y0 = y >= 0 && y < height ? y : -1;
	int y1 = (y + 1) >= 0 && (y + 1) < height ? y + 1 : -1;
	unsigned short v0 = x0 >= 0 && y0 >= 0 ? src[y0 * width + x0] : 0;
	unsigned short v1 = x1 >= 0 && y0 >= 0 ? src[y0 * width + x1] : 0;
	unsigned short v2 = x0 >= 0 && y1 >= 0 ? src[y1 * width + x0] : 0;
	unsigned short v3 = x1 >= 0 && y1 >= 0 ? src[y1 * width + x1] : 0;
	dst[idx] = (int(w.x * v0 + w.y * v1 + w.z * v2 + w.w * v3) + (1 << 14)) >> 15;
}

__global__ void kernel_depth_filter_step_1(uint32_t img_height, uint32_t img_width, float depth_threshold, float* const depth_map, float* const depth_map_temp, unsigned char* mask_temp)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;

	// int offset_y = idy * 64 + idx; 
	int offset_y = idy * blockDim.x * gridDim.x + idx;

	int nr = img_height;
	int nc = img_width;

	if (offset_y < img_height - 1 && offset_y > 1)
	{
		//读数据

		float* depthPtr = depth_map + (offset_y * img_width);
		float* beforeDepthPtr = depth_map + ((offset_y - 1) * img_width);
		float* nextDepthPtr = depth_map + ((offset_y + 1) * img_width);

		float* featureTemp = depth_map_temp + (offset_y * img_width);
		unsigned char* maskPtr = mask_temp + (offset_y * img_width);

		float depth_diff[9];
		for (int col = 1; col < img_width; col += 1)
		{
			maskPtr[col] = 255;
			if (depthPtr[col] <= 0)
			{
				featureTemp[col] = -1;
				continue;
			}

			// 总共是0-7八个点的计算
			depth_diff[0] = beforeDepthPtr[col - 1] > 0 ? abs(beforeDepthPtr[col - 1] - depthPtr[col]) * 2. / (beforeDepthPtr[col - 1] + depthPtr[col]) : -1;
			depth_diff[1] = beforeDepthPtr[col] > 0 ? abs(beforeDepthPtr[col] - depthPtr[col]) * 2. / (beforeDepthPtr[col] + depthPtr[col]) : -1;
			depth_diff[2] = beforeDepthPtr[col + 1] > 0 ? abs(beforeDepthPtr[col + 1] - depthPtr[col]) * 2. / (beforeDepthPtr[col + 1] + depthPtr[col]) : -1;
			depth_diff[3] = depthPtr[col - 1] > 0 ? abs(depthPtr[col - 1] - depthPtr[col]) * 2. / (depthPtr[col - 1] + depthPtr[col]) : -1;
			depth_diff[4] = depthPtr[col + 1] > 0 ? abs(depthPtr[col + 1] - depthPtr[col]) * 2. / (depthPtr[col + 1] + depthPtr[col]) : -1;
			depth_diff[5] = nextDepthPtr[col - 1] > 0 ? abs(nextDepthPtr[col - 1] - depthPtr[col]) * 2. / (nextDepthPtr[col - 1] + depthPtr[col]) : -1;
			depth_diff[6] = nextDepthPtr[col] > 0 ? abs(nextDepthPtr[col] - depthPtr[col]) * 2. / (nextDepthPtr[col] + depthPtr[col]) : -1;
			depth_diff[7] = nextDepthPtr[col + 1] > 0 ? abs(nextDepthPtr[col + 1] - depthPtr[col]) * 2. / (nextDepthPtr[col + 1] + depthPtr[col]) : -1;

			// 这个点的值等于depth的最大值
			float maxDepthDiff = -1;
			for (int i = 0; i < 8; i += 1)
			{
				if (depth_diff[i] > maxDepthDiff)
				{
					maxDepthDiff = depth_diff[i];
				}
			}
			// 孤立点直接过滤
			if (maxDepthDiff == -1)
			{
				depthPtr[col] = 0;
				continue;
			}

			featureTemp[col] = abs(maxDepthDiff);

			if (featureTemp[col] > depth_threshold)
			{
				maskPtr[col] = 0;
			}
		}
	}
}

__global__ void kernel_depth_filter_step_2(uint32_t img_height, uint32_t img_width, float depth_threshold, float* const depth_map, float* const depth_map_temp, unsigned char* mask_temp)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;

	// int offset_y = idy * 64 + idx; 
	int offset_y = idy * blockDim.x * gridDim.x + idx;

	int nr = img_height;
	int nc = img_width;

	if (offset_y < img_height - 2 && offset_y > 2)
	{
		//读数据

		unsigned char* maskPtr = mask_temp + (offset_y * img_width);

		float* depthPtr = depth_map + (offset_y * img_width);

		float* featureTempBeforePtr = depth_map_temp + ((offset_y - 1) * img_width);
		float* featureTempPtr = depth_map_temp + (offset_y * img_width);
		float* featureTempNextPtr = depth_map_temp + ((offset_y + 1) * img_width);

		float depthFeatureResult;
		float depthDiff[8];

		for (int col = 0; col < img_width; col += 1)
		{
			if (maskPtr[col] == 255)
			{
				maskPtr[col] = 0;
				continue;
			}
			// 比较相邻9个点的值，然后获取
			depthDiff[0] = featureTempBeforePtr[col - 1];
			depthDiff[1] = featureTempBeforePtr[col];
			depthDiff[2] = featureTempBeforePtr[col + 1];
			depthDiff[3] = featureTempPtr[col - 1];

			depthDiff[4] = featureTempPtr[col + 1];
			depthDiff[5] = featureTempNextPtr[col - 1];
			depthDiff[6] = featureTempNextPtr[col];
			depthDiff[7] = featureTempNextPtr[col + 1];

			float compareTemp;
			for (int i = 0; i < DEPTH_DIFF_NUM_THRESHOLD; i += 1)
			{
				for (int j = i + 1; j < 8; j += 1)
				{
					if (depthDiff[j] == -1)
					{
						continue;
					}
					if (depthDiff[i] > depthDiff[j])
					{
						compareTemp = depthDiff[i];
						depthDiff[i] = depthDiff[j];
						depthDiff[j] = compareTemp;
					}
				}
			}

			depthFeatureResult = depthDiff[DEPTH_DIFF_NUM_THRESHOLD - 1];

			if (depthFeatureResult > depth_threshold || depthFeatureResult == -1)
			{
				depthPtr[col] = 0;
			}

		}
	}
}

__global__ void kernel_filter_radius_outlier_removal(uint32_t img_height, uint32_t img_width,float* const point_cloud_map,unsigned char* remove_mask, float dot_spacing_2, float r_2,int threshold)
{
 	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y; 
  
	const unsigned int serial_id = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{
		/****************************************************************************/
		//定位区域
		if (point_cloud_map[3 * serial_id + 2] > 0)
		{
			remove_mask[serial_id] = 255;
			int w = 5;

			int s_r = idy - w;
			int s_c = idx - w;

			int e_r = idy + w;
			int e_c = idx + w;

			if (s_r < 0)
			{
				s_r = 0;
			}
			if (s_c < 0)
			{
				s_c = 0;
			}

			if (e_r >= img_height)
			{
				e_r = img_height - 1;
			}

			if (e_c >= img_width)
			{
				e_c = img_width - 1;
			}

			int num = 0;

			for (int r = s_r; r <= e_r; r++)
			{
				for (int c = s_c; c <= e_c; c++)
				{
					float space2 = ((idx - c) * (idx - c) + (idy - r) * (idy - r)) * dot_spacing_2;
					if (space2 > r_2)
						continue;

					int pos = r * img_width + c;
					if (point_cloud_map[3 * pos + 2] > 0)
					{  
						float dx= point_cloud_map[3 * serial_id + 0] - point_cloud_map[3 * pos + 0];
						float dy= point_cloud_map[3 * serial_id + 1] - point_cloud_map[3 * pos + 1];
						float dz= point_cloud_map[3 * serial_id + 2] - point_cloud_map[3 * pos + 2];

						float d2 = dx * dx + dy * dx + dz * dz;
						// float dist = std::sqrt(dx * dx + dy * dx + dz * dz); 
 
						// if (radius > dist)
						if (r_2 > d2)
						{
							num++;
						}
					}
				}
			} 

			if (num < threshold)
			{ 
				remove_mask[serial_id] = 0;
			} 
		}
		else
		{ 
			remove_mask[serial_id] = 0;
		}

		/******************************************************************/
	}
}

__global__ void kernel_removal_points_base_mask(uint32_t img_height, uint32_t img_width,float* const point_cloud_map,float* const depth_map,uchar* remove_mask)
{
  	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y; 
  
	const unsigned int serial_id = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{
		if(0 == remove_mask[serial_id])
		{
			depth_map[serial_id] = 0;
			point_cloud_map[3 * serial_id + 0] = 0;
			point_cloud_map[3 * serial_id + 1] = 0;
			point_cloud_map[3 * serial_id + 2] = 0;
		}

	}

}

#define STEP_FIX_PHASE 64

__global__ void kernel_fix_unwrap_phase(int width, int height, unsigned short* d_in_unwrap_phase)
{
	// 每次读取的数据是一行的数据，后续需要优化
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y; 

	int offset_y = idy * blockDim.x * gridDim.x + idx;
	int offset = offset_y * width;

	float pixel_before = 1.;
	float pixel_now = 1.;
	float phase_before = 0.;
	float phase_now = 0.;

	if (offset_y < height)
	{
		for (int col = 1; col < width - 1; col += 1)
		{
			if (d_in_unwrap_phase[offset + col] < d_in_unwrap_phase[offset + col + 1] && (d_in_unwrap_phase[offset + col + 1] / STEP_FIX_PHASE - d_in_unwrap_phase[offset + col] / STEP_FIX_PHASE) == 1)
			{
				pixel_before = pixel_now;
				phase_before = phase_now;
				phase_now = d_in_unwrap_phase[offset + col + 1] / STEP_FIX_PHASE * STEP_FIX_PHASE;

        		pixel_now = col + ((float)(phase_now - d_in_unwrap_phase[offset + col]) / (float)(d_in_unwrap_phase[offset + col + 1] - d_in_unwrap_phase[offset + col]));

				if (phase_now - phase_before > STEP_FIX_PHASE)
				{
					continue;
				}

				for (int i = ceil(pixel_before); i < pixel_now; i += 1)
				{
					if (d_in_unwrap_phase[offset + i] < phase_now && d_in_unwrap_phase[offset + i] > phase_before)
					{
						d_in_unwrap_phase[offset + i] = phase_before + STEP_FIX_PHASE * ((i - pixel_before)/(pixel_now - pixel_before));
					}
				}

			}
		}

	}


}
