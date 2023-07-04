#include "merge_hdr.cuh"



__global__ void cuda_merge_hdr_depth_6(const float*  depth_map_0,const float*  depth_map_1,const float*  depth_map_2,
	const float*  depth_map_3,const float*  depth_map_4,const float*  depth_map_5,
	const unsigned char* brightness_0,const unsigned char* brightness_1,const unsigned char* brightness_2,
	const unsigned char* brightness_3,const unsigned char* brightness_4,const unsigned char* brightness_5,
	uint32_t img_height, uint32_t img_width, float* const depth_map,unsigned char * const brightness)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{
 

		float pixel= 0;
		pixel +=  brightness_0[offset];
		pixel +=  brightness_1[offset];
		pixel +=  brightness_2[offset];
		pixel +=  brightness_3[offset];
		pixel +=  brightness_4[offset];
		pixel +=  brightness_5[offset];

		pixel/= 6.0;


		brightness[offset] = pixel;

		if(brightness_0[offset] < 255 && depth_map_0[offset] > 0)
		{
			// brightness[offset] = brightness_0[offset];
			depth_map[offset] = depth_map_0[offset];
		}

		else if(brightness_1[offset] < 255 && depth_map_1[offset] > 0)
		{
			// brightness[offset] = brightness_1[offset];
			depth_map[offset] = depth_map_1[offset];
		}
		else if(brightness_2[offset] < 255 && depth_map_2[offset] > 0)
		{
			// brightness[offset] = brightness_1[offset];
			depth_map[offset] = depth_map_2[offset];
		}
		else if(brightness_3[offset] < 255 && depth_map_3[offset] > 0)
		{
			// brightness[offset] = brightness_1[offset];
			depth_map[offset] = depth_map_3[offset];
		}
		else if(brightness_4[offset] < 255 && depth_map_4[offset] > 0)
		{
			// brightness[offset] = brightness_1[offset];
			depth_map[offset] = depth_map_4[offset];
		}
		else
		{	
			// brightness[offset] = brightness_2[offset];
			depth_map[offset] = depth_map_5[offset];
		}
		//没有深度则用最亮的深度值
		if (depth_map[offset] <= 0)
		{
			depth_map[offset] = depth_map_0[offset];
		}
	}
}

__global__ void cuda_merge_hdr_depth_5(const float*  depth_map_0,const float*  depth_map_1,const float*  depth_map_2,
	const float*  depth_map_3,const float*  depth_map_4,
	const unsigned char* brightness_0,const unsigned char* brightness_1,const unsigned char* brightness_2,
	const unsigned char* brightness_3,const unsigned char* brightness_4,
	uint32_t img_height, uint32_t img_width, float* const depth_map,unsigned char * const brightness)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{



		float pixel= 0;
		pixel +=  brightness_0[offset];
		pixel +=  brightness_1[offset];
		pixel +=  brightness_2[offset];
		pixel +=  brightness_3[offset];
		pixel +=  brightness_4[offset];

		pixel/= 5.0;


		brightness[offset] = pixel;

		if(brightness_0[offset] < 255 && depth_map_0[offset] > 0)
		{
			// brightness[offset] = brightness_0[offset];
			depth_map[offset] = depth_map_0[offset];
		}

		else if(brightness_1[offset] < 255 && depth_map_1[offset] > 0)
		{
			// brightness[offset] = brightness_1[offset];
			depth_map[offset] = depth_map_1[offset];
		}
		else if(brightness_2[offset] < 255 && depth_map_2[offset] > 0)
		{
			// brightness[offset] = brightness_1[offset];
			depth_map[offset] = depth_map_2[offset];
		}
		else if(brightness_3[offset] < 255 && depth_map_3[offset] > 0)
		{
			// brightness[offset] = brightness_1[offset];
			depth_map[offset] = depth_map_3[offset];
		}
		else
		{	
			// brightness[offset] = brightness_2[offset];
			depth_map[offset] = depth_map_4[offset];
		}
		//没有深度则用最亮的深度值
		if (depth_map[offset] <= 0)
		{
			depth_map[offset] = depth_map_0[offset];
		}
	}
}


__global__ void cuda_merge_hdr_depth_4(const float*  depth_map_0,const float*  depth_map_1,const float*  depth_map_2,const float*  depth_map_3,
	const unsigned char* brightness_0,const unsigned char* brightness_1,const unsigned char* brightness_2,const unsigned char* brightness_3,
	uint32_t img_height, uint32_t img_width, float* const depth_map,unsigned char * const brightness)
	{
		const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
		const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
		const unsigned int offset = idy * img_width + idx;
	
		if (idx < img_width && idy < img_height)
		{
	
	
	
			float pixel= 0;
			pixel +=  brightness_0[offset];
			pixel +=  brightness_1[offset];
			pixel +=  brightness_2[offset];
			pixel +=  brightness_3[offset];
	
			pixel/= 4.0;
	
	
			brightness[offset] = pixel;
	
			if(brightness_0[offset] < 255 && depth_map_0[offset] > 0)
			{
				// brightness[offset] = brightness_0[offset];
				depth_map[offset] = depth_map_0[offset];
			}
	
			else if(brightness_1[offset] < 255 && depth_map_1[offset] > 0)
			{
				// brightness[offset] = brightness_1[offset];
				depth_map[offset] = depth_map_1[offset];
			}
			else if(brightness_2[offset] < 255 && depth_map_2[offset] > 0)
			{
				// brightness[offset] = brightness_1[offset];
				depth_map[offset] = depth_map_2[offset];
			}
			else
			{	
				// brightness[offset] = brightness_2[offset];
				depth_map[offset] = depth_map_3[offset];
			}
			//没有深度则用最亮的深度值
			if (depth_map[offset] <= 0)
			{
				depth_map[offset] = depth_map_0[offset];
			}
		}
	}

__global__ void cuda_merge_hdr_depth_3(const float*  depth_map_0,const float*  depth_map_1,const float*  depth_map_2,const unsigned char* brightness_0,const unsigned char* brightness_1,
	const unsigned char* brightness_2,uint32_t img_height, uint32_t img_width, float* const depth_map,unsigned char * const brightness)
	{
		const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
		const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
		const unsigned int offset = idy * img_width + idx;
	
		if (idx < img_width && idy < img_height)
		{
	
	
	
			float pixel= 0;
			pixel +=  brightness_0[offset];
			pixel +=  brightness_1[offset];
			pixel +=  brightness_2[offset];
	
			pixel/= 3.0;
	
	
			brightness[offset] = pixel;
	
			if(brightness_0[offset] < 255 && depth_map_0[offset] > 0)
			{
				// brightness[offset] = brightness_0[offset];
				depth_map[offset] = depth_map_0[offset];
			}
	
			else if(brightness_1[offset] < 255 && depth_map_1[offset] > 0)
			{
				// brightness[offset] = brightness_1[offset];
				depth_map[offset] = depth_map_1[offset];
			}
			else
			{	
				// brightness[offset] = brightness_2[offset];
				depth_map[offset] = depth_map_2[offset];
			}
				//没有深度则用最亮的深度值
			if(depth_map[offset]<= 0)
			{
				depth_map[offset] = depth_map_0[offset];
			}
	
		}
	}

__global__ void cuda_merge_hdr_depth_2(const float*  depth_map_0,const float*  depth_map_1,const unsigned char* brightness_0,const unsigned char* brightness_1,
	uint32_t img_height, uint32_t img_width, float* const depth_map,unsigned char * const brightness)
	{
		const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
		const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
		const unsigned int offset = idy * img_width + idx;
	
		if (idx < img_width && idy < img_height)
		{
	
			float pixel= 0;
			pixel +=  brightness_0[offset];
			pixel +=  brightness_1[offset];
	
			pixel/= 2.0;
	
	
			brightness[offset] = pixel;
	
			if(brightness_0[offset] < 255 && depth_map_0[offset] > 0)
			{
				// brightness[offset] = brightness_0[offset];
				depth_map[offset] = depth_map_0[offset];
			}
			else 
			{
				// brightness[offset] = brightness_1[offset];
				depth_map[offset] = depth_map_1[offset];
			}

			//没有深度则用最亮的深度值
			if(depth_map[offset]<= 0)
			{
				depth_map[offset] = depth_map_0[offset];
			}

		}
	}



__global__ void cuda_merge_hdr_6(const unsigned char*  phase_map_0,const unsigned char*  phase_map_1,const unsigned char*  phase_map_2,
	const unsigned char*  phase_map_3,const unsigned char*  phase_map_4,const unsigned char*  phase_map_5,
	const unsigned char* brightness_0,const unsigned char* brightness_1,const unsigned char* brightness_2,
	const unsigned char* brightness_3,const unsigned char* brightness_4,const unsigned char* brightness_5,
	uint32_t img_height, uint32_t img_width, unsigned char* const phase_map_out)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{

		if(brightness_0[offset] < 255)
		{
			// brightness[offset] = brightness_0[offset];
			phase_map_out[offset] = phase_map_0[offset];
		}

		else if(brightness_1[offset] < 255)
		{
			// brightness[offset] = brightness_1[offset];
			phase_map_out[offset] = phase_map_1[offset];
		}
		else if(brightness_2[offset] < 255)
		{
			// brightness[offset] = brightness_1[offset];
			phase_map_out[offset] = phase_map_2[offset];
		}
		else if(brightness_3[offset] < 255)
		{
			// brightness[offset] = brightness_1[offset];
			phase_map_out[offset] = phase_map_3[offset];
		}
		else if(brightness_4[offset] < 255)
		{
			// brightness[offset] = brightness_1[offset];
			phase_map_out[offset] = phase_map_4[offset];
		}
		else
		{	
			// brightness[offset] = brightness_2[offset];
			phase_map_out[offset] = phase_map_5[offset];
		}
		//没有深度则用最亮的深度值
		if (phase_map_out[offset] <= 0)
		{
			phase_map_out[offset] = phase_map_0[offset];
		}
	}
}

__global__ void cuda_merge_hdr_5(const unsigned char*  phase_map_0,const unsigned char*  phase_map_1,const unsigned char*  phase_map_2,
	const unsigned char*  phase_map_3,const unsigned char*  phase_map_4,
	const unsigned char* brightness_0,const unsigned char* brightness_1,const unsigned char* brightness_2,
	const unsigned char* brightness_3,const unsigned char* brightness_4,
	uint32_t img_height, uint32_t img_width, unsigned char* const phase_map_out)
{
	const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
	const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
	const unsigned int offset = idy * img_width + idx;

	if (idx < img_width && idy < img_height)
	{
		if(brightness_0[offset] < 255)
		{
			// brightness[offset] = brightness_0[offset];
			phase_map_out[offset] = phase_map_0[offset];
		}

		else if(brightness_1[offset] < 255)
		{
			// brightness[offset] = brightness_1[offset];
			phase_map_out[offset] = phase_map_1[offset];
		}
		else if(brightness_2[offset] < 255)
		{
			// brightness[offset] = brightness_1[offset];
			phase_map_out[offset] = phase_map_2[offset];
		}
		else if(brightness_3[offset] < 255)
		{
			// brightness[offset] = brightness_1[offset];
			phase_map_out[offset] = phase_map_3[offset];
		}
		else
		{	
			// brightness[offset] = brightness_2[offset];
			phase_map_out[offset] = phase_map_4[offset];
		}
		//没有深度则用最亮的深度值
		if (phase_map_out[offset] <= 0)
		{
			phase_map_out[offset] = phase_map_0[offset];
		}
	}
}


__global__ void cuda_merge_hdr_4(const unsigned char*  phase_map_0,const unsigned char*  phase_map_1,const unsigned char*  phase_map_2,const unsigned char*  phase_map_3,
	const unsigned char* brightness_0,const unsigned char* brightness_1,const unsigned char* brightness_2,const unsigned char* brightness_3,
	uint32_t img_height, uint32_t img_width, unsigned char* const phase_map_out)
	{
		const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
		const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
		const unsigned int offset = idy * img_width + idx;
	
		if (idx < img_width && idy < img_height)
		{
	
			if(brightness_0[offset] < 255)
			{
				// brightness[offset] = brightness_0[offset];
				phase_map_out[offset] = phase_map_0[offset];
			}
	
			else if(brightness_1[offset] < 255)
			{
				// brightness[offset] = brightness_1[offset];
				phase_map_out[offset] = phase_map_1[offset];
			}
			else if(brightness_2[offset] < 255)
			{
				// brightness[offset] = brightness_1[offset];
				phase_map_out[offset] = phase_map_2[offset];
			}
			else
			{	
				// brightness[offset] = brightness_2[offset];
				phase_map_out[offset] = phase_map_3[offset];
			}
			//没有深度则用最亮的深度值
			if (phase_map_out[offset] <= 0)
			{
				phase_map_out[offset] = phase_map_0[offset];
			}
		}
	}

__global__ void cuda_merge_hdr_3(const unsigned char*  phase_map_0,const unsigned char*  phase_map_1,const unsigned char*  phase_map_2,const unsigned char* brightness_0,const unsigned char* brightness_1,
	const unsigned char* brightness_2,uint32_t img_height, uint32_t img_width, unsigned char* const phase_map_out)
	{
		const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
		const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
		const unsigned int offset = idy * img_width + idx;
	
		if (idx < img_width && idy < img_height)
		{
			if(brightness_0[offset] < 255)
			{
				// brightness[offset] = brightness_0[offset];
				phase_map_out[offset] = phase_map_0[offset];
			}
	
			else if(brightness_1[offset] < 255)
			{
				// brightness[offset] = brightness_1[offset];
				phase_map_out[offset] = phase_map_1[offset];
			}
			else
			{	
				// brightness[offset] = brightness_2[offset];
				phase_map_out[offset] = phase_map_2[offset];
			}
				//没有深度则用最亮的深度值
			if(phase_map_out[offset]<= 0)
			{
				phase_map_out[offset] = phase_map_0[offset];
			}
	
		}
	}

__global__ void cuda_merge_hdr_2(const unsigned char*  phase_map_0,const unsigned char*  phase_map_1,const unsigned char* brightness_0,const unsigned char* brightness_1, uint32_t img_height, uint32_t img_width, unsigned char* const phase_map_out)
	{
		const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
		const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
		const unsigned int offset = idy * img_width + idx;
	
		if (idx < img_width && idy < img_height)
		{	
			if(brightness_0[offset] < 255)
			{
				// brightness[offset] = brightness_0[offset];
				phase_map_out[offset] = phase_map_0[offset];
			}
			else 
			{
				// brightness[offset] = brightness_1[offset];
				phase_map_out[offset] = phase_map_1[offset];
			}

			//没有深度则用最亮的深度值
			if(phase_map_out[offset]<= 0)
			{
				phase_map_out[offset] = phase_map_0[offset];
			}

		}
	}


__global__ void cuda_count_sum_pixel(const unsigned char* brightness,uint32_t img_height, uint32_t img_width, float* sum_pixels)
{
		const unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;
		const unsigned int idy = blockIdx.y * blockDim.y + threadIdx.y;
		const unsigned int offset = idy * img_width + idx;
	
		if (idx < img_width && idy < img_height)
		{ 
			*sum_pixels +=  brightness[offset];  
		}
}