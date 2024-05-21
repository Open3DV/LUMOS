#ifndef DEV_MEM_H
#define DEV_MEM_H

#define AXI_VER_ADDR				0x0000                  // 
#define AXI_CMD_ADDR				0x0010
#define AXI_FREQ_ADDR				0x0011
#define AXI_OUT_ADDR				0x0012
#define AXI_TIME_ADDR				0x0013
#define AXI_CYCLES_NUM_ADDR			0x0014
#define AXI_PRE_POINT_NUM_ADDR		0x0015
#define AXI_SCAN_POINT_NUM_ADDR	    0x0016
#define AXI_LASER_OUT_POS_ADDR		0x0017
#define AXI_TRIGGER_OUT_POS_ADDR	0x0018
#define AXI_LASER_LEVEL_ADDR		0x0019

#define AXI_LASER_INC_ADDR			0x001a
#define AXI_LASER_DEC_ADDR			0x001b

#define AXI_BANK_ADDR				0x0030
#define AXI_ADDR_ADDR				0x0031
#define AXI_DATA_ADDR				0x0032
#define AXI_BRIGHTNESS_ADDR			0x001F
#define AXI_DARKNESS_ADDR			0x001E
#define AXI_CLOSED_CURRENT_ADDR		0x001D

int Devmem_Write(unsigned long writeAddr, unsigned long* buf, unsigned long len);

int Devmem_Read(unsigned long readAddr, unsigned long* buf, unsigned long len);


#endif