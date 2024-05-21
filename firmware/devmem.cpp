/**
 * @addtogroup module_devmem
 * @{
 */

 /**
 * @file
 * @brief 内存管理工具，仿照标准linux devmem 工具进行改良，可以自由读写linux内存数据。
 * @details 驱动接口。
 * @version 1.1.0
 * @author sky.houfei
 * @date 2019-8-6
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <stdbool.h>
#include "devmem.h"

//*****************************************************************************
#define FATAL do { fprintf(stderr, "Error at line %d, file %s (%d) [%s]", __LINE__, __FILE__, errno, strerror(errno)); exit(1); } while(0)
 
#define MAP_SIZE 4096UL  			//映射的内存区大小（一般为一个叶框大小）
#define MAP_MASK (MAP_SIZE - 1)   	//MAP_MASK = 0XFFF

//*****************************************************************************
/**
* @brief 直接写入到内存实际的物理地址。
* @details 通过 mmap 映射关系，找到对应的内存实际物理地址对应的虚拟地址，然后写入数据。
* 写入长度，每次最低4字节
* @param[in] writeAddr, unsigned long, 需要操作的物理地址。
* @param[in] buf，unsigned long *, 需要写入的数据。
* @param[in] len，unsigned long, 需要写入的长度，4字节为单位。
* @return ret, int, 如果发送成功，返回0，发送失败，返回-1。
*/
int Devmem_Write(unsigned long writeAddr, unsigned long* buf, unsigned long len)
{
	int i = 0;
	int ret = 0;
    int fd;
	void *map_base, *virt_addr; 
	unsigned long addr = writeAddr;
    int offset_len = 0;

	if(len == 0)
	{
        printf("%s %s %d, len = 0", __FILE__, __func__, __LINE__);
        return -1;
    }
	
	if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1)
	{
		fprintf(stderr, "Error at line %d, file %s (%d) [%s]", __LINE__, __FILE__, errno, strerror(errno));
		return -1;
    }
	
	/* Map one page */ //将内核空间映射到用户空间
    map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, addr & ~MAP_MASK);
    if(map_base == (void *) -1)
	{
		fprintf(stderr, "Error at line %d, file %s (%d) [%s]", __LINE__, __FILE__, errno, strerror(errno));
		close(fd);
		return -1;
    }
	
		// 发送实际数据内容
 	for (i = 0; i < len; i++)
 	{
		// 翻页处理
        if(offset_len >= MAP_MASK)
        { 
            offset_len = 0;
            if(munmap(map_base, MAP_SIZE) == -1)
        	{
        		fprintf(stderr, "Error at line %d, file %s (%d) [%s]", __LINE__, __FILE__, errno, strerror(errno));
        		close(fd);
        		return -1;
        	}
            map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, addr & ~MAP_MASK);
            if(map_base == (void *) -1)
        	{
        		fprintf(stderr, "Error at line %d, file %s (%d) [%s]", __LINE__, __FILE__, errno, strerror(errno));
        		close(fd);
        		return -1;
            }
            printf("map_base over 4k = [%p].", map_base);		// 翻页打印提示
        }
    
		virt_addr = map_base + (addr & MAP_MASK);	// 映射地址
		*((unsigned long *) virt_addr) = buf[i]; 	// 写入数据
//		addr += 4;
        offset_len += 4;
	}
	
	    
	if(munmap(map_base, MAP_SIZE) == -1)
	{
		fprintf(stderr, "Error at line %d, file %s (%d) [%s]", __LINE__, __FILE__, errno, strerror(errno));
		return -1;
	}
	
    close(fd);
	return 0;
}


/**
* @brief 从实际物理地址读取数据。
* @details 通过 mmap 映射关系，找到对应的实际物理地址对应的虚拟地址，然后读取数据。
* 读取长度，每次最低4字节。
* @param[in] readAddr, unsigned long, 需要操作的物理地址。
* @param[out] buf，unsigned char *, 读取数据的buf地址。
* @param[in] bufLen，unsigned long , buf 参数的容量，4字节为单位，如 unsigned long buf[100]，那么最大能接收100个4字节。
* 用于避免因为buf容量不足，导致数组越界之类的软件崩溃问题。
* @return len，unsigned long, 读取的数据长度，字节为单位。如果读取出错，则返回0，如果正确，则返回对应的长度。
*/
int Devmem_Read(unsigned long readAddr, unsigned long* buf, unsigned long len)
{
	int i = 0;
    int fd,ret;
    int offset_len = 0;
    void *map_base, *virt_addr; 
	off_t addr = readAddr;
	unsigned long littleEndianLength = 0;
	
	if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1)
	{
		fprintf(stderr, "Error at line %d, file %s (%d) [%s]", __LINE__, __FILE__, errno, strerror(errno));
		return 0;
    }
	
    /* Map one page */ //将内核空间映射到用户空间
    map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, addr & ~MAP_MASK);
    if(map_base == (void *) -1)
	{
		fprintf(stderr, "Error at line %d, file %s (%d) [%s]", __LINE__, __FILE__, errno, strerror(errno));
		close(fd);
		return 0;
    }
	
	for (i = 0; i < len; i++)
 	{
		// 翻页处理
        if(offset_len >= MAP_MASK)
        {
            offset_len = 0;
            if(munmap(map_base, MAP_SIZE) == -1)
        	{
        		fprintf(stderr, "Error at line %d, file %s (%d) [%s]", __LINE__, __FILE__, errno, strerror(errno));
        		close(fd);
        		return 0;
        	}
            map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, addr & ~MAP_MASK);
            if(map_base == (void *) -1)
        	{
        		fprintf(stderr, "Error at line %d, file %s (%d) [%s]", __LINE__, __FILE__, errno, strerror(errno));
        		close(fd);
        		return 0;
            }
        }
    
		virt_addr = map_base + (addr & MAP_MASK);	// 将内核空间映射到用户空间操作
		buf[i] = *((unsigned long *) virt_addr);	// 读取数据
// 		addr += 4;
        offset_len += 4;
	}
	
	if(munmap(map_base, MAP_SIZE) == -1)
	{
		fprintf(stderr, "Error at line %d, file %s (%d) [%s]", __LINE__, __FILE__, errno, strerror(errno));
		close(fd);
		return 0;
	}
    close(fd);
	return i;
}
