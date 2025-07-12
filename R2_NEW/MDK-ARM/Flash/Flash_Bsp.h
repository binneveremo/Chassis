#pragma once 

//提供内部flash的sector管理
//sram_buf的大小必须是32的整数倍，最大为128kb，这里取kb的整数倍
#define FLASHBSP_SRAMBUF_SIZE_KB 1

#define FLASHBSP_SRAMBUF_SIZE (FLASHBSP_SRAMBUF_SIZE_KB * 1024)
typedef struct
{
   volatile unsigned int sector;                           //flash扇区
   volatile unsigned char sram_buf[FLASHBSP_SRAMBUF_SIZE];   //sram缓冲区
	 struct {
		unsigned char error;
	 }flagof;
	
} FlashBsp_t;

void FlashBsp_Init(void);   //初始化flash_bsp，须给出sector(FLASH_SECTOR_X)
void FlashBsp_Write(unsigned char* data, unsigned int sram_addr, unsigned int size);    //将数据写入sram缓冲区，sram_addr为相对于sram_buf缓冲区的偏移地址
void FlashBsp_Read(unsigned char* data, unsigned int sram_addr, unsigned int size);     //从sram缓冲区读取数据，sram_addr为相对于sram_buf缓冲区的偏移地址
void FlashBsp_Program(void);                                                    //将sram缓冲区的数据写入flash
void FlashBsp_Erase(void);                                                      //擦除flash并清空sram缓冲区                                                  //将flash的数据刷新到sram缓冲区




