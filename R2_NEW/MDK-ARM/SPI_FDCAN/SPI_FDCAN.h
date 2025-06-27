#ifndef __SPI_FDCAN_H
#define __SPI_FDCAN_H

#include "drv_spi.h"
#include "fdcan.h"
extern FDCAN_HandleTypeDef hfdcan4;
#define FDCAN4_Init() DRV_SPI_Initialize(CAN_500K_1M, CAN_FIFO_CH1, CAN_FIFO_CH2);
struct SPI_hfdcan_t{
	int header;
	char data_length;
	unsigned char * data;
};
void Get_SPIFDCAN_Data(void);



#endif










