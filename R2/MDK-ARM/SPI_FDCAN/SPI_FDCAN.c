#include "SPI_FDCAN.h"
FDCAN_HandleTypeDef hfdcan4;
struct SPI_hfdcan_t spi_hfdcan;
canfd_frame RXFRAME;
extern void SPI_FDCANCallback(int header,unsigned char * Row_Data);

void Get_SPIFDCAN_Data(void){
	MCP2518FD_ReceiveData(&RXFRAME, CAN_FIFO_CH2);
	spi_hfdcan.data = RXFRAME.data;
	spi_hfdcan.header = RXFRAME.can_id;
	spi_hfdcan.data_length = RXFRAME.d_len;
	SPI_FDCANCallback(spi_hfdcan.header,spi_hfdcan.data);
}

















