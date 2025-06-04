#ifndef __CAN_BSP_H
#define __CAN_BSP_H

#include "SPI_FDCAN.h"
#include "stdbool.h"
#include "Can_Bsp.h"
#include "fdcan.h"
extern FDCAN_HandleTypeDef hfdcan4;

int FDCAN_Send(FDCAN_HandleTypeDef *hfdcan,int head, char * idtype,unsigned char * send, char * cantype, int length, char * brs);
uint8_t FDCAN_SendData(FDCAN_HandleTypeDef *hfdcan, uint8_t *TxData, uint32_t StdId, uint32_t Length);
uint8_t FDCAN_SendData_Ext(FDCAN_HandleTypeDef *hfdcan, uint8_t *TxData, uint32_t ExtId, uint32_t Length, uint32_t Data_type);
#endif