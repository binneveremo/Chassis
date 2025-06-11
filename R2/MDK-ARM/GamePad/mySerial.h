#pragma once
#include "main.h"
#include <stdio.h>
#include "usart.h"

#define BUFFER_SIZE 500
#define require_queue_length 20

typedef struct
{
	// 串口端口
	UART_HandleTypeDef *huart;
	// 串口数据接收
	uint8_t Uart_RxBuff[2][BUFFER_SIZE];		//接收缓冲 双缓冲
	uint8_t Uart_RxFlag;				// 串口接收标识符号
	uint8_t Uart_RxBuffIndex;		// 串口接收缓冲区索引
	uint16_t Uart_Rx_Cnt;								// 串口接收计数
	// 串口数据请求
	int Uart_need_index;
	uint8_t *Uart_need_buf[require_queue_length];
	volatile int *Uart_need_cnt[require_queue_length];
}myUartStruct;

// 加入请求队列
// 退出请求队列
uint16_t append_for_need(myUartStruct *UartStruct, uint8_t *store_addr, volatile int *length);
void cancel_for_need(myUartStruct *UartStruct_x, uint16_t index);

// 串口DMA-IDLE中断回调
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
// 串口接收错误回调
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

// 串口重定向
int fputc(int ch, FILE *f);
int fgetc(FILE * f);

// 串口初始化 / 反初始化
uint8_t Serial_init(myUartStruct *UartStruct_x, UART_HandleTypeDef *huart);
uint8_t Serial_DeInit(myUartStruct *UartStruct_x);

// 串口处理
void ArrangeSerialList();
