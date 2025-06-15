#include "mySerial.h"
#include <string.h>
#include <stdlib.h>
#include "usart.h"
#include "dma.h"

//#include "Config.h"
#include "mngCommu.h"
#include "Send.h"

myUartStruct *UartStruct[5];
uint8_t Uart_num = 0;

// 串口数据请求队列
// 加入请求队列
uint16_t append_for_need(myUartStruct *UartStruct_x, uint8_t *store_addr, volatile int *length)
{
	uint16_t index = UartStruct_x->Uart_need_index;
	UartStruct_x->Uart_need_buf[index] = store_addr;
	UartStruct_x->Uart_need_cnt[index] = length;
	UartStruct_x->Uart_need_index = (UartStruct_x->Uart_need_index) + 1 % require_queue_length;
	return index;
}
void cancel_for_need(myUartStruct *UartStruct_x, uint16_t index)
{
	for(int i=index; i<require_queue_length-1; i++)
	{
		UartStruct_x->Uart_need_buf[i] = UartStruct_x->Uart_need_buf[i+1];
		UartStruct_x->Uart_need_cnt[i] = UartStruct_x->Uart_need_cnt[i+1];
	}
	UartStruct_x->Uart_need_buf[require_queue_length-1] = NULL;
	UartStruct_x->Uart_need_cnt[require_queue_length-1] = NULL;
	UartStruct_x->Uart_need_index--;
}

/*  串口接收  */
/*  串口中断回调函数  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	
	R1ExchangeData_Decode(huart);
	for(int i=0; i<Uart_num; i++)
	{
		if(huart->Instance == UartStruct[i]->huart->Instance)
		{
			// 切换缓冲区继续接收
			HAL_UARTEx_ReceiveToIdle_DMA(huart, UartStruct[i]->Uart_RxBuff[!UartStruct[i]->Uart_RxBuffIndex], BUFFER_SIZE);
//			HAL_UARTEx_ReceiveToIdle_DMA(huart, UartStruct[i]->Uart_RxBuff[!UartStruct[i]->Uart_RxBuffIndex], sizeof(recPkg));
			// 设置标志位与数据长度
			UartStruct[i]->Uart_RxFlag = 1;
			UartStruct[i]->Uart_Rx_Cnt = size;
			// 处理串口数据
			ArrangeSerialList();
			// 传递串口数据
			for(int i=0; i<require_queue_length; i++)
			{
				if(UartStruct[i]->Uart_need_buf[i] != 0)
				{
					memcpy(UartStruct[i]->Uart_need_buf[i], UartStruct[i]->Uart_RxBuff, UartStruct[i]->Uart_Rx_Cnt);
					*UartStruct[i]->Uart_need_cnt[i] = UartStruct[i]->Uart_Rx_Cnt;
				}
				else
					break;
			}
			memset(UartStruct[i]->Uart_need_buf, 0, sizeof(UartStruct[i]->Uart_need_buf));
			UartStruct[i]->Uart_need_index = 0;
			// 置零标志位
			UartStruct[i]->Uart_RxFlag = 0;
			UartStruct[i]->Uart_Rx_Cnt = 0;
			// 翻转缓冲区索引
			UartStruct[i]->Uart_RxBuffIndex = !UartStruct[i]->Uart_RxBuffIndex;
		}
	}
}
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance == huart1.Instance)
//	{
//		
//	}
//}

/*  串口初始化  */
uint8_t Serial_init(myUartStruct *UartStruct_x, UART_HandleTypeDef *huart)
{
		UartStruct_x->huart = huart;
		HAL_UART_Init(huart);
		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
		HAL_UARTEx_ReceiveToIdle_DMA(huart, UartStruct_x->Uart_RxBuff[0], BUFFER_SIZE);
		__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);  // 禁用传输过半中断
		UartStruct[Uart_num] = UartStruct_x;
		Uart_num++;
		return 0;
}
uint8_t Serial_reInit(myUartStruct *UartStruct_x)
{
		HAL_UART_Init(UartStruct_x->huart);
		__HAL_UART_ENABLE_IT(UartStruct_x->huart, UART_IT_IDLE);
		HAL_UARTEx_ReceiveToIdle_DMA(UartStruct_x->huart, UartStruct_x->Uart_RxBuff[0], BUFFER_SIZE);
		__HAL_DMA_DISABLE_IT(UartStruct_x->huart->hdmarx, DMA_IT_HT);  // 禁用传输过半中断
		return 0;
}
uint8_t Serial_DeInit(myUartStruct *UartStruct_x)
{
    // 停止 UART 的 DMA 接收
    HAL_UART_DMAStop(UartStruct_x->huart);
    // 禁用 UART 的 IDLE 中断
    __HAL_UART_DISABLE_IT(UartStruct_x->huart, UART_IT_IDLE);
    // 解除 UART 初始化
    if (HAL_UART_DeInit(UartStruct_x->huart) != HAL_OK) {
        // 如果解除初始化失败，可以在此处理错误
        return 1;  // 表示失败
    }
    return 0;  // 表示成功
}

// 串口出错回调(出错则重置串口)
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	
	Wireless_init();
    // 检查错误类型
    if (huart->ErrorCode & HAL_UART_ERROR_FE) {
        // 处理帧错误 (Framing Error)
    }
    if (huart->ErrorCode & HAL_UART_ERROR_NE) {
        // 处理噪声错误 (Noise Error)
    }
    if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
        // 处理溢出错误 (Overrun Error)
    }
    if (huart->ErrorCode & HAL_UART_ERROR_PE) {
        // 处理奇偶校验错误 (Parity Error)
    }
		// 重置串口
		for(int i=0; i<Uart_num; i++)
		{
			if(huart->Instance == UartStruct[i]->huart->Instance)
			{
				Serial_DeInit(UartStruct[i]);
				memset(UartStruct[i]->Uart_RxBuff, 0x00, BUFFER_SIZE*2);
				UartStruct[i]->Uart_RxBuffIndex = 0;
				Serial_reInit(UartStruct[i]);
			}
		}
}

/* 串口重定向 */
//int fputc(int ch, FILE *f){
//	uint8_t temp[1] = {ch};
//	HAL_UART_Transmit(&TX_Uart, temp, 1, 0xffff);
//	return ch;
//}
int fgetc(FILE * f)
{
	uint8_t ch = 0;
	HAL_UART_Receive(&TX_Uart,&ch, 1, 0xffff);
	return ch;
}

/* 数据处理 */
__weak void ArrangeSerialList(){
	
}