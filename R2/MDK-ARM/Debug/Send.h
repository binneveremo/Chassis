#ifndef __SEND_H
#define __SEND_H

#include "Television.h"
#include "Fake_Rtos.h"
#include "stdbool.h"
#include <string.h>
#include "Global.h"
#include "Basket.h"
#include "usart.h"
#include <stdio.h>

#define R1_Data_Num 10
#define Send_Put_Data(index,data) {send.convert.float_data[index] = data;}

struct Send{
	struct{
		char send_flag;
		unsigned char send[40];
	}Debug;
	struct{
		unsigned char send[R1_Data_Num];
		unsigned char receive[R1_Data_Num];
		struct Point pos;
		struct Point net;
		bool get_dataflag; 
		bool request_flag;
	}R1_Exchange;
	uint8_uint32_float_union convert;
};
extern struct Send send;
void Wireless_init(void);
void Send_MessageToR1(void);
void R1ExchangeData_Decode(UART_HandleTypeDef *huart);

void Send_Float_Data(char num);
#endif
