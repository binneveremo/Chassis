#pragma once

#include "Television.h"
#include "Fake_Rtos.h"
#include "stdbool.h"
#include <string.h>
#include "Global.h"
#include "Basket.h"
#include "NetWork.h"
#include <stdio.h>

#define R1_Data_Num 15
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
		float ball_fly_time;
		float distance;
		float shoot_rpm;
		int shoot_begin;
		bool get_dataflag; 
		bool request_flag;
		struct {
			float ball_fly_time;
			float ball_incar_time;
		}except;
	}R1_Exchange;
	uint8_uint32_float_union convert;
};
extern __attribute__((section(".sram_data"))) struct Send send;
void Wireless_init(void);
void Send_MessageToR1(void);
void R1ExchangeData_Decode(UART_HandleTypeDef *huart);
void Send_ReceiveBallMessage(void);
void Send_Float_Data(char num);


