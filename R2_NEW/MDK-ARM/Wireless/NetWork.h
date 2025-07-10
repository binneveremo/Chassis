#pragma once

#include "Television.h"
#include "Fake_Rtos.h"
#include "stdbool.h"
#include <string.h>
#include "Global.h"
#include "Basket.h"
#include "NetWork.h"
#include <stdio.h>

#define R1_Data_Num 19


#define Send_Put_Data(index,data) {send.convert.float_data[index] = data;}

struct Send{
	struct{
		char send_flag;
		unsigned char send[40];
	}Debug;
	uint8_uint32_float_union convert;
};
extern struct Send send;

struct R1_t {
	struct {
		unsigned char buffer[R1_Data_Num];
		struct Point shoot_pos;
		float shoot_rpm;
		float shoot_incar_time;
		char shoot_flag;
	}receive;
	struct {
		unsigned char buffer[R1_Data_Num];
		enum {
			send_real_time,
			send_once,
			send_none,
		}mode;
		struct {
			char request;
		}flagof;
		struct Point target;
	}send;
	struct {
		float ball_offset_time;
		float ball_fly_time;
		float ball_incar_time;
		float ball_total_time;
	}expect;
	struct {
		float distance;
		float opposite_angle;
		char getdata_flag;
		int shoot_begin;
		int ball_fly_time;
	}deal;
};


extern struct R1_t shoot;


void Wireless_init(void);
void R1ExchangeData_Decode(UART_HandleTypeDef *huart);
void Send_MessageToR1(void);
void Send_Float_Data(char num);
double Polynomial_4ExpectBallFlyingTime(float distance);
double Polynomial_4ExpectBallIncarTime(float distance);
void Exchange_With_R1(void);
void Set_SendMode(struct Point target,char * mode,bool request);
void BallTime_Cal(void);
void Send_ReceiveBallMessage(void);
















