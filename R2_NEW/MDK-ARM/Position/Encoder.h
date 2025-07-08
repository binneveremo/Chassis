#ifndef __ENCODER_H
#define __ENCODER_H


#include "string.h"
#include "stdbool.h"
#define ENC_NUM 2
struct Odometer {
	int o1_pre;
	int o2_pre;
	int o1_row;
	int o2_row;
	//码盘的差值
	float do1;
	float do2;
	//场地坐标系的插值
	float dx_field;
	float dy_field;
	//码盘的场地坐标
	float enc_x_field;
	float enc_y_field;
	//车体的场地坐标
	float car_x_field;
	float car_y_field;
	//上线标志位
	char xenc_online;
	char yenc_online;
};
extern struct Odometer odometer;
void Encoder_XY_VX_VY_Cal(int dt);
void Encoder_Init(void);
void Odometer_Clear(char * ifarmor);
void Get_Encoder_Data(int id,unsigned char * data);

#endif
