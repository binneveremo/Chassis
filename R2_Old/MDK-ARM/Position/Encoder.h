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
	/*需要包含以下变量
	1.每个码盘的dx dy也就是原生数据的dx dy
	2.计算到场地坐标系的dx dy
	3.补偿之后的dx dy
	*//////////////////////////////
	
	float do1;
	float do2;
	float dx_field;
	float dy_field;
	float x_field;
	float y_field;

	float offset_angle;
	//x轴行驶过的里程
	float xdis;
	float ydis;
	bool xenc_online;
	bool yenc_online;
};
extern struct Odometer odometer;
void Encoder_XY_VX_VY_Cal(int dt);
void Encoder_Init(void);
void Odometer_Clear(char * ifarmor);
void Get_Encoder_Data(int id,unsigned char * data);

#endif
