#ifndef __TELEVISION_H
#define __TELEVISION_H

#include "Location.h"
#include "Chassis.h"
#include "Can_Bsp.h"
#include "Guard.h"
#include "string.h"
#include "Flow.h"

#define BasketAndPos_id 0xA1

#define site_id  0x201
#define vision_reset_id  0x202
#define basket_id 0xA1
#define position_id  0xA2
#define online_id 0xA3

#define Vision_Flag_Clear() {vision.basketlock.online_flag  = false,vision.position.online_flag = false;}
#define Vision_Reset() {FDCAN_Send(&hfdcan4,vision_reset_id,"STD",NULL,"FD",4,"OFF");}

struct Vision{
	struct {
		char online_flag;
		unsigned char data[20];
	}basketlock;
	struct {
		char online_flag;
		unsigned char data[24];
	}position;
	struct {
		float height;
		struct Point carcenter_field;
		struct Point carcenter_fieldinterp;
	}field;
	struct {
		struct Point basket_visual;
		struct Point ladar_visual;
	}visual;
	struct{
		float basket_xoffset;
		float basket_yoffset;
		float ladar2siteangleoffset;
	}param;
	struct {
		char gyro_offset_angle_init;
		char used_flag;
	}flagof;
	int header;
	uint8_uint32_float_union convert;
};





//坐标系说明
/*
视觉坐标系
vfield
basket_vfield
ladar_vfield
car_vfield
场地坐标系
car_field
*/
extern __attribute__((section(".sram_data"))) struct Vision vision;
void Vision_Basket_Decode(void);
void Get_Vision_Data(int header,unsigned char * data);
void Ladar_Decode(void);
void Send_Velocity_Vision(void);
void Vision_Filed_Basket_XY_Cal(int dt);
float Easy_Filter(float x);



#endif


