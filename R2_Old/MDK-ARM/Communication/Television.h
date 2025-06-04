#ifndef __TELEVISION_H
#define __TELEVISION_H

#include "Location.h"
#include "Chassis.h"
#include "Can_Bsp.h"
#include "Kalman.h"
#include "Guard.h"
#include "string.h"
#include "Flow.h"

#define BasketAndPos_id 0xA1




#define site_id  0x201
#define reset_id  0x202
#define basket_id 0xA1
#define pos_id    0xA2
#define online_id 0xA3


#define Vision_Reset()  (FDCAN_Send(&hfdcan3,reset_id,"STD",NULL,"FD",4,"OFF"))


struct Vision{
	struct {
		char online_flag;
		char get_flag;
		unsigned char data[24];
		float height;
		struct Point basket_vfield;
		struct Point ladar_vfield;
		struct Point car_vfield;
		struct Point car_vfieldinterpenc;
	}basket;
	struct {
		char online_flag;
		unsigned char data[16];
		struct Point ladar_field;
		struct Point car_field;
	}pos;
	uint8_uint32_float_union convert;
	unsigned char send[16];
};
extern struct Vision vision;
void Vision_Basket_Decode(void);
void Get_Vision_Data(int header,unsigned char * data);
void Ladar_Decode(void);
#define Vision_Flag_Clear() (vision.pos.online_flag = NONE,vision.basket.online_flag = NONE)
void Send_Velocity_Vision(void);
void LadarPosInterpolation(int dt);
#endif


