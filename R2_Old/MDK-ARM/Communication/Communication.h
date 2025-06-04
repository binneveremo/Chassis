#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H
#include "Location.h"
#include "Can_Bsp.h"
#include "Global.h"
#include "string.h"
#include "Interact.h"

extern struct R1_Exchange R1;
struct R1_Exchange{
	float x;
	float y;
	float oppsite_angle;
	char receive_ball_flag;
	uint8_uint32_float_union convert;
};
extern struct R1_Data r1;
void Tell_Yao_Xuan(char *message);
void Test_Communication(int * test);
void Get_R1_Data(unsigned char * data);



#endif
