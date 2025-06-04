#ifndef __FLOW_H
#define __FLOW_H

#include "Chassis.h"
#include "Send.h"

struct Flow {
	struct {
		char end;
		char received;
		char jumped;
		char defend_send;
		char jump_send;
		char back;
		char dribble;
		char stick_ball;
	}flag_of;
	enum{
		basketpos_lock = 1,
		turn_ready,
		receiving,
		oppositebasket,
		dunk,
		end
	}state;
	
};
extern struct Flow flow;
#define Flow_State_Reset() (flow.state = basketpos_lock)



void ControlStatus_Detect(void);


void Flow(void);
void Tell_Guo_Xuan_Jia(char *message);
void Car_State_Decode(int id,unsigned char * data);

void Velocity_Test(void);
void Debug_Flow(void);
void Flow_Test(void);
void Flow_Reset(void);

void Run_Point_Test(void);
//////////////////////雷达测试相关函数////////////////////////////////
void Ladar_Offset_Flow(void);
void Ladar_Offset_Test_Clear(void);
extern char ladar_offset_finish_flag;
///////////////////////编码器测试相关函数//////////////////////////////
void Odometer_Offset_Flow(void);
void Odometer_Offset_Test_Clear(void);
extern char Odometer_offset_finish_flag;
void Flow_Test2(void);
void Back_ToReset(void);
void Back(void);
void Dribbble_Flow(void);

void Basket_Flow(void);

void Auto_Basket_Lock(void);
#endif



