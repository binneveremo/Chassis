#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "Communication.h"
#include "Location.h"
#include "stdbool.h"
#include "string.h"
#include "HO7213.h"
#include "mine.h"
#include "VESC.h"

#define Chassis_SelfLock(x) {chassis.lock.permit = x;}


#define VESC_NUM 4
#define TURN_NUM 4
#define front_wheel 0		
#define left_wheel 1
#define right_wheel 2
#define behind_wheel 3

#ifdef Carbon_Car

//转向轮 发送的ID
#define front_turn_send_id 2
#define left_turn_send_id 3
#define right_turn_send_id 4
#define behind_turn_send_id 1
//转向轮 回传的ID
#define front_turn_receive_id 102
#define left_turn_receive_id 103
#define right_turn_receive_id 104
#define behind_turn_receive_id 101
//驱动轮 VESC的ID
#define front_drive_id 7
#define left_drive_id 8
#define right_drive_id 9
#define behind_drive_id 6
//左转 偏置变大
#define front_offset 137
#define left_offset 40
#define right_offset 82
#define behind_offset -66


#else
//转向轮 发送的ID
#define front_turn_send_id 1
#define left_turn_send_id 3
#define right_turn_send_id 2
#define behind_turn_send_id 4
//转向轮 回传的ID
#define front_turn_receive_id 101
#define left_turn_receive_id 103
#define right_turn_receive_id 102
#define behind_turn_receive_id 104
//驱动轮 VESC的ID
#define front_drive_id 6
#define left_drive_id 8
#define right_drive_id 7
#define behind_drive_id 9
//左转 偏置变大
#define front_offset -13
#define left_offset 72
#define right_offset  100
#define behind_offset  -69

#endif
enum opposite_t{
	none,
	R1,
	self_basket,
	oppo_basket,
	forward,
};

struct Spot_t{
	struct {
		float p;
		
		float i;
		float istart;
		float iend;
		float ilimit;
		
		float outlimit;
		
		float fade_start;
		float fade_end;
		
		float lock_dis;
	}param;
	struct {
		float gain;
		float itotal_x;
		float itotal_y;
		float outx;
		float outy;
	}process;
};
extern struct Spot_t spot;
struct Chassis{
	enum{
		GamePad_Control,
		Auto_Control,
		Debug_Control,
	}Control_Status;
	struct {
		struct VESC  drive[VESC_NUM];
		struct HO7213 turn[TURN_NUM];
	}motor;
	struct{
		struct{
			char slow;
			char accel;
			char inverse;
			char standard;
			char noheader;
			char shutdown;
			bool rotate;
		}gamepad;
	}flagof;
	enum opposite_t opposite;
	struct {
		char permit;
		char flag;
		char reason[20];
	}lock;
};
struct correct_angle_t{
	float velocity_gain;
	float accel_gain;
	float p;
	float i;
	float d;
	float fade_max;
	float fade_min;
	float outlimit;
	float lock_angle;
	
	float error;
	float itotal;
	float ilimit;
	float istart;
	float iend;
};











#define Set_Target_Point(x) memcpy(&site.target, &x, sizeof(x))
#define TurnMotor_OffsetAngleInit() {\
	chassis.motor.turn[front_wheel].offset_angle  = front_offset;		\
	chassis.motor.turn[right_wheel].offset_angle  = right_offset;		\
	chassis.motor.turn[left_wheel].offset_angle   = left_offset;		\
	chassis.motor.turn[behind_wheel].offset_angle = behind_offset;	\
	for(int i = 0; i < TURN_NUM;i++)																\
		chassis.motor.turn[i].param.p = 155,chassis.motor.turn[i].param.d = 55;\
}


extern struct Chassis chassis;
extern struct Mark mark;
extern struct Spot_t spot_skill;
extern struct Spot_t spot_basket;
extern struct correct_angle_t cr_skill;
extern struct correct_angle_t cr_basket;


void Get_VESC_Data(int id,unsigned char * data);
//泡点参数初始化
//纠正角度
//舵轮的输出以及解码
void Turn_Motor_Decode(int id,unsigned char * data);
void VectorWheel_SetSpeed(void);
void VectorWheel_SetAngle(void);
void Chassis_Velocity_Out(float left,float front,float anticlock);
//设置目标点
//void Set_Target_Point(struct Point point);
//释放技能
//手柄遥控
void GamePad_Velocity_Control(void);

//自动自锁
void Self_Lock_Auto(void);
void Self_Lock_Out(char * lock_reason);
float Angle_Lock(float now,float target,struct correct_angle_t * cr);
void PositionWithAngle_Lock(struct Point now,struct Point target,struct Spot_t * spot,struct correct_angle_t * cr);

bool TurnMotor_InTurnPosition(void);


void Debug_Test(void);

#endif



