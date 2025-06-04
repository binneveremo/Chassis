#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "Communication.h"
#include "Location.h"
#include "stdbool.h"
#include "string.h"
#include "Global.h"
#include "HO7213.h"
#include "mine.h"
#include "VESC.h"

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
#define behind_offset 24


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


struct Mark
{
	float p;
	float i;
	float istart;
	float iend;
	float ilimit;
	float outlimit;
	float gain;
	
	float itotal_x;
	float itotal_y;
	float outx;
	float outy;
	///////新加的测试参数
	float total_dis;
	float brake_ilimit;
	float brake_outlimit;
	float brake_distance;
	float brake_gain;
	float brake_percent;
};
struct Chassis{
	enum{
			safe,
			gamepad_standard,
			gamepad_noheader,
			basket_lock,
			dribble,
			back,
	}Control_Status;
	struct {
		struct VESC  drive[VESC_NUM];
		struct HO7213 turn[TURN_NUM];
	}motor;
	struct{
		char self_lock;
	  char GamePad_Slow;
		char GamePad_Accel;
		char GamePad_Inverse;
		char lock_reason[20];
	}Flagof;
	
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




void Get_VESC_Data(int id,unsigned char * data);
extern struct Chassis chassis;
extern struct Mark mark;
//泡点参数初始化
void Velocity_Run_Par_Init(void);
//跑点
void Velocity_Run(float vx,float vy,float angle);
void Velocity_Run_Test(float vx,float vy,float vr);
//纠正角度
float Correct_Angle(float target);
void Correct_Angle_Par_Init(void);
//舵轮的输出以及解码
void Turn_Motor_Decode(int id,unsigned char * data);
void VectorWheel_SetSpeed(void);
void VectorWheel_SetAngle(void);
void Chassis_Velocity_Out(float left,float front,float anticlock);
//设置目标点
//void Set_Target_Point(struct Point point);
//释放技能
void Release_Skill(int time);
//手柄遥控
void GamePad_Velocity_Standard(void);
void GamePad_Velocity_FreeNoheader(void);
void GamePad_Velocity_R1DirNoheader(void);
//到点判断
unsigned char Arrive_Point(struct Point point);
unsigned char Near_Point(struct Point point);
//自动自锁
void Self_Lock_Auto(void);
void Self_Lock_Out(char * lock_reason);

void Mark_PID_Par_Init(void);
void Position_With_Mark_PID_Run(void);

extern int Dribble_Begin;
void Dribble_Flow(void);
void Debug_Detect(void);


bool TurnMotor_InTurnPosition(void);
#endif



