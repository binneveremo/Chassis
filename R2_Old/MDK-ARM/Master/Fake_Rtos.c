#include "Communication.h"
#include "Television.h"
#include "Fake_Rtos.h"
#include "Location.h"
#include "Encoder.h"
#include "Correct.h"
#include "Chassis.h"
#include "Global.h"
#include "Flow.h"
#include "mine.h"
#include "Send.h"
#include "stdio.h"
#include "Gyro.h"
#include "RGB.h"
#include "SPI_FDCAN.h"

#include "HighTorque.h"
#include "CPU_Load.h"
#include "Interact.h"
#include "Basket.h"

void motor_control(void const * argument)
{
   for(;;)
  {
		switch(chassis.Control_Status){
			case gamepad_standard:
				GamePad_Velocity_Standard();    
			break;
			case gamepad_noheader:
				GamePad_Velocity_FreeNoheader();
			break;
			case basket_lock:
				Basket_Flow();
			break;
			case back:
				Back();
			break;
			case dribble:
				Dribbble_Flow();
			break;
			case safe:
			break;
		}
		//察觉到空置状态的变化
		ControlStatus_Detect();
		Self_Lock_Auto();
	  VectorWheel_SetAngle();
		VectorWheel_SetSpeed();
		osDelay(4);
	}
}
void communication(void const * argument)
{
  for(;;)
  {
		Vision_Basket_Decode();
		GamePad_Data_Cla();
	  Send_PositionToR1();		
		osDelay(20);
	}
}
void location(void const * argument)
	
{
  for(;;)
  {
	  YIS506_Decode();
		//陀螺仪原始数据计算 
		Encoder_XY_VX_VY_Cal(2);
		//获取陀螺仪加速度
		Gyro_AX_AY_Cal();
		//编码器速度计与陀螺仪及速度计的融合
		Enc_VXVY_Fuse_With_Gyro_AXAY(2);
		//雷达与编码器的重定位融合
		//Kalman_Test();
		//为车车选择坐标系
    Location_Type_Choose();
		//插帧得到篮筐和当前坐标的相关信息
		BasketPositionCal_AccordingVision(2);
		//码盘线性插帧
		LadarPosInterpolation(2);
		//DT35解算
		osDelay(2);
  }
}
void Detect(void const * argument)
{
  for(;;)
  {
		LossConnect_Check();
		Can_Detect();
    osDelay(400);
  }
}
////////////////////////////////////////////////////////////璇老师的进程//////////////////////////////
void HTMotorControl(void const * argument)
{
	for(;;)
	{
		Single_Control();
		HighTorque_SendPosParam_f(&hfdcan1, 6);
		osDelay(4);
	}
}

void ParamsChange(void const * argument)
{
	for(;;)
	{
		Loop_Judgement();
		Overall_Control();
		osDelay(10);
	}
}
////////////////////////////////////我是可爱小猫酱，喵喵喵~~~//////////////////////////////




























