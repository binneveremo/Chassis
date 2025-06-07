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
			case GamePad_Control:
				GamePad_Velocity_Control();
			break;
			case Auto_Control:
				Auto_Flow();
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
	  Send_MessageToR1();		
		osDelay(10);
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
		RGB_Show_Msg();
		LossConnect_Check();
		Can_Detect();
    osDelay(200);
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




























