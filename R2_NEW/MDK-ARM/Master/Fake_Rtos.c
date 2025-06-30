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

#include "Control.hpp"
#include "HighTorque.h"
#include "CPU_Load.h"
#include "Interact.h"
#include "Basket.h"



void motor_control(void const * argument)
{
   for(;;)
  {
		ChassisLock_Clear();
		switch(chassis.Control_Status){
			case GamePad_Control:
				GamePad_Velocity_Control();
			break;
			case Auto_Control:
				Auto_Flow();
			break;
			case Debug_Control:				
			break;  
		}
	  VectorWheel_SetAngle();
		VectorWheel_SetSpeed();
		VectorWheel_LockCheck();
		osDelay(4);
	}
}
void communication(void const * argument)
{
  for(;;)
  {
//		Send_Put_Data(0,site.now.x);
//		Send_Put_Data(1,site.car.xfilter.position);
//		Send_Put_Data(2,site.car.vx_enc);
//		Send_Put_Data(3,site.car.xfilter.velocity);
//		Send_Float_Data(4);
		
		GamePad_Data_Cla();
	  Send_MessageToR1();
		RGB_Show();
		osDelay(15);
	}
}
void location(void const * argument)
{
  for(;;)
  {
		//加速度计算
	  Gyro_AX_AY_Cal();
	  //陀螺仪原始数据计算
	  Encoder_XY_VX_VY_Cal(2);
		//雷达坐标计算
		Vision_Filed_Basket_XY_Cal(2);
		//x轴数据融合滤波
		KalmanX_Update(site.now.x,site.field.vx_enc,site.field.ax_gyro,&site.field.xfilter);
		//y轴数据滤波
		KalmanY_Update(site.now.y,site.field.vy_enc,site.field.ay_gyro,&site.field.yfilter);
	  //选择你的定位英雄
	  Location_Type_Choose();
		//隔一行知识为了好看
	  osDelay(2);
  }
}
void Detect(void const * argument)
{
  for(;;)
  {
		LossConnect_Check();
		Can_Detect();
    osDelay(100);
  }
}
////////////////////////////////////////////////////////////璇老师的进程//////////////////////////////
void HTMotorControl(void const * argument)
{
	for(;;)
	{
		HT_Test();
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




























