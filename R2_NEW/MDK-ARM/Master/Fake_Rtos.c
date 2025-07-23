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
#include "NetWork.h"
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
		ControlStatus_Detect();
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

		Send_Put_Data(0,vision.visual.ladar_visual.x);
		Send_Put_Data(1,vision.visual.ladar_visual.y);
		Send_Put_Data(2,ang2rad(site.now.r));
		Send_Float_Data(3);

		//手柄数据解析
		GamePad_Data_Cla();
		//向R1发送的相关函数
		Send_MessageToR1();
		//RGB输出
		RGB_Show();
		//发送篮筐距离
		Send_BasketDis();
		//发送底盘是否移动
		Send_VisionMove();
		//单纯为了好看的注释
		osDelay(10);
	}
}
void location(void const * argument)
{
  for(;;)
  {
		//根据舵轮的数据回传进行计算
		CarStatusExcept_AccordVectorWheel();
		//加速度计算
	  Gyro_AX_AY_Cal();
		//对电调回传速度以及码盘速度进行融合
		WheelVelocity_Fuse_WithOdometer();
		//识别加速度突变
		Receive_BallCheck();
	  //陀螺仪原始数据计算
	  Encoder_XY_VX_VY_Cal(2);
		//雷达坐标计算
		Vision_Filed_Basket_XY_Cal(2);
		//x轴数据融合滤波
		KalmanX_Update(vision.field.carcenter_fieldinterp.x,site.field.vx_fuse,site.field.ax_gyro,&site.field.xfilter);
		//y轴数据滤波
		KalmanY_Update(vision.field.carcenter_fieldinterp.y,site.field.vy_fuse,site.field.ay_gyro,&site.field.yfilter);
	  //将场地坐标系的三个状态计算到车体坐标系下
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
    osDelay(150);
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




























