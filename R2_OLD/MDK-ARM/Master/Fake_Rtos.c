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
#if MPC
#include "Second_Order.hpp"
#endif
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
			case Debug_Control:
				
			break;
		}
	  VectorWheel_SetAngle();
		VectorWheel_SetSpeed();
		osDelay(4);
	}
}
void communication(void const * argument)
{
  for(;;)
  {
		Send_Put_Data(0,site.filter[1]);
		Send_Put_Data(1,site.field.vx_enc);
		Send_Put_Data(2,site.filter[2]);
		Send_Put_Data(3,site.field.ax_gyro);
		Send_Float_Data(4);
		Vision_Basket_Decode();
		GamePad_Data_Cla();
	  //Send_MessageToR1();
		osDelay(25);

	}
}
void location(void const * argument)
{
  for(;;)
  {
	  Gyro_AX_AY_Cal();
	  // 陀螺仪原始数据计算
	  Encoder_XY_VX_VY_Cal(2);
	  // 获取陀螺仪加速度
	  Location_Type_Choose();
	  // 插帧得到篮筐和当前坐标的相关信息
	  BasketPositionCal_AccordingVision(2);
	  // 码盘线性插帧
	  LadarPosInterpolation(2);
	  // DT35解算
	  #if MPC
	  Kalman3D_Update(site.now.x, site.field.vx_enc, site.field.ax_gyro);
	  #endif 
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




























