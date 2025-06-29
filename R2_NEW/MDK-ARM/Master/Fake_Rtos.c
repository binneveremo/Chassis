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
char in;
float r,g,b;
int color;
void communication(void const * argument)
{
  for(;;)
  {
		GamePad_Data_Cla();
	  Send_MessageToR1();
		RGB_Show();
		osDelay(10);
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




























