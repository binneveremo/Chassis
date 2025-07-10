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

bool stable,threshold;

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
				GamePad_Velocity_Control(); 
				Receive_BallCheck();
			break;  
		}
		//MPC_Calculate(last,next,ang2rad(site.now.r),0.5,&front,&left);
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
//		Send_Put_Data(0,chassis.except_status.front_velocity / 1000 * 6);
//		Send_Put_Data(1,site.field.xfilter.velocity);
//		Send_Put_Data(2,Easy_Filter(chassis.except_status.front_accel));
//		Send_Put_Data(3,site.field.xfilter.accel);
//		Send_Put_Data(4,chassis.except_status.rotate_velocity);
//		Send_Put_Data(5,site.gyro.omiga);
    Send_Put_Data(0,chassis.expect_status.front_accel);
		Send_Put_Data(1,site.field.xfilter.accel);
		Send_Float_Data(2);
		
		GamePad_Data_Cla();
		Send_MessageToR1();
		RGB_Show();
		Send_BasketDis();
		osDelay(10);
	}
}
void location(void const * argument)
{
  for(;;)
  {
		CarStatusExcept_AccordVectorWheel();
		
		stable = Variance_Check(chassis.expect_status.front_velocity,10,0.2,"check");
		//加速度计算
	  Gyro_AX_AY_Cal();
		//识别加速度突变
		Receive_BallCheck();
	  //陀螺仪原始数据计算
	  Encoder_XY_VX_VY_Cal(2);
		//雷达坐标计算
		Vision_Filed_Basket_XY_Cal(2);
		//x轴数据融合滤波
		KalmanX_Update(vision.field.carcenter_fieldinterp.x,site.field.vx_enc,site.field.ax_gyro,&site.field.xfilter);
		//y轴数据滤波
		KalmanY_Update(vision.field.carcenter_fieldinterp.y,site.field.vy_enc,site.field.ay_gyro,&site.field.yfilter);
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




























