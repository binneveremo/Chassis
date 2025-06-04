#include "Communication.h"
#include "Television.h"
#include "Fake_Rtos.h"
#include "Chassis.h"
#include "Can_Bsp.h"
#include "string.h"
#include "Basket.h"
#include "Flow.h"

/*///////////////////////////////////
1.找到最近的篮筐半径
2.设置死区 只有到达死去才会自锁
3.动态设置所篮筐角度死区 距离越近 并且速度越慢 角度死区越小
*/////////////////////////////////////////

struct Flow flow;
struct Point home_point = {                              
  .x = 600,
  .y = -600,
  .r = 0
};
struct Point dunk_point = {
  .x = 12975,
	.y = -4000,
	.r = 0.4
};


void Basket_Flow(void){
	switch(flow.state){
		case basketpos_lock:
			Tell_Yao_Xuan("fold");
			BasketPositionLock();
			flow.state = (basketpositionlock.flagof.lock_flag == 1)?turn_ready:flow.state;
		break;
		case turn_ready:
			if(TurnMotor_InTurnPosition() == true) flow.state = receiving;
		break;
		case receiving:
			Tell_Yao_Xuan("catch");
			//Chassis_Velocity_Out(0,0,Correct_Angle(send.R1_Exchange.pos.r + 180));
			flow.state = (flow.flag_of.received == 1)?oppositebasket:flow.state;
		break;
		case oppositebasket:
			Chassis_Velocity_Out(0,0,BasketAngleLock());
			flow.state = (fabs(basketanglelock.progress.error) < 1.5)?dunk:flow.state;
		break;
		case dunk:
			Self_Lock_Out("BasketFlow");
			Tell_Yao_Xuan("predunk");
			//Tell_Yao_Xuan("lift");
			flow.state = end;
		break;
		case end:
			flow.flag_of.end = true;
		break;
	}
}
void Auto_Basket_Lock(void){
	
}
void Flow_Reset(void){
	Clear(flow.flag_of);
	Clear(basketlock.flagof);
	Flow_State_Reset();
}
void Run_Point_Test(void){
	Set_Target_Point(dunk_point);
	Position_With_Mark_PID_Run();
}

////////////////////////////////////编码器偏置测试//////////////////////////////////////////////
void Car_State_Decode(int id,unsigned char * data){
	if(id == 0xB1)
		flow.flag_of.received = 1;
	if(id == 0xB2)
		flow.flag_of.stick_ball = true;
}
void Back(void){
	Set_Target_Point(home_point);
	Position_With_Mark_PID_Run();
	if(Point_Distance(site.now,site.target) < 500) Self_Lock_Out("HomePoint");
	if(Point_Distance(site.now,site.target) < 500) chassis.Control_Status = gamepad_standard;
}

void ControlStatus_Detect(void){
	static char last;
	flow.flag_of.dribble = ((last != dribble) && (chassis.Control_Status == dribble))?1:flow.flag_of.dribble;
	flow.flag_of.dribble = ((last == dribble) && (chassis.Control_Status != dribble))?0:flow.flag_of.dribble;
	//切换到比的状态流程自动复位
	if(((last != basket_lock) && (chassis.Control_Status == basket_lock)))	Flow_Reset();
	
	last = chassis.Control_Status;
}

int dribble_wait_time = 2100;
int dribble_front_velocity = 6800;
int dribble_left_velocity = 800;
//void Dribbble_Flow(void){
//	static int begin;
//	if(flow.flag_of.dribble == 1){
//		Tell_Yao_Xuan("predunk");
//		Tell_Yao_Xuan("dribble");
//		begin = HAL_GetTick();
//		flow.flag_of.dribble = 0;	}
//		
//	if(HAL_GetTick() - begin > 2500){
//		Tell_Yao_Xuan("fold");
//		Chassis_Velocity_Out(500,6800,0);}
//	else 
//		Self_Lock_Out("WaitDribble");
//	if(HAL_GetTick() - begin > 2800){
//		Tell_Yao_Xuan("predunk");}
//}
void Dribbble_Flow(void){
	static int begin;
	static int real_start;
	if(flow.flag_of.dribble == 1){
		Tell_Yao_Xuan("predunk");
		Tell_Yao_Xuan("dribble");
		begin = HAL_GetTick();
		flow.flag_of.dribble = 0;	}
	
		if(flow.flag_of.stick_ball == true)
		{
			real_start = HAL_GetTick();
			flow.flag_of.stick_ball = false;
		}
	
		if(real_start - begin > dribble_wait_time){
			Tell_Yao_Xuan("fold");
			Chassis_Velocity_Out(dribble_left_velocity, dribble_front_velocity, 0);
		}
		else
			Chassis_Velocity_Out(0,100,0);
		if(real_start - begin > 2300){
			Tell_Yao_Xuan("predunk");
		}
}






















































		//Send_Velocity_Vision();
		//Send_Put_Data(0, vision.pos.ladar_field.x);
		//Send_Put_Data(1, vision.pos.ladar_field.y);
		//Send_Put_Data(2, ang2rad(vision.pos.ladar_field.r));
		//Send_Put_Data(3, 3);
		//Send_Float_Data(4);
		//Send_Put_Data(0, filtered_pos_2nd);
		//Send_Put_Data(1, filtered_trq_2nd);
		//Send_Put_Data(2, Compensation_trq);
		//Send_Put_Data(3, HighTorque[0].fdbk.spd);
		//Send_Float_Data(4);

