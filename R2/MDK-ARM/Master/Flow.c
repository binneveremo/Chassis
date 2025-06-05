#include "Communication.h"
#include "Television.h"
#include "Fake_Rtos.h"
#include "Chassis.h"
#include "Can_Bsp.h"
#include "string.h"
#include "Basket.h"
#include "Flow.h"

struct Flow flow;
struct Point home_point = {                              
  .x = 600,
  .y = -600,
  .r = 0
};
/* @brief 扣篮流程 
0.跑点初始化
1.到点
/                */
struct dunk_t dunk;
void Dunk_Flow(void){
	switch(dunk.state){
		case init:
			BasketPoint_Init();
			dunk.state = goto_dunkpoint;
		case goto_dunkpoint:
			BasketPositionLock();
			Tell_Yao_Xuan("fold");
			dunk.state = (basketpositionlock.flagof.lock_flag == 1)?turnmotor_ready:dunk.state;
		break;
		case turnmotor_ready:
			if(TurnMotor_InTurnPosition() == true) dunk.state = wait_shoot;
		break;
		case wait_shoot:
			Tell_Yao_Xuan("catch");
#if Opposite_R1
		
			Chassis_Velocity_Out(0,0,Correct_Angle(send.R1_Exchange.pos.r + 180));
#endif
		dunk.state = (flow.flagof.R1_Shooted == true)?oppositebasket:dunk.state;
		break;
		case oppositebasket:
#if Opposite_R1
			Chassis_Velocity_Out(0,0,BasketAngleLock());
#else 
		Self_Lock_Out("BasketFlow");
#endif
			dunk.state = (fabs(basketanglelock.progress.error) < 1.5)?jump:dunk.state;
		break;
		case jump:
			Self_Lock_Out("BasketFlow");
			Tell_Yao_Xuan("predunk");                                                                                 
			dunk.state = end;
		break;
		case end:
			dunk.flagof.end = true;
		break;
	}
}
/// @brief 回家流程
struct back_t back;
void Back_Flow(void){
	Set_Target_Point(home_point);
	Position_With_Mark_PID_Run();
	if(Point_Distance(site.now,site.target) < 500) 
		back.flagof.end = true,Self_Lock_Out("HomePoint");
}
/// @brief 运球流程
struct dribble_t dribble = {.time.wait = 1300,.time.end = 2400,.parameter.dribble_front_velocity = 5500,.parameter.dribble_left_velocity = 1200,};
void Dribble_Flow(void){
	if(dribble.flagof.init == false){
		Tell_Yao_Xuan("dribble");
		dribble.time.begin = HAL_GetTick();
	}
	if(flow.flagof.stick_ball == true) {
		dribble.time.begin = HAL_GetTick();
		flow.flagof.stick_ball = false;
		dribble.flagof.init = true;
	}
	if(HAL_GetTick() - dribble.time.begin > dribble.time.wait)
		Chassis_Velocity_Out(dribble.parameter.dribble_left_velocity,dribble.parameter.dribble_front_velocity,0);
	else
		Self_Lock_Out("WaitDribble");
		Tell_Yao_Xuan(((HAL_GetTick() - dribble.time.begin < dribble.time.wait + 400)&&(HAL_GetTick() - dribble.time.begin > dribble.time.wait))?"fold":((HAL_GetTick() - dribble.time.begin > dribble.time.wait + 1000)?"moving":"catch"));
		dribble.flagof.end = (HAL_GetTick() - dribble.time.begin > dribble.time.end) ? true : false;
}
/////////技能挑战赛流程
struct skill_t skill;
void Skill_Flow(void){
	static char last_success_times;
	switch(skill.status){
		case begin:
			Set_Target_Point(skill.target.point[skill.success_time]);
			if((Point_Distance(site.now,site.target) < skill.param.shoot_advanced_dis) && (flow.flagof.R1_Shooted == false) && (skill.flagof.shoot_requested == false))
				skill.flagof.shoot_requested = true,Send_MessageToR1("request");
			if((Point_Distance(site.now,site.target) < skill.param.catch_advanced_dis) && (skill.flagof.net_catched == false))
				skill.flagof.net_catched = true,Tell_Yao_Xuan("catch");
			if(last_success_times != skill.success_time)
				last_success_times = skill.success_time,skill.status = clear;
		break;
		case clear:
			Tell_Yao_Xuan("defend");
			Clear(skill.flagof);
			skill.status = begin;
		break;
	}
}

/// @brief 返回手柄控制
void Back_GamePadControl(void){
	Clear(dribble.flagof);
	Clear(dunk.flagof);
	Clear(back.flagof);
	chassis.Control_Status = GamePad_Control;
}
/// @brief 自动流程
void Auto_Flow(void){
	switch(flow.type){
		case dribble_flow:
			Dribble_Flow();
		break;
		case dunk_flow:
			Dunk_Flow();
		break;
		case back_flow:
			Back_Flow();
		break;
	}
#define Rocker_Move ((hypot(GamePad_Data.rocker[0],GamePad_Data.rocker[1]) > 30) || (hypot(GamePad_Data.rocker[2],GamePad_Data.rocker[3]) > 30))
	if((dribble.flagof.end == true) || (dunk.flagof.end == true) || (back.flagof.end == true) || (Rocker_Move == true))
		Back_GamePadControl();
#undef Rocker_Move
}
void ControlStatus_Detect(void){

}

