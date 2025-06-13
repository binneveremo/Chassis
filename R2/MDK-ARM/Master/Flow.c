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
  .x = 500,
  .y = -650,
  .r = 0
};
struct dunk_t dunk;
void Dunk_Flow(void){
	switch(dunk.state){
		case init:
			BasketPoint_Init(&dunk.flagof.init);
			dunk.state = goto_dunkpoint;
		case goto_dunkpoint:
			BasketPosition_Lock();
			Tell_Yao_Xuan("defend");
			if(chassis.lock.flag == 1)  dunk.state = turnmotor_ready;
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
			//dunk.state = (fabs(basketanglelock.progress.error) < 1.5)?jump:dunk.state;
		break;
		case jump:
			Self_Lock_Out("BasketFlow");
			Tell_Yao_Xuan("lift");                                                                                 
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
	PositionWithAngle_Lock(site.now,home_point,&spot_skill,&cr_skill);
	if(Point_Distance(site.now,site.target) < 500) 
		back.flagof.end = true,Self_Lock_Out("HomePoint");
}
/// @brief 运球流程
struct dribble_t dribble = {.time.xuan_stamp = 1700,.time.wait = 700,.time.end = 2000,.parameter.dribble_front_velocity = 3700,.parameter.dribble_left_velocity = 300,};
void Dribble_Flow(void){
	int now = HAL_GetTick();
	switch(dribble.status){
		case prepare:
			if(dribble.flagof.prepared == false){
				Tell_Yao_Xuan("dribble");
				interact.defend_status = (interact.defend_status == catch_ball)?predunk:interact.defend_status;
				interact.defend_status = ((interact.defend_status == initial) || (interact.defend_status == fold))?catch_ball:interact.defend_status;
				dribble.flagof.prepared = true;
			}
			if(flow.flagof.stick_ball == true){
				dribble.time.begin = now;
				dribble.status = dribble_begin;
			}
		break;
		case dribble_begin:
			if(now - dribble.time.begin > dribble.time.wait)
				Chassis_Velocity_Out(dribble.parameter.dribble_left_velocity,dribble.parameter.dribble_front_velocity,0);
			else
				Self_Lock_Out("WaitDribble");
			if(now - dribble.time.begin > dribble.time.xuan_stamp)
				Tell_Yao_Xuan("catch");
			else 
				Tell_Yao_Xuan("fold");
			if(now - dribble.time.begin > dribble.time.end)
				dribble.flagof.end = true;
		break;
	}
}
/////////技能挑战赛流程
struct skill_t skill = {
#if false
	.target.point[0] = {.x = 390,  .y = -386,  .r = 0},
	.target.point[1] = {.x = -451, .y = 393,.r = 0},
	.target.point[2] = {.x = -1380,.y = 322,.r = 0},
	.target.point[3] = {.x = -1524,.y = -1613,.r = 0},
	.target.point[4] = {.x = -1470,.y = -3723,.r = 0},
	.target.point[5] = {.x = -546, .y = -3735,.r = 0},
	.target.point[6] = {.x = 430, .y = -2860,.r = 0},
	.target.point[0] = {.x = 390,  .y = -386,  .r = -62},
#elif false
	.target.point[0] = {.x = 0, .y =0,.r = -70},
	.target.point[1] = {.x = 1239, .y = -997,.r = -56},
	.target.point[2] = {.x = 2213,.y = -913,.r = -52},
	.target.point[3] = {.x = 2040,.y = 1081,.r = -40},
	.target.point[4] = {.x = 1842,.y = 3129,.r = -18},
	.target.point[5] = {.x = 788, .y = 3042,.r = -25},
	.target.point[6] = {.x = 134, .y = 2106,.r = -45},
#elif true
	.target.point[0] = {.x = 2760,.y = -5284,.r = -61},
	.target.point[1] = {.x = 3423,.y = -5874,.r = -58},
	.target.point[2] = {.x = 4466,.y = -6059,.r = -49},
	.target.point[3] = {.x = 4546,.y = -4041,.r = -42},
	.target.point[4] = {.x = 4516,.y = -1902,.r = -28},
	.target.point[5] = {.x = 3608,.y = -1948,.r = -23},
	.target.point[6] = {.x = 2795,.y = -2693,.r = -42},
#endif

	.param.catch_advanced_dis[0] = 200,
	.param.catch_advanced_dis[1] = 200,
	.param.catch_advanced_dis[2] = 200,
	.param.catch_advanced_dis[3] = 200,
	.param.catch_advanced_dis[4] = 200,
	.param.catch_advanced_dis[5] = 200,
	.param.catch_advanced_dis[6] = 200,

	.param.shoot_advanced_dis[0] = 50,
	.param.shoot_advanced_dis[1] = 50,
	.param.shoot_advanced_dis[2] = 50,
	.param.shoot_advanced_dis[3] = 50,
	.param.shoot_advanced_dis[4] = 50,
	.param.shoot_advanced_dis[5] = 50,
	.param.shoot_advanced_dis[6] = 50,

	.param.lock_dis = 80,
	.param.lock_angle = 8,
};
void Skill_Flow(void){
	static char last_success_times;
	char index = skill.success_time % 7;      
	switch(skill.status){
		case begin:
			PositionWithAngle_Lock(site.now,Merge_Point(skill.target.point[index],send.R1_Exchange.pos),&spot_skill,&cr_skill);
			if((Point_Distance(site.now,skill.target.point[index]) < skill.param.catch_advanced_dis[index]) && (skill.flagof.net_catched == false))
				skill.flagof.net_catched = true,Tell_Yao_Xuan("defend");
			if((Point_Distance(site.now,skill.target.point[index]) < skill.param.lock_dis))
				Self_Lock_Out("SkillFlow"),send.R1_Exchange.request_flag = true;
			else 
				send.R1_Exchange.request_flag = true;
			if(last_success_times != skill.success_time)
				last_success_times = skill.success_time,skill.status = clear;
		break;
		case clear:
			if(skill.success_time == 8) {
				skill.flagof.end = true;
				return;
			}
			Tell_Yao_Xuan("defend");
			//Clear(spot.process);
			Clear(skill.flagof);
			skill.status = begin;
		break;
	}
}

/// @brief 返回手柄控制
void Back_GamePadControl(void){
	Zero(skill.status);
	Clear(skill.flagof);
	
	Zero(dunk.state);
	Clear(dunk.flagof);
	
	Zero(dribble.status);
	Clear(dribble.flagof);
	
	Clear(back.flagof);
	Clear(flow.flagof);
	
	Zero(skill.success_time);
	Clear(send.R1_Exchange.request_flag);
	//清除自动流程的枚举
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
		case skill_flow:
			Skill_Flow();
		break;
	}
	if((dribble.flagof.end == true) || (dunk.flagof.end == true) || (back.flagof.end == true) || (Rocker_Move() == true) || (skill.flagof.end == true))
		Back_GamePadControl();
#undef Rocker_Move
}
void ControlStatus_Detect(void){

}


