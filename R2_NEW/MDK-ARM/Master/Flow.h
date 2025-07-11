#ifndef __FLOW_H
#define __FLOW_H

#include "Chassis.h"

#include "Basket.h"
#include "NetWork.h"




////////////////////////////////////////////////////整体流程////////////////////////////////////////////////
struct Flow {
	struct {
		char end;
		char pole_top;
		char stick_ball;
		char R1_Shooted;
		char receive_ball_bygyro;
	}flagof;
	enum
	{
		dribble_flow = 1,
		dunk_flow,
		back_flow,
		skill_flow,
		attack_flow,
		trap_flow,
	}type;
};
extern struct Flow flow;









////////////////////////////////////////////////////运球流程////////////////////////////////////////////////
struct dribble_t {
	enum{
		prepare,
		dribble_begin,	
	}status;
	struct {
		int dribble_front_velocity;
		int dribble_left_velocity;
	}parameter;
	struct {
		int begin;
		int xuan_stamp;
		int wait;
		int end;
		int pole_uptime;
	}time;
	struct {
		char prepared;
		char drrbbled;
		char init;
	}flagof;
};
extern struct dribble_t dribble;












////////////////////////////////////////////////////扣篮流程////////////////////////////////////////////////
struct dunk_t {
	enum{
		goto_dunkpoint,
		jump,
		end
	}state;
	struct {
		char init;
		char confirm;
		char net_ok;
	}flagof;
};
extern struct dunk_t dunk;





























////////////////////////////////////////////////////突袭流程////////////////////////////////////////////////
struct attack_t {
	enum{
		attack_init,
		attack_runp,
		attack_jump,
	}status;
	struct {
		
	
	
	}flagof;
};



////////////////////////////////////////////////////回家流程////////////////////////////////////////////////
struct back_t{
	struct {
		char end;
	}flagof;
};
extern struct back_t back;


















////////////////////////////////////////////////////静态技能赛流程////////////////////////////////////////////////
struct skill_t {
	enum{
		begin,
		clear,
	}status;
	struct {
		int shoot_advanced_dis[8];
		int catch_advanced_dis[8];
		int catch_delay_time[8];
		float lock_dis;
		float lock_angle;
		struct Spot_t spot[8];
		float min_velocity_left[8];
		float max_velocity_left[8];
	}param;
	struct {
		struct Point point[8];
	}target;
	struct{
		char net_catched;
	}flagof;
	unsigned char success_time;
};
extern struct skill_t skill;







////////////////////////////////////////////////////////////////动态技能赛流程/////////////////////////////
/*
1.固定速度 固定方向前进 
2.速度稳定之后 预测时间
3.接球 放球
4.下一个点

*/



struct Trap_t {
	enum{
		trap_prepare,
		trap_move,
		trap_stable,
		trap_end,
	}status;
	struct{
		struct Point point[7];
		float expect_velocity[7];
	}param;
	struct {
		float dir_ang;
		struct Point cusp;
		
	}process;
	struct {	
		char net_catched;
		char velocity_stable;
		
		
		char end;
	}flagof;
	unsigned char success_time;
};



















void Trap_Flow_Test(struct Point end,float rpm,struct Point * expect);
bool Expect_Position(struct Point start,struct Point end,float dir_ang,float rpm,struct Point * expect);
float Variance_Check(float data,char num,float threshold,char * type);
void ControlStatus_Detect(void);
void Back_GamePadControl(void);
void Auto_Flow(void);
struct Point SkillFlow_R2PositionToR1(struct Point point);
void Receive_BallCheck(void);
#endif





