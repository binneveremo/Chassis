#ifndef __FLOW_H
#define __FLOW_H

#include "Chassis.h"

#include "Basket.h"
#include "NetWork.h"

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
	}type;
};
extern struct Flow flow;










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
	}time;
	struct {
		char prepared;
		char drrbbled;
		char init;
	}flagof;
};
extern struct dribble_t dribble;

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

struct attack_t {
	enum{
		attack_init,
		attack_runp,
		attack_jump,
	}status;
	struct {
		
	
	
	}flagof;
};














struct back_t{
	struct {
		char end;
	}flagof;
};
extern struct back_t back;

struct skill_t {
	enum{
		begin,
		clear,
	}status;
	struct {
		int shoot_advanced_dis[7];
		int catch_advanced_dis[7];
		int catch_delay_time[7];
		float lock_dis;
		float lock_angle;
		struct Spot_t spot[7];
		float min_velocity_left[7];
		float max_velocity_left[7];
	}param;
	struct {
		struct Point point[7];
	}target;
	struct{
		char net_catched;
		char end;
	}flagof;
	unsigned char success_time;
};
extern struct skill_t skill;

void ControlStatus_Detect(void);
void Back_GamePadControl(void);
void Auto_Flow(void);
struct Point SkillFlow_R2PositionToR1(struct Point point);
void Receive_BallCheck(void);
#endif



