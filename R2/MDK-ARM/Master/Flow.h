#ifndef __FLOW_H
#define __FLOW_H

#include "Chassis.h"
#include "Send.h"

struct Flow {
	struct {
		char stick_ball;
		char R1_Shooted;
	}flagof;
	enum
	{
		dribble_flow = 1,
		dunk_flow,
		back_flow,
		skill_flow,
	}type;
};
extern struct Flow flow;
struct dribble_t {
	struct {
		int dribble_front_velocity;
		int dribble_left_velocity;
	}parameter;
	struct {
		int begin;
		int now;
		int end;
		int wait;
	}time;
	struct {
		char end;
		char init;
	}flagof;
};
extern struct dribble_t dribble;
struct dunk_t {
	struct {
		char init;
		char end;
	}flagof;
	enum{
		init,
		goto_dunkpoint,
		turnmotor_ready,
		wait_shoot,
		oppositebasket,
		jump,
		end
	}state;
};
extern struct dunk_t dunk;
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
		int lock_dis;
	}param;
	struct {
		struct Point point[7];
	}target;
	struct{
		char net_catched;
		char end;
	}flagof;
	char success_time;
};
extern struct skill_t skill;

void ControlStatus_Detect(void);
void Back_GamePadControl(void);
void Auto_Flow(void);
struct Point SkillFlow_R2PositionToR1(struct Point point);

#endif



