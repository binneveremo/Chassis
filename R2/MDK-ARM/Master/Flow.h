#ifndef __FLOW_H
#define __FLOW_H

#include "Chassis.h"
#include "Send.h"

struct Flow {
	struct {
		char end;
		char received;
		char jumped;
		char defend_send;
		char jump_send;
		char back;
		char dribble;
		char stick_ball;
	}flagof;
	enum
	{
		dribble_flow,
		dunk_flow,
		back_flow,
	}
	type;
};
extern struct Flow flow;
struct dribble_t {
	struct {
		int dribble_front_velocity;
		int dribble_left_velocity;
	}parameter;
	struct {
		int begin;
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
		get_basket_point,
		turn_ready,
		receiving,
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

void ControlStatus_Detect(void);
void Back_GamePadControl(void);
void Auto_Flow(void);


#endif



