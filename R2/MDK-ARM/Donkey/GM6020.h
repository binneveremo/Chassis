#ifndef __GM6020_H
#define __GM6020_H
#include "Global.h"
struct Motor_Status{
	float v;
	float out;
	float angle;
};
struct Wheel{
	float vx;
	float vy;
	float dir;
	float angle;
	float rpm;
};
struct Velocity_PID{
	float p;
	float i;
	float d;
	float velocitypre;
	float itotal;
	float istart;
	float iend;
	float ilimit;
	float outlimit;
};
struct Angle_PID{
	float p;
	float i;
	float d;
	float anglepre;
	float anglemulti;
	float itotal;
	float istart;
	float iend;
	float ilimit;
	float outlimit;
	float death;
	int dir;
};
struct GM6020 {
	struct Motor_Status status[3];
	struct Velocity_PID velocity[3];
	struct Angle_PID angle[3];
};

void GM6020_Status_Cla(void);
void GM6020_Par_Init(void);
void GM6020_OutPut(void);
void GM6020_Velocity_PID(int name, float target);
void GM6020_Angle_PID(int name, float target);
float GM6020_S_Velocity(float target,int frequency);
#endif

