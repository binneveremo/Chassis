#ifndef __GUARD_H
#define __GUARD_H
#include "Location.h"
#include "Chassis.h"

struct Guard_Trace
{
		float p;
    float outx;
    float outy;
    float outlimit;
    float accel_step;
    float break_angle;
    float brake_distance;
};

void Guard_Trace_Par_Init(void);
void Guard_Trace_Run(void);

#endif






