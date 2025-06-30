#pragma once
#include "Global.h"
CPP_BEGIN


void Kalman_Init(void);
void KalmanX_Update(float position, float velocity, float acceleration,struct status_node_t * statusx);
void KalmanY_Update(float position, float velocity, float acceleration,struct status_node_t * statusy);


void MPC_Init(void);
void MPC_Calculate(struct Point last,struct Point next,float now_rad,float velocity_target,float * left,float * front);


CPP_END