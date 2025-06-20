#pragma once
#include "Global.h"
#ifdef __cplusplus
extern "C" {
#endif
#if MPC 

void Kalman3D_Init(void);
void Kalman3D_Update(float position, float velocity, float acceleration);
float MPC_Test(float position, float velocity, float accel);

#endif
#ifdef __cplusplus
}
#endif
