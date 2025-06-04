#ifndef __KALMAN_H__
#define __KALMAN_H__

#include "Location.h"
#include "Global.h"
#include "math.h"


struct EKF{
	float r;
	float q;
	float a_hat_prior;
	float p_prior;
	float y;
	float k;
	float a_hat;
	float p;
};
float EKF_Filter(struct EKF * ekf, float input ,float gain);
void Position_Velocity_Accel_SecondOrderKalmanCal(void);
void Kalman_Test(void);
void SecondOrder_KalmanSystemInit(void);
#endif
