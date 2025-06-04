#ifndef __BASKET_H
#define __BASKET_H

#include "Television.h"
#include "Kalman.h"
#include "Global.h"
#include "stdbool.h"
#include "math.h"
extern struct Point opposite_basket_point;
extern struct Point self_basket_point;
struct Basket_Lock_t{
	struct
	{
		float ladar_offsetrad;
		float basketdis;
		float anglebetween_ladarandpole;
		float limitzoneanglimit;
		float siteinterp_gain;
		float angleinterp_gain;
		int Time_Threshold;
	} parameter;
	struct
	{
		float ladar2basketx;
		float ladar2baskety;
		float ladar2basketdis;
		float ladar2basketangle;
	} position;
	float protectselfbasket_angle;
};
struct BasketPosition_Lock_t{
	struct {
		struct Point global;
		struct Point partial;
	}now;
	struct {
		struct Point global;
		struct Point partial;
	}target;
	struct
	{
		float p;
		float d;
		float i;
		float fade_start;
		float fade_end;
		float istart;
		float iend;
		float ilimit;
		float outlimit;
	} param;
	struct
	{
		float gain;
		float error;
		float itotal_x;
		float itotal_y;
		float outx;
		float outy;
	} process;
	struct
	{
		char lock_flag;
	} flagof;
};

struct BasketAngle_Lock_t{
	struct
	{
		float p;
		float i;
		float d;
		float fade_start;
		float fade_end;
		float istart;
		float iend;
		float ilimit;
		float outlimit;
		float accel_gain;
		float velocity_gain;
		float predict_step;
	} param;
	struct
	{
		float gain;
		float error;
		float itotal;
	} progress;
	struct
	{
		char stable_flag;
	} flagof;
};

extern struct Basket_Lock_t basketlock;
extern struct BasketAngle_Lock_t basketanglelock;
extern struct BasketPosition_Lock_t basketpositionlock;

void BasketPoint_Init(void);
void BasketPositionCal_AccordingVision(float dt);
void BsaketPoint_SelfLockAuto(void);
float BasketAngleLock(void);
void BasketPositionLock(void);

#endif
