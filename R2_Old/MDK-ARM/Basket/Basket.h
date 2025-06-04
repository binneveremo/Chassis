#ifndef __BASKET_H
#define __BASKET_H

#include "Television.h"
#include "Kalman.h"
#include "Global.h"
#include "stdbool.h"
#include "math.h"

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
		struct Point now_interp_vfield;
		struct Point basket_target_vfield;
		struct Point backwardladar_field;
		struct Point backwardcar_field;
	} position;
	struct
	{
		bool danger;
		char nearest_point_init;
	} flagof;
	int begin;
};
struct BasketPosition_Lock_t{
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

struct Partial_Site{




};

void BasketPositionCal_AccordingVision(float dt);
void BsaketPoint_SelfLockAuto(void);
float BasketAngleLock(void);
void Vfield_Convert_Field(struct Point *field, struct Point vfield);
void Vfield_Convert_Field_Test(void);
void BasketPositionLock_ParInit(void);
void BasketPositionLock(void);
void BasketAngleLock_ParInit(void);
#define BasketPositionLock_ParInit(){         \
		basketpositionlock.param.p = 3.5;          \
		basketpositionlock.param.i = 0.5;          \
		basketpositionlock.param.istart = -1;      \
		basketpositionlock.param.fade_start = 580; \
		basketpositionlock.param.fade_end = 170;   \
		basketpositionlock.param.iend = 500;       \
		basketpositionlock.param.ilimit = 700;     \
		basketpositionlock.param.outlimit = 6800;  \
	}

#endif
