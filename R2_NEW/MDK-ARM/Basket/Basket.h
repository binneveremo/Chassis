#ifndef __BASKET_H
#define __BASKET_H

#include "Television.h"
#include "Global.h"
#include "stdbool.h"
#include "math.h"
extern struct Point opposite_basket_point;
extern struct Point self_basket_point;
struct Basket_Lock_t{
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
		float basketdis;
		float anglebetween_ladarandpole;
		float limitzoneanglimit;
	} parameter;
	struct
	{
		float ladar2basketx;
		float ladar2baskety;
		float ladar2basketdis;
		float ladar2basketangle;
		float percent;
	} position;
	struct {
		char dunk_position;
	}flagof;
	float protectselfbasket_angle;
};


extern struct Basket_Lock_t basketlock;
void BasketPoint_Init(void);
void BasketPositionCal_AccordingVision(float dt);
void BasketPosition_Lock(void);

#endif
