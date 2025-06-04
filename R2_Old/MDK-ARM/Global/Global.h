#ifndef __GLOBAL_H
#define __GLOBAL_H


#include "stdbool.h"
#include "string.h"
#include "math.h"

//#define Carbon_Car 

#ifdef Carbon_Car


#else 
#define CENTER_OFFSETX  400
#define CENTER_OFFSETY -375


#endif


#define NONE 0
#define PI 3.1415926f

#define ang2rad(ang) (ang*PI/180.0f) 
#define rad2ang(rad) (rad*180.0f/PI) 
#define NormalizeAng_Single(ang)  ((ang > 180)?ang - 360:((ang < -180)?ang + 360:ang))
#define Limit(x,y,z) ((x < y)?y:((x > z)?z:x))
#define Point_Distance(a,b) (hypot(a.x - b.x,a.y - b.y))
#define Point_InDistance(a,b,dis) (hypot(a.x - b.x,a.y - b.y) < dis)?1:0)
#define Clear(x) (memset(&x,(unsigned char)NULL,sizeof(x)))
#define Normalize_Liner(a,b,c)  ((Limit(fabs(c),a,b)-a)/(b -a))
#define Normalize_Pow(a,b,c,p)  pow(((Limit(fabs(c),a,b)-a)/(b -a)),p)

struct Point {
	float x;
	float y;
	float r;
};
typedef union __attribute__((packed))
{
	float float_data[8];
	unsigned int uint32_data[8];
	unsigned char uint8_data[32];
} uint8_uint32_float_union;

float char2float(unsigned char * data);



#endif
