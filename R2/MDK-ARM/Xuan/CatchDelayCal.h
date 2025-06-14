#ifndef CATCHDELAYCAL_H
#define CATCHDELAYCAL_H

#include "main.h"
#include <math.h>
#include <float.h>
#include "Interact.h"
#include "stdbool.h"
#include "HighTorque.h"
#include "Global.h"
#include "Flow.h"

#ifndef M_PI
#define M_PI  3.1415926
#endif


#define GRAVITY_ACCEL     9.7913
#define AIR_DENSITY       1.09f //kg/m^3

#define BALL_RADIUS       0.121f
#define BALL_MASS         0.620f
#define DRAG_COEFFICIENT  0.47f

#define BALL_CROSS_SECTIONAL_AREA (M_PI * BALL_RADIUS * BALL_RADIUS)
#define K_DRAG (0.5f * AIR_DENSITY *BALL_CROSS_SECTIONAL_AREA * DRAG_COEFFICIENT)

#define SIM_TIME_STEP 0.001f
#define MAX_SIM_DURATION 10.0f

#define HORIZONTAL_TOLERANCE 0.10f
#define VERTICAL_TOLERANCE   0.05f
#define SPEED_THRESHOLD      0.01f //judge whether the ball is moving

#define SHOOT_ANGLE          65.0f
#define SHOOT_HEIGHT         1.18f
#define NET_HEIGHT           1.90755f
#define NET_TO_CENTER        0.28974f


typedef struct
{
    float x; //horizontal position
    float y; //vertical position
    float vx; //horizontal velocity
    float vy; //vertical velocity
    float time; //time passed
}BallState_t;

typedef struct
{
    float final_x;
    float time_s;
}SimResult_t;

uint32_t CatchTimeCal();


#endif 