#ifndef __LOCATION_H
#define __LOCATION_H
#include "Global.h"
#define ladar_angle 0
#define gyro_angle 1

struct Site{
	struct Point target;
	struct Point now;
	struct {
		float x_enc;
		float y_enc;
		float ax_gyro;
		float ay_gyro;
		float vx_enc;
		float vy_enc;
		float vx_fuse;
		float vy_fuse;
		struct status_node_t xfilter;
		struct status_node_t yfilter;
	}field;
	struct {
		float r;
		float omiga;
	}gyro;
	struct {
		float ax_gyro;
		float ay_gyro;
		float vx_enc;
		float vy_enc;
		float accel_totalgyro;
		float velocity_totalgyro;
		float velocity_totalenc;
		struct status_node_t xfilter;
		struct status_node_t yfilter;
	}car;
};
#define theta ang2rad(site.now.r)
extern struct Site site;
#define Car2Field(fieldx,fieldy,carx,cary) {				 \
	*(fieldx) = *(carx) * cos(theta) - *(cary) * sin(theta); \
	*(fieldy) = *(carx) * sin(theta) + *(cary) * cos(theta); \
}                                                     
#define Field2Car(carx,cary,fieldx,fieldy) {				 \
	*(carx) = *(fieldx) * cos(theta) + *(fieldy) * sin(theta);   \
	*(cary) =-*(fieldx) * sin(theta) + *(fieldy) * cos(theta);   \
}    


void WheelVelocity_Fuse_WithOdometer(void);
void Location_Type_Choose(void);
void Enc_VXVY_Fuse_With_Gyro_AXAY(float dt);
//雷达与马盘数据融合

#endif