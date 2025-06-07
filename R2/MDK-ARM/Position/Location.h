#ifndef __LOCATION_H
#define __LOCATION_H
#include "Global.h"
#define ladar_angle 0
#define gyro_angle 1

struct Site{
	struct Point target;
	struct Point now;
	//编码器计算出来的单独的位置以及速度
	struct {
		float x_enc;
		float y_enc;
		float ax_gyro;
		float ay_gyro;
		float vx_enc;
		float vy_enc;
		float vx_gyro;
		float vy_gyro;
	}field;
	struct {
		float r;
		float omiga;
	}gyro;
	struct {
		float ax_gyro;
		float ay_gyro;
		float vx_gyro;
		float vy_gyro;
		float vx_enc;
		float vy_enc;
		float accel_totalgyro;
		float velocity_totalgyro;
		float velocity_totalenc;
	}car;
	struct {
		struct Point target;
		struct Point enc;
	}partial;
};
enum Location_Type{
	odometer_location,
	vision_location,
	dt35_location,
	mix_location
};

extern struct Site site;

void Location_Type_Choose(void);
void DT35_Fuse_With_Odometer(float dt);

void Enc_VXVY_Fuse_With_Gyro_AXAY(float dt);
//雷达与马盘数据融合
void Ladar_With_Odometer_Kalman(float dt);
void Ladar_With_Odometer_Relocation(float dt);
#endif