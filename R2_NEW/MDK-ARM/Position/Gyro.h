#ifndef __GYRO_H
#define __GYRO_H
#include "Television.h"
#include "usart.h"
#include "tim.h"

#define accel_ratio 0.01f//分辨率（具体看手册）
#define omega_ratio 0.0078125f
#define euler_radio 0.0078125f

#define _accel_offset 320.0f//偏移量（具体看手册）
#define _omega_offset 4000.0f
#define _euler_offset 250.0f

#define ExtID_accel 0x0CF02D59 //扩展帧ID 
#define ExtID_omega 0x0CF02A59
#define ExtID_yaw 0x0CF02959 

#define YIS512_FILTER_SIZE 5


#define Gyro_AngleReset(target) (yis506.euler.yaw_offset = yis506.euler.yaw - target)

struct YIS506{
	char reset_flag;
    int header;
	bool online_flag;
	unsigned char * data;
  struct{
		float accel;
		float accel_offset;
		float omega;
	}X,Y,Z;
  struct{
		float yaw_offset;
		float pitch;
		float yaw;
		float roll;
	}euler;
};


void Gyro_AX_AY_Cal(void);
extern struct YIS506 yis506;
void Get_Gyro_Data(int header,unsigned char * data);
void YIS506_Decode(void);
void YIS506_Fuse_With_Ladar_Angle(int time);
void Gyro_Reset(void);

void Identify_Accel_Abrupt(void);
#endif

















