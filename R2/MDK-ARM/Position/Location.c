#include "Location.h"
#include "Encoder.h"
#include "Television.h"
#include <string.h>
#include "Global.h"
#include "Kalman.h"
#include "math.h"

struct Site site;
struct EKF ekf_x = {
	.q = 0.0002,
	.r = 1.8,
};
struct EKF ekf_y = {
	.q = 0.0002,
	.r = 1.8,
};
//
void Location_Type_Choose(void){
	site.now.x = vision.field.carcenter_fieldinterp.x;
	site.now.y = vision.field.carcenter_fieldinterp.y;
	site.now.r = site.gyro.r;
}
///////////////////////////////////////////////////////码盘与陀螺仪速度融合计算///////////////////////////////////////////////////////////////
//码盘的y轴是陀螺仪的-x轴
void Enc_VXVY_Fuse_With_Gyro_AXAY(float dt){
	site.car.vx_gyro = EKF_Filter(&ekf_x,site.car.vx_enc,site.car.ax_gyro*dt / 1000);
	site.car.vy_gyro = EKF_Filter(&ekf_y,site.car.vy_enc,site.car.ay_gyro*dt / 1000);
	site.car.velocity_totalgyro = hypot(site.car.vx_gyro,site.car.vy_gyro);
	site.field.vx_gyro  = site.car.vx_gyro * cos(ang2rad(site.now.r)) - site.car.vy_gyro * sin(ang2rad(site.now.r));
	site.field.vy_gyro  = site.car.vx_gyro * sin(ang2rad(site.now.r)) + site.car.vy_gyro * cos(ang2rad(site.now.r));
}
//////////////////////////////////////////////////////码盘与陀螺仪与雷达速度融合计算/////////////////////////////////////////////////////////////
struct EKF odo_ladar_ekf_x = {
	.q = 0.002,
	.r = 1.2,
};
struct EKF odo_ladar_ekf_y = {
	.q = 0.002,
	.r = 1.2,
};
//当前的思路：
/*
1.有雷达数据的时候直接把雷达数据输入滤波器；
2.没有雷达数据的时候，根据卡尔曼滤波的先验估计*0.4 + 马盘数据的0.6作为输入
3.卡尔曼滤波的增益一直是滤波过后的速度
4.当马盘的累计误差达到5000的时候，马盘的数据重新定位
*/
struct EKF odo_ladar_kalman_x = {
	.q = 0.002,
	.r = 1.2,
};
struct EKF odo_ladar_kalman_y = {
	.q = 0.002,
	.r = 1.2,
};

//////////////////////////////////////////////////////////////////////////////////雷达与马盘的数据融合/////////////////////////////////////////////////////////////////////
void Ladar_With_Odometer_Kalman(float dt){

}	











