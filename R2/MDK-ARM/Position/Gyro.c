#include "Kalman.h"
#include "Global.h"
#include "Gyro.h"
#include "usart.h"
#include "dma.h"
#include "string.h"
#include "math.h"
#include "fdcan.h"
struct YIS506 yis506;
void Gyro_AX_AY_Cal(void){
	site.car.ax_gyro = -(yis506.X.accel - yis506.X.accel_offset);
	site.car.ay_gyro = -(yis506.Y.accel - yis506.Y.accel_offset);
	site.car.accel_totalgyro = hypot(site.car.ax_gyro,site.car.ay_gyro);
	site.field.ax_gyro = site.car.ax_gyro * cos(ang2rad(site.now.r)) - site.car.ay_gyro * sin(ang2rad(site.now.r));
  site.field.ay_gyro = site.car.ax_gyro * sin(ang2rad(site.now.r)) + site.car.ay_gyro * cos(ang2rad(site.now.r));
}
void Get_Gyro_Data(int header,unsigned char * data){
	if((header == ExtID_accel) ||  (header == ExtID_omega) || (header == ExtID_yaw)){
		memcpy(yis506.data,data,8);
		yis506.header = header;
		YIS506_Decode();
	}
}
void Accel_Cal(void)
{
  //把指向一个八位的指针转成指向16位的指针，以达到位运算的目的
  yis506.X.accel = *(const uint16_t*)&(yis506.data[0])*accel_ratio-_accel_offset;  
  yis506.Y.accel = *(const uint16_t*)&(yis506.data[2])*accel_ratio-_accel_offset;  
  yis506.Z.accel = *(const uint16_t*)&(yis506.data[4])*accel_ratio-_accel_offset;  
}
//获取角度以及角速度
void Omega_Cal(void)
{
  unsigned char temp[4];
  //注意大端小端模式
  memmove(temp,&yis506.data[0],3);
  yis506.X.omega = ((*(uint32_t*)temp) &0x000fffff)*omega_ratio - _omega_offset;
  memmove(temp,&yis506.data[2],3);                
  yis506.Y.omega = ((*(uint32_t*)temp) >> 4)*omega_ratio - _omega_offset;
  memmove(temp,&yis506.data[5],3);
  yis506.Z.omega = ((*(uint32_t*)temp) &0x000fffff)*omega_ratio - _omega_offset;
  site.gyro.omiga = yis506.Z.omega;
}
void Euler_Cal(void)
{
	yis506.euler.pitch = *(const unsigned short int*)&(yis506.data[0])*euler_radio -_euler_offset;
	yis506.euler.roll =  *(const unsigned short int*)&(yis506.data[2])*euler_radio -_euler_offset;
	yis506.euler.yaw  =  *(const unsigned short int*)&(yis506.data[4])*euler_radio -_euler_offset;
	float angle = yis506.euler.yaw - yis506.euler.yaw_offset;
	site.gyro.r = (angle > 180)?angle - 360:(angle < -180 ? angle + 360:angle);
}
//////////////////////////////////////////陀螺仪中断解算//////////////////////////////////
void YIS506_Decode(void){
	if(yis506.header == ExtID_yaw)
		Euler_Cal(),yis506.online_flag = true;
	else if(yis506.header == ExtID_accel)
		Accel_Cal();
	else if(yis506.header == ExtID_omega)
		Omega_Cal();
}
/////////////////////////////////////////////定期修正陀螺仪角度/////////////////////////////
void Gyro_Reset(void){
	yis506.reset_flag = true;
	yis506.euler.yaw_offset = yis506.euler.yaw;
}
void Gyro_Init(void){
	yis506.euler.yaw_offset = yis506.euler.yaw;
}


























