#include "Can_Bsp.h"
#include "Encoder.h"
#include "Global.h"
#include "string.h"
#include "math.h"
#include "Gyro.h"
//定义enc1为记录x轴方向的码盘，调节sign1使得向前的时候增量为正数
//定义enc2为记录y轴方向的码盘，调节sign2使得向左的时候增量为正数
#define odo_can hfdcan2
#ifdef Carbon_Car
	#define circle_num 32
#else 
	#define circle_num 50
#endif

#define signx -1
#define signy 1
#define encx_id 1
#define ency_id 2
#define ratio 25.321f
unsigned char encoder_data[2][8];
unsigned char encoder_send[8];
struct Odometer odometer;
void Get_Encoder_Data(int id,unsigned char * data){
	if(id == encx_id)
		memcpy(&encoder_data[0],data,7),odometer.xenc_online = true;
	else if(id == ency_id)
		memcpy(&encoder_data[1],data,7),odometer.yenc_online = true;
}
void Diff_Odometer(void){
	odometer.o1_row = ((encoder_data[0][6] << 24) + (encoder_data[0][5] << 16) + (encoder_data[0][4] << 8) + encoder_data[0][3]);
	odometer.o2_row = ((encoder_data[1][6] << 24) + (encoder_data[1][5] << 16) + (encoder_data[1][4] << 8) + encoder_data[1][3]);
	odometer.do1 = signx*(odometer.o1_row - odometer.o1_pre);
	odometer.do2 = signy*(odometer.o2_row - odometer.o2_pre);
	if(odometer.do1 > circle_num * 2048)
    odometer.do1 -= circle_num * 4096;
	else if(odometer.do1 < -circle_num * 2048)
		odometer.do1 += circle_num * 4096;
	if(odometer.do2 > circle_num * 2048)
    odometer.do2 -= circle_num * 4096;
	else if(odometer.do2 < -circle_num * 2048)
		odometer.do2 += circle_num * 4096;
	odometer.o1_pre = odometer.o1_row;
	odometer.o2_pre = odometer.o2_row;
}
void Set_ZeroPoint(unsigned char ID){
	encoder_send[0]=0x04;
	encoder_send[1]=ID;
	encoder_send[2]=0x06;
	encoder_send[3]=0x00;
	FDCAN_Send(&odo_can,ID,"STD",encoder_send,"CLASSIC",4,"OFF"); 
}
void Encoder_XY_VX_VY_Cal(int dt){
  Diff_Odometer();
	//计算旋转补偿
	//计算车体速度
#ifdef Carbon_Car
	float dx_car = ( odometer.do1  * 0.70710678 - odometer.do2  * 0.70710678) / ratio;
	float dy_car = (-odometer.do2  * 0.70710678 - odometer.do1  * 0.70710678) / ratio;
#else
	float dx_car = odometer.do1 / ratio;
	float dy_car = odometer.do2 / ratio;
#endif
	//计算车体坐标系的速度
	//初步计算场地坐标系的dx dy
	float dx_field = dx_car* cos(ang2rad(site.now.r)) - dy_car*sin(ang2rad(site.now.r));
	float dy_field = dx_car* sin(ang2rad(site.now.r)) + dy_car*cos(ang2rad(site.now.r));
	//计算误差角度之后的dx dy
	odometer.dx_field = dx_field* cos(odometer.offset_angle) - dy_field*sin(odometer.offset_angle);
	odometer.dy_field = dy_field* cos(odometer.offset_angle) + dx_field*sin(odometer.offset_angle);
	//对 dx dy 进行积分
	odometer.x_field = odometer.dx_field + odometer.x_field;
	odometer.y_field = odometer.dy_field + odometer.y_field;
	////与定位系统交互
	site.car.vx_enc = dx_car / dt;
	site.car.vy_enc = dy_car / dt;
	site.car.velocity_totalenc = hypot(site.car.vx_enc,site.car.vy_enc);
	//与定位系统交互
	site.field.x_enc = odometer.x_field;
	site.field.y_enc = odometer.y_field;
	//计算位置微分 也就是速度
	site.car.vx_enc = odometer.dx_field / dt;
	site.car.vy_enc = odometer.dy_field / dt;
	//计算行驶过的累计历程
	odometer.xdis += fabs(odometer.dx_field);
	odometer.ydis += fabs(odometer.dy_field);
	//辅助坐标系的计算
	site.partial.enc.x += odometer.dx_field;
	site.partial.enc.y += odometer.dy_field;
}
void Encoder_Init(void){
	Set_ZeroPoint(0x01);
	HAL_Delay(100);
	Set_ZeroPoint(0x02);
	HAL_Delay(100);
	Encoder_XY_VX_VY_Cal(1);
	Odometer_Clear("armor");
	odometer.offset_angle = 0;
}
void Odometer_Clear(char * ifarmor){
	if(strcmp(ifarmor,"armor") == 0){
		odometer.x_field =  450;
		odometer.y_field = -425;
	}
	else {
		odometer.x_field =  410;
		odometer.y_field = -385;
	}
}








