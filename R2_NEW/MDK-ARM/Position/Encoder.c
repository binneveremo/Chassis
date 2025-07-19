#include "Can_Bsp.h"
#include "Encoder.h"
#include "Global.h"
#include "string.h"
#include "math.h"
#include "Gyro.h"
//定义enc1为记录x轴方向的码盘，调节sign1使得向前的时候增量为正数
//定义enc2为记录y轴方向的码盘，调节sign2使得向左的时候增量为正数
#define odo_can hfdcan2
#define circle_num 32


float signx = 1;
float signy = -1;
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
	static float car_x_field_last,car_y_field_last;

  Diff_Odometer();
	//计算车体码盘                          
	float dy_car = ( odometer.do1  * 0.70710678 - odometer.do2  * 0.70710678) / ratio;
	float dx_car = (-odometer.do2  * 0.70710678 - odometer.do1  * 0.70710678) / ratio;
	//计算车体坐标系的速度
	odometer.dy_field = dx_car* cos(ang2rad(site.now.r)) - dy_car*sin(ang2rad(site.now.r));
	odometer.dx_field = dx_car* sin(ang2rad(site.now.r)) + dy_car*cos(ang2rad(site.now.r));
	//计算出场地坐标
	odometer.enc_x_field = odometer.dx_field + odometer.enc_x_field;
	odometer.enc_y_field = odometer.dy_field + odometer.enc_y_field;
	//计算出车体中心的场地坐标
	//142.80, 频率: 0.16, 相位: -1.55, 偏移: -28.42 x轴
	//140.95, 频率: 0.16, 相位: 0.01, 偏移: 226.87 y轴
	odometer.car_x_field = odometer.enc_x_field - 142.80 * (sin(2 * PI * 0.16 * ang2rad(site.now.r) - 1.55));
	odometer.car_y_field = odometer.enc_y_field - 140.95 * (sin(2 * PI * 0.16 * ang2rad(site.now.r) + 0.01));
	//根据坐标中心计算出真实的场地速度
	site.field.vx_enc = (odometer.car_x_field - car_x_field_last) / dt;
	site.field.vy_enc = (odometer.car_y_field - car_y_field_last) / dt;
	//保留上一次码盘中心坐标的值
	car_x_field_last = odometer.car_x_field;
	car_y_field_last = odometer.car_y_field;
	
	Field2Car(&site.car.vx_enc,&site.car.vy_enc,&site.field.vx_enc,&site.field.vy_enc);
	
	site.field.x_enc = odometer.car_x_field;
	site.field.y_enc = odometer.car_y_field;
	
	site.car.velocity_totalenc = hypot(site.car.vx_enc,site.car.vy_enc);
}
void Encoder_Init(void){
	Set_ZeroPoint(0x01);
	HAL_Delay(100);
	Set_ZeroPoint(0x02);
	HAL_Delay(100);
	Encoder_XY_VX_VY_Cal(1);
	Odometer_Clear("armor");
}
void Odometer_Clear(char * ifarmor){
	if(strcmp(ifarmor,"armor") == 0){
		odometer.car_x_field =  450;
		odometer.car_y_field = -425;
	}
	else {
		odometer.car_x_field =  410;
		odometer.car_y_field = -385;
	}
}








