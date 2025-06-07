#include "Television.h"
struct Vision vision;


float basket_xoffset = 0;
float basket_yoffset = 0;
//从厕所半场出发
//float ladar2siteangleoffset = -1.45; 
//从宿舍半场出发
float ladar2siteangleoffset = -2.4;

void Vision_Basket_Decode(void)
{
#if OLD_COMMUNICATION
	memcpy(vision.convert.uint8_data, vision.basketlock.data, 20);
	vision.vfield.basket_vfield.x = vision.convert.float_data[0] * 1000 + basket_xoffset;
	vision.vfield.basket_vfield.y = vision.convert.float_data[1] * 1000 + basket_yoffset;

	vision.vfield.ladar_vfield.x = vision.convert.float_data[2] * 1000;
	vision.vfield.ladar_vfield.y = vision.convert.float_data[3] * 1000;
	vision.vfield.ladar_vfield.r = vision.convert.float_data[4] * rad2ang(1);
	
	
	memcpy(vision.convert.uint8_data, vision.position.data, 20);
	vision.vfield.carzero_vfield.r = vision.convert.float_data[3] * rad2ang(1);
	vision.vfield.carzero_vfield.x = vision.convert.float_data[0] * 1000 - 126.69 * (sin(2 * PI * 0.16 * ang2rad(vision.vfield.carzero_vfield.r) + 1.29) - sin(1.29));
	vision.vfield.carzero_vfield.y = vision.convert.float_data[1] * 1000 - 124.75 * (sin(2 * PI * 0.16 * ang2rad(vision.vfield.carzero_vfield.r) - 0.25) + sin(0.25));
	
	
	float r = ang2rad(ladar2siteangleoffset);
	vision.field.carcenter_field.x = vision.vfield.carzero_vfield.x * cos(r) + vision.vfield.carzero_vfield.y * sin(r) + 390;
	vision.field.carcenter_field.y = vision.vfield.carzero_vfield.y * cos(r) - vision.vfield.carzero_vfield.x * sin(r) - 386;
	vision.field.carcenter_field.r = vision.vfield.carzero_vfield.r;
#else
	memcpy(vision.convert.uint8_data, vision.basketlock.data, 20);
	vision.vfield.basket_vfield.x = vision.convert.float_data[0] * 1000 + basket_xoffset;
	vision.vfield.basket_vfield.y = vision.convert.float_data[1] * 1000 + basket_yoffset;

	vision.vfield.ladar_vfield.x = vision.convert.float_data[2] * 1000;
	vision.vfield.ladar_vfield.y = vision.convert.float_data[3] * 1000;
	vision.vfield.ladar_vfield.r = vision.convert.float_data[4] * rad2ang(1);
	
	
	memcpy(vision.convert.uint8_data, vision.position.data, 20);
	vision.field.carcenter_field.r = vision.convert.float_data[3] * rad2ang(1);
	vision.field.carcenter_field.x = vision.convert.float_data[0] * 1000 - 126.69 * (sin(2 * PI * 0.16 * ang2rad(vision.field.carcenter_field.r) + 1.29) - sin(1.29));
	vision.field.carcenter_field.y = vision.convert.float_data[1] * 1000 - 124.75 * (sin(2 * PI * 0.16 * ang2rad(vision.field.carcenter_field.r) - 0.25) + sin(0.25));
#endif
	
}
void Get_Vision_Data(int header, unsigned char *data){
	switch (header)
	{
	case basket_id:
		vision.basketlock.online_flag = true;
		memcpy(vision.basketlock.data, data, 20);
		break;
	case position_id:
		vision.position.online_flag = true;
		vision.position.unused_flag = true;
		memcpy(vision.position.data, data, 24);
		break;
	case online_id:
		vision.position.online_flag = true;
	default:
		break;
	}
}
void Send_Velocity_Vision(void)
{
#ifndef Carbon_Car
	memset(vision.convert.uint8_data, 0, sizeof(vision.convert));
	vision.convert.float_data[0] = site.field.x_enc / 1000;
	vision.convert.float_data[1] = site.field.y_enc / 1000;
	vision.convert.float_data[2] = ang2rad(site.now.r);
#define MAX_VELOCITY 2
#define MIN_VELOCITY 0.3
	vision.convert.float_data[3] = ((site.car.velocity_totalgyro < MAX_VELOCITY) && (site.car.velocity_totalgyro > MIN_VELOCITY)) ? 1.00f : 0.00f;
	memcpy(vision.send, vision.convert.uint8_data, sizeof(vision.send));
	FDCAN_Send(&hfdcan3, site_id, "STD", vision.send, "FD", 16, "OFF");
#endif
}

#define position_kalman_encinterp false
#define position_liner_encinterp true

struct EKF ladarx_interp = {
	.q = 0.7,
	.r = 1.8,
};
struct EKF ladary_interp = {
	.q = 0.7,
	.r = 1.8,
};
struct EKF ladarr_interp = {
	.q = 0.002,
	.r = 1.8,
};

void LadarPosInterpolation(int dt)
{
#if position_liner_encinterp
	static float dx, dy,dr;
	if (vision.position.unused_flag == true)
	{
		dx = 0, dy = 0,dr = 0;
		vision.position.unused_flag = false;
	}
	else
	{
		dx += site.field.vx_enc * dt;
		dy += site.field.vy_enc * dt;
		dr += site.gyro.omiga * dt / 1000;
	}
	vision.vfield.carzero_vfieldinterp.x = vision.vfield.carzero_vfield.x + dx;
	vision.vfield.carzero_vfieldinterp.y = vision.vfield.carzero_vfield.y + dy;
	vision.vfield.carzero_vfieldinterp.r = NormalizeAng_Single(vision.vfield.carzero_vfield.r + dr);
	
	vision.field.carcenter_fieldinterp.x = vision.field.carcenter_field.x + dx * 0.5;
	vision.field.carcenter_fieldinterp.y = vision.field.carcenter_field.y + dy * 0.5;
#elif position_kalman_encinterp
	vision.vfield.car_vfieldinterp.x = EKF_Filter(&ladarx_interp, vision.basket.car_vfield.x, basketlock.parameter.siteinterp_gain * basketlock.parameter.siteinterp_gain * dt * site.field.vx_enc);
	vision.vfield.car_vfieldinterp.y = EKF_Filter(&ladary_interp, vision.basket.car_vfield.y, basketlock.parameter.siteinterp_gain * basketlock.parameter.siteinterp_gain * dt * site.field.vy_enc);
	vision.vfield.car_vfieldinterp.r = NormalizeAng_Single(EKF_Filter(&ladarr_interp,vision.vfield.car_vfield.r,basketlock.parameter.angleinterp_gain*ang2rad(site.gyro.omiga)));
#endif
	static float partialr_last;
	basketpositionlock.now.partial.x += site.field.vx_enc * dt;
	basketpositionlock.now.partial.y += site.field.vy_enc * dt;
	basketpositionlock.now.partial.r += site.gyro.r - partialr_last;
	partialr_last = site.gyro.r;
	//计算锁自己篮框的角度
	
	basketlock.protectselfbasket_angle = rad2ang(atan2f(vision.field.carcenter_field.y - self_basket_point.y,vision.field.carcenter_field.x - self_basket_point.x));
}

void Vision_Reset() {    
	FDCAN_Send(&hfdcan3,vision_reset_id,"STD",NULL,"FD",4,"OFF");   
	vision.basketlock.online_flag = false;	
	vision.position.online_flag = false;	
	osDelay(10);	
	vision.basketlock.online_flag = false;	
	vision.position.online_flag = false;	
}



























































































//	vision.field.carcenter_field.x = vision.convert.float_data[0] * 1000 - 126.69 * (sin(2 * PI * 0.16 * ang2rad(vision.vfield.ladar_vfield.r)));
//	vision.field.carcenter_field.y = vision.convert.float_data[1] * 1000 - 124.75 * (sin(2 * PI * 0.16 * ang2rad(vision.vfield.ladar_vfield.r)));
//	vision.field.carcenter_field.r = vision.convert.float_data[3] * rad2ang(1);
	
//	vision.vfield.carzero_vfield.x = vision.vfield.ladar_vfield.x - 126.69 * (sin(2 * PI * 0.16 * ang2rad(vision.vfield.ladar_vfield.r) + 1.29));
//	vision.vfield.carzero_vfield.y = vision.vfield.ladar_vfield.y - 124.75 * (sin(2 * PI * 0.16 * ang2rad(vision.vfield.ladar_vfield.r) - 0.25));
//	vision.vfield.carzero_vfield.r = vision.vfield.ladar_vfield.r;
	
//	float r = ang2rad(ladar2siteangleoffset);
//	vision.field.carcenter_field.x = vision.vfield.carzero_vfield.x * cos(r) + vision.vfield.carzero_vfield.y * sin(r) + 390;
//	vision.field.carcenter_field.y = vision.vfield.carzero_vfield.y * cos(r) - vision.vfield.carzero_vfield.x * sin(r) - 386;
//	vision.field.carcenter_field.r = vision.vfield.carzero_vfield.r;










////获取视觉数据
// #ifdef Carbon_Car
//	float basket_xoffset = -100;
//	float basket_yoffset = 0;
//	float ladar2siteangleoffset = -1.4;
// #else
//	float basket_xoffset = 230;
//	float basket_yoffset = 0;
//	float ladar2siteangleoffset = -0.5;
// #endif
// void Vision_Basket_Decode(void){
// #ifdef Carbon_Car
//	memcpy(vision.convert.uint8_data,vision.pos.data,16);
//	vision.pos.ladar_field.x = vision.convert.float_data[0] * 1000;
//	vision.pos.ladar_field.y = vision.convert.float_data[1] * 1000;
//	vision.pos.ladar_field.r = vision.convert.float_data[3] * rad2ang(1);
//
//	vision.pos.car_field.x = vision.pos.ladar_field.x - 126.69*(sin(2 *PI*0.16* ang2rad(vision.pos.ladar_field.r) + 1.29) - sin(1.29));
//	vision.pos.car_field.y = vision.pos.ladar_field.y - 124.75*(sin(2 *PI*0.16* ang2rad(vision.pos.ladar_field.r) - 0.25) + sin(0.25));
//	vision.pos.car_field.r = vision.pos.ladar_field.r;
//
//
//	memcpy(vision.convert.uint8_data,vision.basket.data,24);
//	vision.basket.basket_vfield.x = vision.convert.float_data[0] * 1000 + basket_xoffset;
//	vision.basket.basket_vfield.y = vision.convert.float_data[1] * 1000 + basket_yoffset;
//
//	vision.basket.ladar_vfield.x = vision.convert.float_data[2] * 1000;
//	vision.basket.ladar_vfield.y = vision.convert.float_data[3] * 1000;
//	vision.basket.ladar_vfield.r = vision.convert.float_data[5] * rad2ang(1);
//
//	vision.basket.car_vfield.x = vision.basket.ladar_vfield.x - 126.69*(sin(2 *PI*0.16* ang2rad(vision.pos.ladar_field.r) + 1.29));
//	vision.basket.car_vfield.y = vision.basket.ladar_vfield.y - 124.75*(sin(2 *PI*0.16* ang2rad(vision.pos.ladar_field.r) - 0.25));
//	vision.basket.car_vfield.r = vision.basket.ladar_vfield.r;
//
//	vision.basket.height = vision.convert.float_data[4] * 1000;
// #else
//	memcpy(vision.convert.uint8_data,vision.pos.data,16);
//	//视觉坐标系下车体当前坐标
//	vision.basket.ladar_vfield.x = vision.convert.float_data[0] * 1000;
//	vision.basket.ladar_vfield.y = vision.convert.float_data[1] * 1000;
//	vision.basket.ladar_vfield.r = vision.convert.float_data[3] * rad2ang(1);
//
//	float x = vision.basket.ladar_vfield.x - 261.31 * (sin(2 * PI * 0.16 * ang2rad(vision.basket.ladar_vfield.r) + 1.61) - sin(1.61));
//	float y = vision.basket.ladar_vfield.y - 258.26 * (sin(2 * PI * 0.16 * ang2rad(vision.basket.ladar_vfield.r) + 0.04) - sin(0.04));
//	float r = ang2rad(ladar2siteangleoffset);
//	vision.basket.car_vfield.x = x * cos(r) + y * sin(r);
//	vision.basket.car_vfield.y = y * cos(r) - x * sin(r);
//	vision.basket.car_vfield.r = vision.basket.ladar_vfield.r;

//	//获取视觉坐标系下的篮筐坐标
//	memcpy(vision.convert.uint8_data,vision.basket.data,8);
//	vision.basket.basket_vfield.x = vision.convert.float_data[0] * 1000 + basket_xoffset;
//	vision.basket.basket_vfield.y = vision.convert.float_data[1] * 1000 + basket_yoffset;
// #endif
//}
// void Get_Vision_Data(int header,unsigned char * data){
// #ifdef Carbon_Car
//	switch(header){
//		case basket_id:
//			memcpy(vision.basket.data,data,24);
//			vision.basket.online_flag = 1;
//		break;
//		case pos_id:
//			memcpy(vision.pos.data,data,16);
//		break;
//		case online_id:
//			vision.pos.online_flag = 1;
//		break;
//		default:
//			break;
//	}
// #else
//	switch(header){
//		case pos_id:
//			memcpy(vision.pos.data,data,16);
//			vision.pos.online_flag = 1;
//		break;
//		case basket_id:
//			memcpy(vision.basket.data,data,8);
//			vision.basket.online_flag = 1;
//		break;
//		default:
//			break;
//	}
// #endif
//}

// void Send_Velocity_Vision(void){
// #ifndef Carbon_Car
//	memset(vision.convert.uint8_data,0,sizeof(vision.convert));
//	vision.convert.float_data[0] = site.field.x_enc / 1000;
//	vision.convert.float_data[1] = site.field.y_enc / 1000;
//	vision.convert.float_data[2] = ang2rad(site.now.r);
// #define MAX_VELOCITY 2
// #define MIN_VELOCITY 0.3
//	vision.convert.float_data[3] = ((site.car.velocity_totalgyro < MAX_VELOCITY) && (site.car.velocity_totalgyro > MIN_VELOCITY))?1.00f:0.00f;
//	memcpy(vision.send,vision.convert.uint8_data,sizeof(vision.send));
//	FDCAN_Send(&hfdcan3,site_id,"STD",vision.send,"FD",16,"OFF");
// #endif
// }

