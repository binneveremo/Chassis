#include "Television.h"
#include "Gyro.h"

struct Vision vision = {.param.basket_xoffset = 300,.param.ladar2siteangleoffset = -2.4};
#define OLD_COMMUNICATION false

void Vision_DataDecode(void){	
	if(vision.flagof.used_flag == true)
		return;
	memcpy(vision.convert.uint8_data, vision.basketlock.data, 20);
	vision.visual.basket_visual.x = vision.convert.float_data[0] * 1000 + vision.param.basket_xoffset;
	vision.visual.basket_visual.y = vision.convert.float_data[1] * 1000 + vision.param.basket_yoffset;
	
	
	memcpy(vision.convert.uint8_data, vision.position.data,16);
	vision.visual.ladar_visual.x = vision.convert.float_data[0] * 1000;
	vision.visual.ladar_visual.y = vision.convert.float_data[1] * 1000;
	vision.visual.ladar_visual.r = vision.convert.float_data[3] * rad2ang(1);

	vision.field.carcenter_field.r = vision.convert.float_data[3] * rad2ang(1);
	vision.field.carcenter_field.x = vision.convert.float_data[0] * 1000 - 264.07 * (sin(2 * PI * 0.16 * ang2rad(vision.visual.ladar_visual.r) + 1.54));
	vision.field.carcenter_field.y = vision.convert.float_data[1] * 1000 - 261.97 * (sin(2 * PI * 0.16 * ang2rad(vision.visual.ladar_visual.r) - 0.01));
	
	vision.visual.carzero_visual.x = vision.convert.float_data[0] * 1000 - 264.07 * (sin(2 * PI * 0.16 * ang2rad(vision.field.carcenter_field.r) + 1.54) - sin(1.54));
	vision.visual.carzero_visual.y = vision.convert.float_data[1] * 1000 - 261.97 * (sin(2 * PI * 0.16 * ang2rad(vision.field.carcenter_field.r) + 0.01) - sin(0.01));
	vision.visual.carzero_visual.r = vision.convert.float_data[3] * rad2ang(1);
	
	if((vision.flagof.gyro_offset_angle_init == false) && (vision.header == position_id) && (vision.position.online_flag == true))
		Gyro_AngleReset(vision.visual.ladar_visual.r),vision.flagof.gyro_offset_angle_init = true;
	
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
		vision.flagof.used_flag = false;
		memcpy(vision.position.data, data, 16);
		break;
	case online_id:
		vision.position.online_flag = true;
		vision.flagof.gyro_offset_angle_init = false;
	default:
		break;
	}
	vision.header = header;
}

void Vision_Filed_Basket_XY_Cal(int dt){
	Vision_DataDecode();
	static float dx, dy,dr;
	if (vision.flagof.used_flag == false)
	{
		dx = 0, dy = 0,dr = 0;
		vision.flagof.used_flag = true;
	}
	else
	{
		dx += site.field.vx_enc * dt;
		dy += site.field.vy_enc * dt;
		dr += site.gyro.omiga * dt / 1000;
	}
	vision.visual.carzero_visualinterp.x = vision.visual.carzero_visual.x + dx;
	vision.visual.carzero_visualinterp.y = vision.visual.carzero_visual.y + dy;
	vision.visual.carzero_visualinterp.r = NormalizeAng_Single(vision.visual.carzero_visual.r + dr);
	
	vision.field.carcenter_fieldinterp.x = vision.field.carcenter_field.x + dx;
	vision.field.carcenter_fieldinterp.y = vision.field.carcenter_field.y + dy;
	//计算锁自己篮框的角度
	basketlock.protectselfbasket_angle = rad2ang(atan2f(vision.field.carcenter_field.y - self_basket_point.y,vision.field.carcenter_field.x - self_basket_point.x));
	
	BasketPositionCal_AccordingVision(dt);
}

















































































//	vision.field.carcenter_field.x = vision.convert.float_data[0] * 1000 - 126.69 * (sin(2 * PI * 0.16 * ang2rad(vision.visual.ladar_visual.r)));
//	vision.field.carcenter_field.y = vision.convert.float_data[1] * 1000 - 124.75 * (sin(2 * PI * 0.16 * ang2rad(vision.visual.ladar_visual.r)));
//	vision.field.carcenter_field.r = vision.convert.float_data[3] * rad2ang(1);
	
//	vision.visual.carzero_visual.x = vision.visual.ladar_visual.x - 126.69 * (sin(2 * PI * 0.16 * ang2rad(vision.visual.ladar_visual.r) + 1.29));
//	vision.visual.carzero_visual.y = vision.visual.ladar_visual.y - 124.75 * (sin(2 * PI * 0.16 * ang2rad(vision.visual.ladar_visual.r) - 0.25));
//	vision.visual.carzero_visual.r = vision.visual.ladar_visual.r;
	
//	float r = ang2rad(ladar2siteangleoffset);
//	vision.field.carcenter_field.x = vision.visual.carzero_visual.x * cos(r) + vision.visual.carzero_visual.y * sin(r) + 390;
//	vision.field.carcenter_field.y = vision.visual.carzero_visual.y * cos(r) - vision.visual.carzero_visual.x * sin(r) - 386;
//	vision.field.carcenter_field.r = vision.visual.carzero_visual.r;










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
//	vision.basket.basket_visual.x = vision.convert.float_data[0] * 1000 + basket_xoffset;
//	vision.basket.basket_visual.y = vision.convert.float_data[1] * 1000 + basket_yoffset;
//
//	vision.basket.ladar_visual.x = vision.convert.float_data[2] * 1000;
//	vision.basket.ladar_visual.y = vision.convert.float_data[3] * 1000;
//	vision.basket.ladar_visual.r = vision.convert.float_data[5] * rad2ang(1);
//
//	vision.basket.car_visual.x = vision.basket.ladar_visual.x - 126.69*(sin(2 *PI*0.16* ang2rad(vision.pos.ladar_field.r) + 1.29));
//	vision.basket.car_visual.y = vision.basket.ladar_visual.y - 124.75*(sin(2 *PI*0.16* ang2rad(vision.pos.ladar_field.r) - 0.25));
//	vision.basket.car_visual.r = vision.basket.ladar_visual.r;
//
//	vision.basket.height = vision.convert.float_data[4] * 1000;
// #else
//	memcpy(vision.convert.uint8_data,vision.pos.data,16);
//	//视觉坐标系下车体当前坐标
//	vision.basket.ladar_visual.x = vision.convert.float_data[0] * 1000;
//	vision.basket.ladar_visual.y = vision.convert.float_data[1] * 1000;
//	vision.basket.ladar_visual.r = vision.convert.float_data[3] * rad2ang(1);
//
//	float x = vision.basket.ladar_visual.x - 261.31 * (sin(2 * PI * 0.16 * ang2rad(vision.basket.ladar_visual.r) + 1.61) - sin(1.61));
//	float y = vision.basket.ladar_visual.y - 258.26 * (sin(2 * PI * 0.16 * ang2rad(vision.basket.ladar_visual.r) + 0.04) - sin(0.04));
//	float r = ang2rad(ladar2siteangleoffset);
//	vision.basket.car_visual.x = x * cos(r) + y * sin(r);
//	vision.basket.car_visual.y = y * cos(r) - x * sin(r);
//	vision.basket.car_visual.r = vision.basket.ladar_visual.r;

//	//获取视觉坐标系下的篮筐坐标
//	memcpy(vision.convert.uint8_data,vision.basket.data,8);
//	vision.basket.basket_visual.x = vision.convert.float_data[0] * 1000 + basket_xoffset;
//	vision.basket.basket_visual.y = vision.convert.float_data[1] * 1000 + basket_yoffset;
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

