#include "Television.h"
struct Vision vision;

#ifdef Carbon_Car
	float basket_xoffset = 0;
	float basket_yoffset = 0;
	//从厕所半场出发
//float ladar2siteangleoffset = -1.45; 
//从宿舍半场出发
float ladar2siteangleoffset = -2.4;
#else          
	float basket_xoffset = 300;
	float basket_yoffset = 0;
	float ladar2siteangleoffset = -0.5;
#endif



void Vision_Basket_Decode(void){
#ifdef Carbon_Car
	memcpy(vision.convert.uint8_data,vision.pos.data,16);
	//视觉坐标系下车体当前坐标
	vision.basket.ladar_vfield.x = vision.convert.float_data[0] * 1000;
	vision.basket.ladar_vfield.y = vision.convert.float_data[1] * 1000;
	vision.basket.height = vision.convert.float_data[2] * 1000;
	vision.basket.ladar_vfield.r = vision.convert.float_data[3] * rad2ang(1);
	
	
	
	vision.basket.car_vfield.x = vision.basket.ladar_vfield.x - 126.69*(sin(2 *PI*0.16* ang2rad(vision.basket.ladar_vfield.r) + 1.29));
	vision.basket.car_vfield.y = vision.basket.ladar_vfield.y - 124.75*(sin(2 *PI*0.16* ang2rad(vision.basket.ladar_vfield.r) - 0.25));
	vision.basket.car_vfield.r = vision.basket.ladar_vfield.r;
	
	float x = vision.basket.ladar_vfield.x - 126.69*(sin(2 *PI*0.16* ang2rad(vision.basket.ladar_vfield.r) + 1.29) - sin(1.29));
	float y = vision.basket.ladar_vfield.y - 124.75*(sin(2 *PI*0.16* ang2rad(vision.basket.ladar_vfield.r) - 0.25) + sin(0.25));
	float r = ang2rad(ladar2siteangleoffset); 
	vision.pos.car_field.x = x * cos(r) + y * sin(r) + 390;
	vision.pos.car_field.y = y * cos(r) - x * sin(r) - 386;
	vision.pos.car_field.r = vision.basket.car_vfield.r;
	
	
//	vision.pos.car_field.x = vision.basket.car_vfield.x * cos(r) + vision.basket.car_vfield.y * sin(r);
//	vision.pos.car_field.y = vision.basket.car_vfield.y * cos(r) - vision.basket.car_vfield.x * sin(r);
//	vision.pos.car_field.r = vision.basket.car_vfield.r;

	//获取视觉坐标系下的篮筐坐标
	memcpy(vision.convert.uint8_data,vision.basket.data,8);
	vision.basket.basket_vfield.x = vision.convert.float_data[0] * 1000 + basket_xoffset;
	vision.basket.basket_vfield.y = vision.convert.float_data[1] * 1000 + basket_yoffset;
#else 
	memcpy(vision.convert.uint8_data,vision.pos.data,16);
	//视觉坐标系下车体当前坐标
	vision.basket.ladar_vfield.x = vision.convert.float_data[0] * 1000;
	vision.basket.ladar_vfield.y = vision.convert.float_data[1] * 1000;
	vision.basket.height = vision.convert.float_data[2] * 1000;
	vision.basket.ladar_vfield.r = vision.convert.float_data[3] * rad2ang(1);
	
	vision.basket.car_vfield.x = vision.basket.ladar_vfield.x - 261.31 * (sin(2 * PI * 0.16 * ang2rad(vision.basket.ladar_vfield.r) + 1.61) - sin(1.61));
	vision.basket.car_vfield.y = vision.basket.ladar_vfield.y - 258.26 * (sin(2 * PI * 0.16 * ang2rad(vision.basket.ladar_vfield.r) + 0.04) - sin(0.04));
	vision.basket.car_vfield.r = vision.basket.ladar_vfield.r;
	
	float r = ang2rad(ladar2siteangleoffset);
	vision.pos.car_field.x = vision.basket.car_vfield.x * cos(r) + vision.basket.car_vfield.y * sin(r);
	vision.pos.car_field.y = vision.basket.car_vfield.y * cos(r) - vision.basket.car_vfield.x * sin(r);
	vision.pos.car_field.r = vision.basket.car_vfield.r;

	//获取视觉坐标系下的篮筐坐标
	memcpy(vision.convert.uint8_data,vision.basket.data,8);
	vision.basket.basket_vfield.x = vision.convert.float_data[0] * 1000 + basket_xoffset;
	vision.basket.basket_vfield.y = vision.convert.float_data[1] * 1000 + basket_yoffset;
#endif
}


void Get_Vision_Data(int header,unsigned char * data){
	switch(header){
		case pos_id:
			vision.basket.get_flag = 1;
			memcpy(vision.pos.data,data,16);
		break;
		case basket_id:
			memcpy(vision.basket.data,data,8);
			vision.basket.online_flag = 1;
		break;
		case online_id:
			vision.pos.online_flag = 1;
		default:
			break;
	}
}
void Send_Velocity_Vision(void){
#ifndef Carbon_Car
	memset(vision.convert.uint8_data,0,sizeof(vision.convert));
	vision.convert.float_data[0] = site.field.x_enc / 1000;
	vision.convert.float_data[1] = site.field.y_enc / 1000;
	vision.convert.float_data[2] = ang2rad(site.now.r);
#define MAX_VELOCITY 2
#define MIN_VELOCITY 0.3
	vision.convert.float_data[3] = ((site.car.velocity_totalgyro < MAX_VELOCITY) && (site.car.velocity_totalgyro > MIN_VELOCITY))?1.00f:0.00f;
	memcpy(vision.send,vision.convert.uint8_data,sizeof(vision.send));
	FDCAN_Send(&hfdcan3,site_id,"STD",vision.send,"FD",16,"OFF");
#endif
}
void LadarPosInterpolation(int dt){
	static float dx,dy;
	if(vision.basket.get_flag == 1){
		dx = 0,dy = 0;
		vision.basket.get_flag = 0;	}
	else{
		dx += site.field.vx_enc * dt;
	  dy += site.field.vy_enc * dt;	}
	vision.basket.car_vfieldinterpenc.x = vision.basket.car_vfield.x + dx;
	vision.basket.car_vfieldinterpenc.y = vision.basket.car_vfield.y + dy;
	vision.basket.car_vfieldinterpenc.r = vision.basket.car_vfield.r;
}

























////获取视觉数据 
//#ifdef Carbon_Car
//	float basket_xoffset = -100;
//	float basket_yoffset = 0;
//	float ladar2siteangleoffset = -1.4;
//#else
//	float basket_xoffset = 230;
//	float basket_yoffset = 0;
//	float ladar2siteangleoffset = -0.5;
//#endif
//void Vision_Basket_Decode(void){
//#ifdef Carbon_Car
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
//#else 
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
//#endif
//}
//void Get_Vision_Data(int header,unsigned char * data){
//#ifdef Carbon_Car
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
//#else 
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
//#endif
//}

//void Send_Velocity_Vision(void){
//#ifndef Carbon_Car
//	memset(vision.convert.uint8_data,0,sizeof(vision.convert));
//	vision.convert.float_data[0] = site.field.x_enc / 1000;
//	vision.convert.float_data[1] = site.field.y_enc / 1000;
//	vision.convert.float_data[2] = ang2rad(site.now.r);
//#define MAX_VELOCITY 2
//#define MIN_VELOCITY 0.3
//	vision.convert.float_data[3] = ((site.car.velocity_totalgyro < MAX_VELOCITY) && (site.car.velocity_totalgyro > MIN_VELOCITY))?1.00f:0.00f;
//	memcpy(vision.send,vision.convert.uint8_data,sizeof(vision.send));
//	FDCAN_Send(&hfdcan3,site_id,"STD",vision.send,"FD",16,"OFF");
//#endif
//}






































