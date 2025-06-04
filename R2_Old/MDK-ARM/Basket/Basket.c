#include "Basket.h"
//1.底盘跑点出现歪歪扭扭的情况 记得给板子复位 或者进如debug模式 就会变好(我也不知道为什么)
//2.给视觉复位 记得进入手柄的debug界面 当basket界面出现6.66和较为稳定的数值 并且ladar的定位数值接近为0 就可以发车了
//3.今天晚上我测的篮筐扣篮死区特别小 如果要调小的话 修改bl.parameter.basketdis = 967这个参数
struct BasketAngle_Lock_t basketanglelock;
struct BasketPosition_Lock_t basketpositionlock;
struct Basket_Lock_t basketlock;
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
struct Point basket_point = {
	.x = 12050,
	.y = -4000,
	.r = 0
};
void BasketPositionCal_AccordingVision(float dt){
	//卡尔曼预测
	memcpy(&basketlock.position.now_interp_vfield,&vision.basket.car_vfieldinterpenc,sizeof(vision.basket.car_vfieldinterpenc));
//	basketlock.position.now_interp_vfield.x = EKF_Filter(&ladarx_interp,vision.basket.car_vfield.x,basketlock.parameter.siteinterp_gain*basketlock.parameter.siteinterp_gain*dt * site.field.vx_enc);
//	basketlock.position.now_interp_vfield.y = EKF_Filter(&ladary_interp,vision.basket.car_vfield.y,basketlock.parameter.siteinterp_gain*basketlock.parameter.siteinterp_gain*dt * site.field.vy_enc);
	basketlock.position.now_interp_vfield.r = NormalizeAng_Single(EKF_Filter(&ladarr_interp,vision.basket.car_vfield.r,basketlock.parameter.angleinterp_gain*ang2rad(site.gyro.omiga)));
	
	basketlock.position.ladar2basketx = vision.basket.basket_vfield.x - basketlock.position.now_interp_vfield.x;
	basketlock.position.ladar2baskety = vision.basket.basket_vfield.y - basketlock.position.now_interp_vfield.y;
	basketlock.position.ladar2basketangle = atan2f(basketlock.position.ladar2baskety , basketlock.position.ladar2basketx);
	basketlock.position.ladar2basketdis = hypot(basketlock.position.ladar2basketx,basketlock.position.ladar2baskety);
	
	basketlock.flagof.danger = ((basketlock.position.ladar2basketdis < 700) && (vision.basket.online_flag == true))?true:false;
}
float BasketAngleLock(void){
	basketanglelock.progress.error = basketlock.position.basket_target_vfield.r - basketlock.position.now_interp_vfield.r;
	float out;
	float p = basketanglelock.param.p * basketanglelock.progress.error;
	basketanglelock.progress.gain = Limit(basketanglelock.param.velocity_gain * site.car.velocity_totalenc + basketanglelock.param.accel_gain * site.car.accel_totalgyro, 1, 4);
	float d = basketanglelock.param.d * site.gyro.omiga;  
	//完成增益的衰减
	basketanglelock.progress.gain *= Normalize_Pow(basketanglelock.param.istart,basketanglelock.param.iend,basketanglelock.progress.error,2);
	//完成积分分离
	basketanglelock.progress.itotal = ((fabs(basketanglelock.progress.error) > basketanglelock.param.istart) && (fabs(basketanglelock.progress.error) < basketanglelock.param.iend))?Limit(basketanglelock.progress.itotal + basketanglelock.param.i * basketanglelock.progress.error, -basketanglelock.param.ilimit, basketanglelock.param.ilimit):0;
	out = Limit(basketanglelock.progress.gain*p + basketanglelock.progress.itotal - d, -basketanglelock.param.outlimit, basketanglelock.param.outlimit);
	return out;
}

void BasketPoint_Init(void){
	if(basketlock.flagof.nearest_point_init == true) return;
	basketlock.position.basket_target_vfield.r = Limit(rad2ang(atan2f(basketlock.position.ladar2baskety , basketlock.position.ladar2basketx)),-basketlock.parameter.limitzoneanglimit,basketlock.parameter.limitzoneanglimit);
	basketlock.position.basket_target_vfield.x = vision.basket.basket_vfield.x - basketlock.parameter.basketdis * cos(ang2rad(basketlock.position.basket_target_vfield.r));
	basketlock.position.basket_target_vfield.y = vision.basket.basket_vfield.y - basketlock.parameter.basketdis * sin(ang2rad(basketlock.position.basket_target_vfield.r));
	basketlock.position.basket_target_vfield.r += basketlock.parameter.anglebetween_ladarandpole; 
	//basketlock.position.basket_target_vfield.r += (basketlock.position.basket_target_vfield.r > 0)?basketlock.parameter.anglebetween_ladarandpole:-basketlock.parameter.anglebetween_ladarandpole;
	//char repeat = (basketlock.position.basket_target_vfield.x > vision.basket.basket_vfield.x)?1:0;basketlock.parameter.anglebetween_ladarandpole:
	//basketlock.position.basket_target_vfield.x = (repeat == 1)?vision.basket.basket_vfield.x + basketlock.parameter.basketdis * cos(basketlock.position.ladar2basketangle):basketlock.position.basket_target_vfield.x;
	//basketlock.position.basket_target_vfield.r = (repeat == 1)?NormalizeAng_Single(180 - rad2ang(basketlock.position.ladar2basketangle) - basketlock.parameter.anglebetween_ladarandpole):basketlock.position.basket_target_vfield.r;
	Clear(basketpositionlock.process);
	Clear(basketpositionlock.flagof);
	//切换到码盘坐标系
	Clear(site.partial);
	site.partial.target.x = basketlock.position.basket_target_vfield.x - vision.basket.car_vfieldinterpenc.x;
	site.partial.target.y = basketlock.position.basket_target_vfield.y - vision.basket.car_vfieldinterpenc.y;
	site.partial.target.r = basketlock.position.basket_target_vfield.r - site.gyro.r;
	
	basketlock.begin = HAL_GetTick();
	basketlock.flagof.nearest_point_init = true;
}
void BsaketPoint_SelfLockAuto(void){
	/////////////////////////////判断是否距离目标点更远了 收集历史数据 如果比历史的三个点都远 就认为更远了//////////////////
#define check_num 4
	static float dis_history[check_num];
	for(int i = check_num - 1; i > 0;i--)
		dis_history[i] = dis_history[i - 1];
	char lagernum = (char)NULL;
	float disnow = Point_Distance(basketlock.position.basket_target_vfield,basketlock.position.now_interp_vfield);
	for(int i = 0; i < check_num;i++)
		lagernum = (disnow > dis_history[i])?lagernum + 1:lagernum;
	dis_history[0] = disnow;
	//距离目标点变远
	static int lock_times;
#define futher_death_bond 80
#define death_bond 20
	basketpositionlock.flagof.lock_flag = ((fabs(basketanglelock.progress.error) < 2.2) && (((lagernum >= check_num - 1) && (basketpositionlock.process.error < futher_death_bond)) || (basketpositionlock.process.error < death_bond)))?1:basketpositionlock.flagof.lock_flag;

	if(basketpositionlock.flagof.lock_flag == 1)	
		Self_Lock_Out("BasketNear");
}
void BasketPositionLock(){
	BasketPoint_Init();
//	float xerror = site.partial.target.x - site.partial.enc.x;
//	float yerror = site.partial.target.y - site.partial.enc.y;
	float xerror = basketlock.position.basket_target_vfield.x - basketlock.position.now_interp_vfield.x;
	float yerror = basketlock.position.basket_target_vfield.y - basketlock.position.now_interp_vfield.y;
	basketpositionlock.process.error = hypot(xerror,yerror);
	basketpositionlock.process.gain = Normalize_Pow(basketpositionlock.param.fade_end,basketpositionlock.param.fade_start,basketpositionlock.process.error,2);
	
	float xp = basketpositionlock.param.p * xerror;
	float xi = ((fabs(xerror) < basketpositionlock.param.iend) && (fabs(xerror) > basketpositionlock.param.istart))?basketpositionlock.param.i * xerror:0;
	basketpositionlock.process.itotal_x = (fabs(xerror) < basketpositionlock.param.istart)?0:Limit(basketpositionlock.process.itotal_x + xi, -basketpositionlock.param.ilimit, basketpositionlock.param.ilimit);
	basketpositionlock.process.outx = basketpositionlock.process.gain * xp + basketpositionlock.process.itotal_x;

	float yp = basketpositionlock.param.p * yerror;
	float yi = ((fabs(yerror) < basketpositionlock.param.iend) && (fabs(yerror) > basketpositionlock.param.istart))?basketpositionlock.param.i * yerror:0;
	basketpositionlock.process.itotal_y = (fabs(yerror) < basketpositionlock.param.istart)?0:Limit(basketpositionlock.process.itotal_y + yi, -basketpositionlock.param.ilimit, basketpositionlock.param.ilimit);
	basketpositionlock.process.outy = basketpositionlock.process.gain * yp + basketpositionlock.process.itotal_y;
	
	// 请记住 第一项为front left 角速度
	float vnow = Limit(hypot(basketpositionlock.process.outx, basketpositionlock.process.outy), -basketpositionlock.param.outlimit,basketpositionlock.param.outlimit);
	float angle = atan2f(yerror, xerror) - ang2rad(basketlock.position.now_interp_vfield.r);

	Chassis_Velocity_Out(vnow * sin(angle),vnow * cos(angle),BasketAngleLock());	
	BsaketPoint_SelfLockAuto();
}
void BasketAngleLock_ParInit(void){  								 
		basketanglelock.param.p = 55;                       
		basketanglelock.param.i = 2;      
		basketanglelock.param.d = 0.005;                                       
		basketanglelock.param.ilimit = 1000;                
		basketanglelock.param.istart = 0.1;                 
		basketanglelock.param.iend = 6;                     
		basketanglelock.param.outlimit = 5000;              
		basketanglelock.param.accel_gain = 0.35;            
		basketanglelock.param.velocity_gain = 0.2;          
#ifdef Carbon_Car
	  basketlock.parameter.basketdis = 785;    
		basketlock.parameter.anglebetween_ladarandpole = 7;   
#else
		basketlock.parameter.basketdis = 1185;      
		basketlock.parameter.anglebetween_ladarandpole = 4; 
#endif	
		basketlock.parameter.siteinterp_gain = 0.5;         
		basketlock.parameter.angleinterp_gain = 0.4;        
		basketlock.parameter.ladar_offsetrad = 0;           
		basketlock.parameter.limitzoneanglimit = 60;				
}





