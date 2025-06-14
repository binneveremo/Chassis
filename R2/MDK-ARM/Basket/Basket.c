#include "Basket.h"
#include "Flow.h"

struct Basket_Lock_t basketlock = {
	.parameter.basketdis = 825,
	.parameter.anglebetween_ladarandpole = 1,
	.parameter.siteinterp_gain = 0.5,
	.parameter.angleinterp_gain = 0.4,
	.parameter.ladar_offsetrad = 0,
	.parameter.limitzoneanglimit = 160,
};
struct Point opposite_basket_point = {
	.x = 12050,
	.y = -4000,
	.r = 0};
struct Point self_basket_point = {
	.x = 1500,
	.y = -4000,
	.r = 0};
// 选择篮筐定位方式
#define GLOBAL_BASKETLOCK_POSITION false
#define PARTIAL_BASKETLOCK_POSITION !GLOBAL_BASKETLOCK_POSITION

#define GLOBAL_BASKETLOCK_ANGLE false
#define PARTIAL_BASKETLOCK_ANGLE !GLOBAL_BASKETLOCK_ANGLE
//////////////////////根据马盘插帧的定位计算篮框的位置以及相对于自身的距离
void BasketPositionCal_AccordingVision(float dt)
{
	// 2ms计算一次 包括篮筐相对于自身坐标的获取 以及对定位进行插帧
	basketlock.position.ladar2basketx = vision.visual.basket_visual.x - vision.visual.carzero_visualinterp.x;
	basketlock.position.ladar2baskety = vision.visual.basket_visual.y - vision.visual.carzero_visualinterp.y;
	basketlock.position.ladar2basketangle = atan2f(basketlock.position.ladar2baskety, basketlock.position.ladar2basketx);
	basketlock.position.ladar2basketdis = hypot(basketlock.position.ladar2basketx, basketlock.position.ladar2baskety);
	
	basketlock.now.partial.x += site.field.vx_enc * dt;
	basketlock.now.partial.y += site.field.vy_enc * dt;
	basketlock.now.partial.r = site.gyro.r;
}
///////////////////////返回的是锁篮筐角度PID的输出
void BasketPoint_Init(char * flag)
{
	if(*flag == true)
		return;
	basketlock.target.global.r = Limit(rad2ang(atan2f(basketlock.position.ladar2baskety, basketlock.position.ladar2basketx)), -basketlock.parameter.limitzoneanglimit, basketlock.parameter.limitzoneanglimit);
	basketlock.target.global.x = vision.visual.basket_visual.x - basketlock.parameter.basketdis * cos(ang2rad(basketlock.target.global.r));
	basketlock.target.global.y = vision.visual.basket_visual.y - basketlock.parameter.basketdis * sin(ang2rad(basketlock.target.global.r));
	basketlock.target.global.r += basketlock.parameter.anglebetween_ladarandpole;

	//为了不清空当前的角度
	Zero(basketlock.now.partial.x);
	Zero(basketlock.now.partial.y);
	
	basketlock.target.partial.x = basketlock.target.global.x - vision.visual.carzero_visualinterp.x;
	basketlock.target.partial.y = basketlock.target.global.y - vision.visual.carzero_visualinterp.y;
	basketlock.target.partial.r = basketlock.target.global.r;
	*flag = true;                             
}
void BasketPosition_Lock(void){
	/*
	无论哪种情况 无论是否全局坐标系 都会选择陀螺仪角度
	无论全局坐标系还是局部坐标系 目标角度都是一定的 但是i自身角度是不一样的 全局坐标系的自身定位就是雷达的定位，角度插值 局部坐标系的定位就是陀螺仪的角度
	*/
#if GLOBAL_BASKETLOCK_POSITION
	PositionWithAngle_Lock(Merge_Point(basketlock.now.global,basketlock.now.partial),basketlock.target.global, &spot_basket, &cr_basket);
#else
	PositionWithAngle_Lock(basketlock.now.partial,basketlock.target.partial, &spot_basket, &cr_basket);
#endif

}



