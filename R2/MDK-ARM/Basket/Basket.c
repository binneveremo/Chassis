#include "Basket.h"
#include "Flow.h"

struct BasketAngle_Lock_t basketanglelock = {
	.param.p = 55,
	.param.d = 0.05,
	.param.i = 2,
	.param.ilimit = 1000,
	.param.istart = 0.1,
	.param.iend = 6,
	.param.outlimit = 5000,
	.param.accel_gain = 0.35,
	.param.velocity_gain = 0.2,
};
struct BasketPosition_Lock_t basketpositionlock = {
	.param.p = 3.5,
	.param.i = 0.5,
	.param.istart = -1,
	.param.fade_start = 580,
	.param.fade_end = 170,
	.param.iend = 500,
	.param.ilimit = 700,
	.param.outlimit = 6800,
};
struct Basket_Lock_t basketlock = {
	.parameter.basketdis = 825,
	.parameter.anglebetween_ladarandpole = 4,
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
#define GLOBAL_BASKETLOCK_POSITION true
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
}
///////////////////////返回的是锁篮筐角度PID的输出
float BasketAngleLock(void)
{
#if GLOBAL_BASKETLOCK_ANGLE
	basketanglelock.progress.error = basketpositionlock.target.global.r - vision.visual.carzero_visualinterp.r;
#elif PARTIAL_BASKETLOCK_ANGLE
	basketanglelock.progress.error = basketpositionlock.target.partial.r - basketpositionlock.now.partial.r;
#endif
	float out;
	float p = basketanglelock.param.p * basketanglelock.progress.error;
	basketanglelock.progress.gain = Limit(basketanglelock.param.velocity_gain * site.car.velocity_totalenc + basketanglelock.param.accel_gain * site.car.accel_totalgyro, 1, 4);
	float d = basketanglelock.param.d * site.gyro.omiga;
	// 完成增益的衰减
	basketanglelock.progress.gain *= Normalize_Pow(basketanglelock.param.istart, basketanglelock.param.iend, basketanglelock.progress.error, 2);
	// 完成积分分离
	basketanglelock.progress.itotal = ((fabs(basketanglelock.progress.error) > basketanglelock.param.istart) && (fabs(basketanglelock.progress.error) < basketanglelock.param.iend)) ? Limit(basketanglelock.progress.itotal + basketanglelock.param.i * basketanglelock.progress.error, -basketanglelock.param.ilimit, basketanglelock.param.ilimit) : 0;
	out = Limit(basketanglelock.progress.gain * p + basketanglelock.progress.itotal, -basketanglelock.param.outlimit, basketanglelock.param.outlimit);
	return out;
}

void BasketPoint_Init(void)
{
	if(dunk.flagof.init == true)
		return;
	basketpositionlock.target.global.r = Limit(rad2ang(atan2f(basketlock.position.ladar2baskety, basketlock.position.ladar2basketx)), -basketlock.parameter.limitzoneanglimit, basketlock.parameter.limitzoneanglimit);
	basketpositionlock.target.global.x = vision.visual.basket_visual.x - basketlock.parameter.basketdis * cos(ang2rad(basketpositionlock.target.global.r));
	basketpositionlock.target.global.y = vision.visual.basket_visual.y - basketlock.parameter.basketdis * sin(ang2rad(basketpositionlock.target.global.r));
	basketpositionlock.target.global.r += basketlock.parameter.anglebetween_ladarandpole;
#if REPEAT
	basketlock.position.basket_target_visual.r += (basketlock.position.basket_target_visual.r > 0) ? basketlock.parameter.anglebetween_ladarandpole : -basketlock.parameter.anglebetween_ladarandpole;
	char repeat = (basketlock.position.basket_target_visual.x > vision.basket.basket_visual.x) ? 1 : 0;
	basketlock.parameter.anglebetween_ladarandpole : basketlock.position.basket_target_visual.x = (repeat == 1) ? vision.basket.basket_visual.x + basketlock.parameter.basketdis * cos(basketlock.position.ladar2basketangle) : basketlock.position.basket_target_visual.x;
	basketlock.position.basket_target_visual.r = (repeat == 1) ? NormalizeAng_Single(180 - rad2ang(basketlock.position.ladar2basketangle) - basketlock.parameter.anglebetween_ladarandpole) : basketlock.position.basket_target_visual.r;
#endif
	Clear(basketpositionlock.process);
	Clear(basketpositionlock.flagof);

	// 切换到马盘坐标系
	Clear(basketpositionlock.now.partial);
	basketpositionlock.target.partial.x = basketpositionlock.target.global.x - vision.visual.carzero_visualinterp.x;
	basketpositionlock.target.partial.y = basketpositionlock.target.global.y - vision.visual.carzero_visualinterp.y;
	basketpositionlock.target.partial.r = basketpositionlock.target.global.r - vision.visual.carzero_visualinterp.r;
	dunk.flagof.init = true;                             
}
void BsaketPoint_SelfLockAuto(void)
{
	/////////////////////////////判断是否距离目标点更远了 收集历史数据 如果比历史的三个点都远 就认为更远了//////////////////
#define check_num 4
	static float dis_history[check_num];
	for (int i = check_num - 1; i > 0; i--)
		dis_history[i] = dis_history[i - 1];
	char lagernum = (char)NULL;
#if GLOBAL_BASKETLOCK_POSITION
	float disnow = Point_Distance(basketpositionlock.target.global, basketpositionlock.now.global);
#elif PARTIAL_BASKETLOCK_POSITION
	float disnow = Point_Distance(basketpositionlock.target.partial, basketpositionlock.now.partial);
#endif
	for (int i = 0; i < check_num; i++)
		lagernum = (disnow > dis_history[i]) ? lagernum + 1 : lagernum;
	dis_history[0] = disnow;
	// 距离目标点变远
	static int lock_times;
#define futher_death_bond 50
#define death_bond 20
	basketpositionlock.flagof.lock_flag = ((fabs(basketanglelock.progress.error) < 1.2) && (((lagernum >= check_num - 1) && (basketpositionlock.process.error < futher_death_bond)) || (basketpositionlock.process.error < death_bond))) ? 1 : basketpositionlock.flagof.lock_flag;

	if (basketpositionlock.flagof.lock_flag == 1)
		Self_Lock_Out("BasketNear");
}
void BasketPositionLock()
{
#if GLOBAL_BASKETLOCK_POSITION
	float xerror = basketpositionlock.target.global.x - vision.visual.carzero_visualinterp.x;
	float yerror = basketpositionlock.target.global.y - vision.visual.carzero_visualinterp.y;
#elif PARTIAL_BASKETLOCK_POSITION
	float xerror = basketpositionlock.target.partial.x - basketpositionlock.now.partial.x;
	float yerror = basketpositionlock.target.partial.y - basketpositionlock.now.partial.y;
#endif
	basketpositionlock.process.error = hypot(xerror, yerror);
	basketpositionlock.process.gain = Normalize_Pow(basketpositionlock.param.fade_end, basketpositionlock.param.fade_start, basketpositionlock.process.error, 2);

	float xp = basketpositionlock.param.p * xerror;
	float xi = ((fabs(xerror) < basketpositionlock.param.iend) && (fabs(xerror) > basketpositionlock.param.istart)) ? basketpositionlock.param.i * xerror : 0;
	basketpositionlock.process.itotal_x = (fabs(xerror) < basketpositionlock.param.istart) ? 0 : Limit(basketpositionlock.process.itotal_x + xi, -basketpositionlock.param.ilimit, basketpositionlock.param.ilimit);
	basketpositionlock.process.outx = basketpositionlock.process.gain * xp + basketpositionlock.process.itotal_x;

	float yp = basketpositionlock.param.p * yerror;
	float yi = ((fabs(yerror) < basketpositionlock.param.iend) && (fabs(yerror) > basketpositionlock.param.istart)) ? basketpositionlock.param.i * yerror : 0;
	basketpositionlock.process.itotal_y = (fabs(yerror) < basketpositionlock.param.istart) ? 0 : Limit(basketpositionlock.process.itotal_y + yi, -basketpositionlock.param.ilimit, basketpositionlock.param.ilimit);
	basketpositionlock.process.outy = basketpositionlock.process.gain * yp + basketpositionlock.process.itotal_y;

	// 请记住 第一项为front left 角速度
	float vnow = Limit(hypot(basketpositionlock.process.outx, basketpositionlock.process.outy), -basketpositionlock.param.outlimit, basketpositionlock.param.outlimit);
	float angle = atan2f(yerror, xerror) - ang2rad(vision.visual.carzero_visualinterp.r);

	Chassis_Velocity_Out(vnow * sin(angle), vnow * cos(angle), BasketAngleLock());
	BsaketPoint_SelfLockAuto();
}