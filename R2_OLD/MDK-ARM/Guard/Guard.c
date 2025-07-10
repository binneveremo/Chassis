#include "Guard.h"

#include "Guard.h"

struct Guard_Trace trace = {	.p = 13,	.brake_distance = 430,	.break_angle = 150,	.outlimit = 9000,};
void Guard_Trace_Run(void)
{
	float xerror = site.target.x - site.now.x;
	float yerror = site.target.y - site.now.y;
	float xp = trace.p * xerror;
	trace.outx = xp;
	float yp = trace.p * yerror;
	trace.outy = yp;
	// 请记住 第一项为front left 角速度
	float vnow = Limit(hypot(trace.outx, trace.outy), -trace.outlimit, trace.outlimit);
	float point_angle = atan2f(yerror, xerror) - ang2rad(site.now.r);
	float front = vnow * cos(point_angle);
	float left = vnow * sin(point_angle);
	// 限制加速步长
	static float left_out, front_out;
	left_out += Limit(left - left_out, -55, 55);
	front_out += Limit(front - front_out, -55, 55);
	// 方向反向的话先刹车再反向加速
	if (hypot(xerror, yerror) <= trace.brake_distance)
		Self_Lock_Out("GuardNear");
}
