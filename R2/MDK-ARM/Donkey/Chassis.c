#include "mngCommu.h"
#include "Chassis.h"
#include "Basket.h"
#include "Flow.h"

// 跑点以及底盘状态的结构体
struct Chassis chassis;
/////////////////////////////////////////////舵轮输出相关
void VectorWheel_SetSpeed(void)
{
	VESC_SetRPM(chassis.motor.drive[front_wheel].dir * chassis.motor.drive[front_wheel].rpm, front_drive_id);
	VESC_SetRPM(chassis.motor.drive[left_wheel].dir * chassis.motor.drive[left_wheel].rpm, left_drive_id);
	VESC_SetRPM(chassis.motor.drive[right_wheel].dir * chassis.motor.drive[right_wheel].rpm, right_drive_id);
	VESC_SetRPM(chassis.motor.drive[behind_wheel].dir * chassis.motor.drive[behind_wheel].rpm, behind_drive_id);
}
void VectorWheel_SetAngle(void)
{
	// Motor_Control_byFDCN(id,力矩, 速度 , 位置, 模式 , 使能, p, d, )
	Motor_Control_byFDCN(front_turn_send_id, 2.5, 30, chassis.motor.turn[front_wheel].target_angle + chassis.motor.turn[front_wheel].offset_angle, 2, 1, chassis.motor.turn[front_wheel].param.p, chassis.motor.turn[front_wheel].param.d, &hfdcan1);
	Motor_Control_byFDCN(left_turn_send_id, 2.5, 30,  chassis.motor.turn[left_wheel].target_angle + chassis.motor.turn[left_wheel].offset_angle, 2, 1, chassis.motor.turn[left_wheel].param.p, chassis.motor.turn[left_wheel].param.d, &hfdcan1);
	Motor_Control_byFDCN(right_turn_send_id, 2.5, 30, chassis.motor.turn[right_wheel].target_angle + chassis.motor.turn[right_wheel].offset_angle, 2, 1, chassis.motor.turn[right_wheel].param.p, chassis.motor.turn[right_wheel].param.d, &hfdcan1);
	Motor_Control_byFDCN(behind_turn_send_id, 2.5, 30, chassis.motor.turn[behind_wheel].target_angle + chassis.motor.turn[behind_wheel].offset_angle, 2, 1, chassis.motor.turn[behind_wheel].param.p, chassis.motor.turn[behind_wheel].param.d, &hfdcan1);
}
void Min_Angle_Cal(struct HO7213 *turn, struct VESC *drive, float target)
{
	float delta = turn->angle_now - target;
	int k = (int)floor((delta + 100.00) / 180.0f);
	float target_temp = target + 180.0f * k;
	int cycles = (int)(fabs(target_temp - target) / 180.0f);
	drive->dir = (cycles % 2 == 1) ? -1 : 1;
	turn->target_angle = target_temp;
}
// 只是计算出来每个电机期望的速度，并还没有输出
void Chassis_Velocity_Out(float left, float front, float anticlock)
{
	// 请记住 默认Y轴正方向为0度 也就是atan计算出来的角度是 相对于y轴的角度
	// vx是left vy是front
	chassis.motor.drive[front_wheel].front = front;
	chassis.motor.drive[front_wheel].left = left + anticlock * 1.03;

	chassis.motor.drive[left_wheel].front = front - anticlock * 0.96547;
	chassis.motor.drive[left_wheel].left = left - anticlock * 0.2605;

	chassis.motor.drive[right_wheel].front = front + anticlock * 0.96547;
	chassis.motor.drive[right_wheel].left = left - anticlock * 0.2605;

	chassis.motor.drive[behind_wheel].front = front;
	chassis.motor.drive[behind_wheel].left = left - anticlock * 1.03;

	chassis.motor.drive[front_wheel].rpm = hypot(chassis.motor.drive[front_wheel].left, chassis.motor.drive[front_wheel].front);
	chassis.motor.drive[left_wheel].rpm = hypot(chassis.motor.drive[left_wheel].left, chassis.motor.drive[left_wheel].front);
	chassis.motor.drive[right_wheel].rpm = hypot(chassis.motor.drive[right_wheel].left, chassis.motor.drive[right_wheel].front);
	chassis.motor.drive[behind_wheel].rpm = hypot(chassis.motor.drive[behind_wheel].left, chassis.motor.drive[behind_wheel].front);

	Min_Angle_Cal(&chassis.motor.turn[front_wheel], &chassis.motor.drive[front_wheel], rad2ang(atan2f(chassis.motor.drive[front_wheel].left, chassis.motor.drive[front_wheel].front)));
	Min_Angle_Cal(&chassis.motor.turn[left_wheel], &chassis.motor.drive[left_wheel], rad2ang(atan2f(chassis.motor.drive[left_wheel].left, chassis.motor.drive[left_wheel].front)));
	Min_Angle_Cal(&chassis.motor.turn[right_wheel], &chassis.motor.drive[right_wheel], rad2ang(atan2f(chassis.motor.drive[right_wheel].left, chassis.motor.drive[right_wheel].front)));
	Min_Angle_Cal(&chassis.motor.turn[behind_wheel], &chassis.motor.drive[behind_wheel], rad2ang(atan2f(chassis.motor.drive[behind_wheel].left, chassis.motor.drive[behind_wheel].front)));
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////手柄控制相关//////////////////////////////////////////////////////////////////////////////
void GamePad_Velocity_Control(void)
{
	float rocker_x, rocker_y, rocker_r;
	rocker_x = -GamePad_Data.rocker[0];
	rocker_y = GamePad_Data.rocker[1];
	rocker_r = -GamePad_Data.rocker[2];
	// 陀螺仪逆时针为正
	float Rocker_GainT = chassis.flagof.gamepad.slow ? 12 : (chassis.flagof.gamepad.accel ? 127 : 68);
	float Rocker_GainR = chassis.flagof.gamepad.slow ? 5 : (chassis.flagof.gamepad.accel ? 35 : 23);
	float r = (chassis.flagof.gamepad.inverse == 1) ? site.now.r + 90 : site.now.r;
	float y = (float)rocker_y * cos(ang2rad(r)) + rocker_x * sin(ang2rad(r));
	float x = (float)rocker_x * cos(ang2rad(r)) - rocker_y * sin(ang2rad(r));
	//计算锁的方向
	float rout;
	switch(chassis.opposite){
		case none:
			rout = Rocker_GainR * rocker_r;
		break;
		case R1:
			rout = Correct_Angle(send.R1_Exchange.pos.r);
		break;
		case self_basket:
			rout = Correct_Angle(basketlock.protectselfbasket_angle);
		break;
		case oppo_basket:
			rout = BasketAngleLock();
		 break;
		case forward:
			rout = Correct_Angle(NONE);
		break;
	}
	if(chassis.flagof.gamepad.noheader)
		Chassis_Velocity_Out(x * Rocker_GainT, y * Rocker_GainT, rout);
	else 
		Chassis_Velocity_Out(rocker_x * Rocker_GainT,rocker_y * Rocker_GainT, rout);
}
bool TurnMotor_InPosition(void)
{
#define TurnMotorDiffAngle(x) (fabs(chassis.motor.turn[x].target_angle - chassis.motor.turn[x].angle_now))
	if (TurnMotorDiffAngle(front_wheel) > 3)
		return false;
	if (TurnMotorDiffAngle(behind_wheel) > 3)
		return false;
	if (TurnMotorDiffAngle(left_wheel) > 3)
		return false;
	if (TurnMotorDiffAngle(right_wheel) > 3)
		return false;
	return true;
}
bool TurnMotor_InTurnPosition(void)
{
	Chassis_Velocity_Out(0, 0, 1);
	return TurnMotor_InPosition();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////跑点相关///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////三种跑法：1.位置PID 2.速度PID 3.位置PID+速度PID////////////////////////////////////////////////////////////////////////////////////
struct correct_angle_t correct_angle = {.p = 35, .d = 0.03, .i = 0.6, .ilimit = 900, .istart = 1, .iend = 10, .outlimit = 6000, .accel_gain = 1.4, .velocity_gain = 0.2,};
float Correct_Angle(float target)
{
	float error = NormalizeAng_Single(target - site.now.r);
	float out;
	float p = correct_angle.p * error;
	float gain = Limit(correct_angle.velocity_gain * site.car.velocity_totalenc + correct_angle.accel_gain * site.car.accel_totalgyro, 1, 4);

	gain = (fabs(error) < correct_angle.istart) ? Limit(fabs(error / correct_angle.istart), 0.5, 1) : 1;
	correct_angle.itotal *= gain;
	correct_angle.itotal = ((fabs(error) > correct_angle.istart) && (fabs(error) < correct_angle.iend)) ? Limit(correct_angle.itotal + correct_angle.i * error, -correct_angle.ilimit, correct_angle.ilimit) : correct_angle.itotal;
	out = Limit(gain * p + correct_angle.itotal - correct_angle.d*site.gyro.omiga, -correct_angle.outlimit, correct_angle.outlimit);
	return out;
}

/////////////////////////////////////////4.位置闭环单向PID
struct Spot_t spot_far = {	.param.p = 3,	.param.i = 2,	.param.istart = 6,	.param.iend = 400,	.param.ilimit = 1000,	.param.outlimit = 16000,	.process.brake_distance = 600,	.param.brake_percent = 0.3,	.param.brake_gain = 0.05, .param.brake_mindis = 600};
struct Spot_t spot_near = {	.param.p = 11,.param.i = 2,	.param.istart = 200,.param.iend = 1000,	.param.ilimit = 2000,	.param.outlimit = 18000,	.process.brake_distance = 600,	.param.brake_percent = 0.77,	.param.brake_gain = 0.05, .param.brake_mindis = 300};
//////////////////////跑点的速度限制       根据最大速度最大速度 10000  那么就会限制刹车距离为 spot.brake_percent * 10000
void Position_With_Mark_PID_Run(char * type){
	static struct Spot_t spot; 
	if(strcmp(type,"far") == 0)
		Copy(spot.param,spot_far);
	else 
		Copy(spot.param,spot_near);
	static struct Point last;
	if (Point_Distance(last, site.target) > 100)
	{
		spot.process.total_dis = Point_Distance(site.target, site.now);
		spot.process.brake_distance = Limit(spot.param.brake_percent * spot.process.total_dis, spot.param.brake_mindis, spot.param.outlimit * spot.param.brake_percent);
		memcpy(&last, &site.target, sizeof(last));
	}
	float xerror = site.target.x - site.now.x;
	float yerror = site.target.y - site.now.y;
	float rerror = site.target.r - site.now.r;

	spot.process.gain = (hypot(xerror, yerror) > spot.process.brake_distance) ? 1 : spot.param.brake_gain;
	float outlimit = spot.param.outlimit;
	float ilimit = spot.param.ilimit;

	float xp = spot.param.p * xerror;
	float xi = ((fabs(xerror) < spot.param.iend) && (fabs(xerror) > spot.param.istart)) ? spot.param.i * xerror : 0;
	spot.process.itotal_x = (fabs(xerror) < spot.param.istart) ? 0 : Limit(spot.process.itotal_x + xi, -ilimit, ilimit);
	spot.process.outx = spot.process.gain * xp + spot.process.itotal_x;

	float yp = spot.param.p * yerror;
	float yi = ((fabs(yerror) < spot.param.iend) && (fabs(yerror) > spot.param.istart)) ? spot.param.i * yerror : 0;
	spot.process.itotal_y = (fabs(yerror) < spot.param.istart) ? 0 : Limit(spot.process.itotal_y + yi, -ilimit, ilimit);
	spot.process.outy = spot.process.gain * yp + spot.process.itotal_y;

	// 请记住 第一项为front left 角速度
	float vnow = Limit(hypot(spot.process.outx, spot.process.outy), -outlimit, outlimit);
	float angle = atan2f(yerror, xerror) - ang2rad(site.now.r);

	Chassis_Velocity_Out(vnow * sin(angle), vnow * cos(angle), Correct_Angle(site.target.r));
}

void Self_Lock_Out(char *lock_reason){
	Min_Angle_Cal(&chassis.motor.turn[front_wheel], &chassis.motor.drive[front_wheel], 0);
	Min_Angle_Cal(&chassis.motor.turn[left_wheel], &chassis.motor.drive[left_wheel], -83);
	Min_Angle_Cal(&chassis.motor.turn[right_wheel], &chassis.motor.drive[right_wheel], 83);
	Min_Angle_Cal(&chassis.motor.turn[behind_wheel], &chassis.motor.drive[behind_wheel], 0);
	for (int i = 0; i < VESC_NUM; i++)
		chassis.motor.drive[i].rpm = 0;
	if (strcmp(chassis.lock.reason, lock_reason))
		memcpy(chassis.lock.reason, lock_reason, strlen(lock_reason));
}
// 自动自锁函数
void Self_Lock_Auto(void)
{
	memset(chassis.lock.reason, (char)NULL, sizeof(chassis.lock.reason));
	char flag = ((fabs((float)GamePad_Data.rocker[0]) <= 1) && (fabs((float)GamePad_Data.rocker[1]) <= 1) && (fabs(GamePad_Data.rocker[2]) <= 5)) ? 1 : 0;
	switch (chassis.Control_Status)
	{
		case GamePad_Control:
			if(flag)	Self_Lock_Out("GamePadZero");
			break;
		default:
			break;
	}
	chassis.lock.flag = (chassis.lock.reason[NONE] != NONE) ? true : false;
	if(GamepadLostConnection)
		Self_Lock_Out("GamePadLoss");
}
/////////////////////////////////////////////////////////////////////////////////////转向电机解算//////////////////////////////////////////////////////////////////////////
void Turn_Motor_Decode(int id, unsigned char *data)
{
	if (id == front_turn_receive_id)
		chassis.motor.turn[front_wheel].angle_now = HO7213_Decode_for_FDC(data) - chassis.motor.turn[front_wheel].offset_angle, chassis.motor.turn[front_wheel].online_flag = true;
	else if (id == right_turn_receive_id)
		chassis.motor.turn[right_wheel].angle_now = HO7213_Decode_for_FDC(data) - chassis.motor.turn[right_wheel].offset_angle, chassis.motor.turn[right_wheel].online_flag = true;
	else if (id == left_turn_receive_id)
		chassis.motor.turn[left_wheel].angle_now = HO7213_Decode_for_FDC(data) - chassis.motor.turn[left_wheel].offset_angle, chassis.motor.turn[left_wheel].online_flag = true;
	else if (id == behind_turn_receive_id)
		chassis.motor.turn[behind_wheel].angle_now = HO7213_Decode_for_FDC(data) - chassis.motor.turn[behind_wheel].offset_angle, chassis.motor.turn[behind_wheel].online_flag = true;
	else
		return;
}
void Get_VESC_Data(int id, unsigned char *data)
{
#define ID_Decode(id) (id & 0x00FF)
	char ID = ID_Decode(id);
	switch (ID)
	{
	case front_drive_id:
		chassis.motor.drive[front_wheel].online_flag = true;
		break;
	case left_drive_id:
		chassis.motor.drive[left_wheel].online_flag = true;
		break;
	case right_drive_id:
		chassis.motor.drive[right_wheel].online_flag = true;
		break;
	case behind_drive_id:
		chassis.motor.drive[behind_wheel].online_flag = true;
		break;
	default:
		break;
	}
}