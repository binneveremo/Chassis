#include "Communication.h"
#include "Television.h"
#include "Fake_Rtos.h"
#include "Chassis.h"
#include "Can_Bsp.h"
#include "string.h"
#include "Basket.h"
#include "Flow.h"
#define Flow_End() (flow.flagof.end = true)



///////////////////////////////////////////////////////////////////////////////扣篮流程////////////////////////////////////////////////////////////////
struct dunk_t dunk;
void Dunk_Flow(void){
	Chassis_Basket_Noheader();
	if(dunk.flagof.confirm == true){
		if(dunk.flagof.net_ok == false)
			 ChooseCatchBall_StatusAuto(),dunk.flagof.net_ok = true;
		if(flow.flagof.R1_Shooted == true)
			Tell_Yao_Xuan("jump"),dunk.flagof.confirm = false,Flow_End();
	}
}






////////////////////////////////////////////////////////////////////////////回家流程//////////////////////////////////////////////////////////////////////
struct back_t back;
struct Point home_point = {                              
  .x = 500,
  .y = -650,
  .r = 0
};
void Back_Flow(void){
	PositionWithAngle_Lock(site.now,home_point,&spot_skill,&cr_skill);
	if(Point_Distance(site.now,site.target) < 500) 
		back.flagof.end = true,Self_Lock_Out("HomePoint");
}











///////////////////////////////////////////////////////////////////////运球流程//////////////////////////////////////////////////////////////////////////////
struct dribble_t dribble = {.time.xuan_stamp = 1850,.time.wait = 650,.time.end = 2000,.parameter.dribble_front_velocity = 6800,.parameter.dribble_left_velocity = -80,};
void Dribble_Flow(void){
	int now = HAL_GetTick();
	switch(dribble.status){
		case prepare:
			if(dribble.flagof.prepared == false){
				Tell_Yao_Xuan("dribble");
				interact.defend_status = (interact.defend_status == catch_ball)?predunk:interact.defend_status;
				interact.defend_status = ((interact.defend_status == initial) || (interact.defend_status == fold))?catch_ball:interact.defend_status;
				dribble.flagof.prepared = true;
			}
			if(flow.flagof.pole_top == true){
				dribble.time.begin = now;
				dribble.status = dribble_begin;
			}
		break;
		case dribble_begin:
			if(now - dribble.time.begin > dribble.time.wait)
				Chassis_Velocity_Out(dribble.parameter.dribble_left_velocity,dribble.parameter.dribble_front_velocity,0);
			else
				Self_Lock_Out("WaitDribble");
			if(now - dribble.time.begin > dribble.time.xuan_stamp || now - dribble.time.begin < (dribble.time.wait + 500))
				Tell_Yao_Xuan("catch");
			else 
				Tell_Yao_Xuan("fold");
			if(now - dribble.time.begin > dribble.time.end)
				Flow_End();
		break;
	}
}



























//////////////////////////////////////////////////////////////////////静态运球赛流程//////////////////////////////////////////////////////////////////////////////////////
struct skill_t skill = {
	.target.point[0] = {.x = 2500,.y = -5284,.r = -61},
	.target.point[1] = {.x = 3288,.y = -5978,.r = -58},
	.target.point[2] = {.x = 4946,.y = -6019,.r = -49},
	.target.point[3] = {.x = 4936,.y = -4024,.r = -42},
	.target.point[4] = {.x = 4959,.y = -1991,.r = -28},
	.target.point[5] = {.x = 3307,.y = -1923,.r = -23},
	.target.point[6] = {.x = 2512,.y = -2693,.r = -42},
	.target.point[7] = {.x = 2490,.y = -7000,.r = -61},

	.param.catch_advanced_dis[0] = 200,
	.param.catch_advanced_dis[1] = 200,
	.param.catch_advanced_dis[2] = 200,
	.param.catch_advanced_dis[3] = 200,
	.param.catch_advanced_dis[4] = 200,
	.param.catch_advanced_dis[5] = 200,
	.param.catch_advanced_dis[6] = 200,
	.param.catch_advanced_dis[7] = 200,

	.param.shoot_advanced_dis[0] = 50,
	.param.shoot_advanced_dis[1] = 50,
	.param.shoot_advanced_dis[2] = 50,
	.param.shoot_advanced_dis[3] = 50,
	.param.shoot_advanced_dis[4] = 50,
	.param.shoot_advanced_dis[5] = 50,
	.param.shoot_advanced_dis[6] = 50,
	.param.shoot_advanced_dis[7] = 50,

	.param.lock_dis = 100,
	.param.lock_angle = 8,
	
	.param.spot[0] = {.param.p = 5.2,	.param.i = 0,	.param.istart = 6,	.param.iend = 400,	.param.ilimit = 1000,	.param.outlimit = 11000, .param.fade_start = 150, .param.fade_end = 100},
	.param.spot[1] = {.param.p = 4.0,	.param.i = 0.3,	.param.istart = 6,	.param.iend = 400,	.param.ilimit = 1000,	.param.outlimit = 11000, .param.fade_start = 250, .param.fade_end = 100},
	.param.spot[2] = {.param.p = 4.1,	.param.i = 0.3,	.param.istart = 6,	.param.iend = 400,	.param.ilimit = 1000,	.param.outlimit = 11000, .param.fade_start = 180, .param.fade_end = 100},
	.param.spot[3] = {.param.p = 4.1,	.param.i = 0.5,	.param.istart = 6,	.param.iend = 400,	.param.ilimit = 1000,	.param.outlimit = 11000, .param.fade_start = 430, .param.fade_end = 100},
	.param.spot[4] = {.param.p = 4.1,	.param.i = 1.0,	.param.istart = 6,	.param.iend = 400,	.param.ilimit = 1000,	.param.outlimit = 11000, .param.fade_start = 200, .param.fade_end = 100},
	.param.spot[5] = {.param.p = 4.2,	.param.i = 1.3,	.param.istart = 6,	.param.iend = 400,	.param.ilimit = 1000,	.param.outlimit = 11000, .param.fade_start = 250, .param.fade_end = 100},
	.param.spot[6] = {.param.p = 4.2,	.param.i = 1.0,	.param.istart = 6,	.param.iend = 400,	.param.ilimit = 1000,	.param.outlimit = 11000, .param.fade_start = 250, .param.fade_end = 100},
	.param.spot[7] = {.param.p = 5.5, .param.i = 0, .param.istart = 6,	.param.iend = 400,	.param.ilimit = 1000,	.param.outlimit = 11000, .param.fade_start = 150, .param.fade_end = 100},
		
	.param.catch_delay_time[0] = 500,
	.param.catch_delay_time[1] = 500,
	.param.catch_delay_time[2] = 500,
	.param.catch_delay_time[3] = 500,
	.param.catch_delay_time[4] = 500,
	.param.catch_delay_time[5] = 500,
	.param.catch_delay_time[6] = 500,
	.param.catch_delay_time[7] = 500,
};
void Skill_Flow(void){
	static int catchbegin_time; //记录开始接住球的时间 方便放球延时
	static char last_success_times;
	char index = skill.success_time % 8;      
	switch(skill.status){
		case begin:
			PositionWithAngle_Lock(site.now,Merge_Point(skill.target.point[index],(struct Point){.r = shoot.deal.opposite_angle}),&skill.param.spot[index],&cr_skill);
			if((Point_Distance(site.now,skill.target.point[index]) < skill.param.catch_advanced_dis[index]) && (skill.flagof.net_catched == false))
				skill.flagof.net_catched = true,Tell_Yao_Xuan((index <= 7)?"catch":"defend");
			if((Point_Distance(site.now,skill.target.point[index]) < skill.param.lock_dis))
				Self_Lock_Out("SkillFlow"),shoot.send.flagof.request = true;
			else 
				shoot.send.flagof.request = false;
			if(last_success_times != skill.success_time)
				last_success_times = skill.success_time,skill.status = clear,catchbegin_time = HAL_GetTick();
		break;
		case clear:
			if(skill.success_time == 9){
				Flow_End();
				return;
			}
			Clear(skill.flagof);
			skill.status = begin;
		break;
	}
}














/////////////////////////////////////////////////////////////////////////////////动态技能挑战赛流程//////////////////////////////////////////////////////////////
struct Trap_t trap = {
	.param.expect_velocity[0] = 2000,
	.param.expect_velocity[1] = 2000,
	.param.expect_velocity[2] = 2000,
	.param.expect_velocity[3] = 2000,
	.param.expect_velocity[4] = 2000,
	.param.expect_velocity[5] = 2000,
	.param.expect_velocity[6] = 2000,
	
	
	.param.point[0] = {.x = 2609,.y = -5266,.r = -61},
	.param.point[1] = {.x = 3527,.y = -6060,.r = -58},
	.param.point[2] = {.x = 5140,.y = -6015,.r = -49},
	.param.point[3] = {.x = 5095,.y = -4030,.r = -42},
	.param.point[4] = {.x = 4908,.y = -1972,.r = -28},
	.param.point[5] = {.x = 3223,.y = -1970,.r = -23},
	.param.point[6] = {.x = 2470,.y = -2730,.r = -42},
	
};
struct Point Expect_Position(float left_rpm,float front_rpm){
	static struct Point expect;
	expect.x = site.now.x + front_rpm / 1394.7 * shoot.expect.ball_total_time;
	expect.y = site.now.y + left_rpm / 1394.7  * shoot.expect.ball_total_time;\
	return expect;
}
bool Velocity_Stable(void){
	
	return true;
}


void Trap_Flow(void){
	static float left_velocity_car,front_velocity_car,left_velocity_field,front_velocity_field;
	switch(trap.status){
		case trap_prepare:
			trap.process.dir_ang = atan2f(trap.param.point[trap.success_time].y - site.now.y,trap.param.point[trap.success_time].x - site.now.x);
			left_velocity_field = trap.param.expect_velocity[trap.success_time] * sin(ang2rad(trap.process.dir_ang));
			front_velocity_field = trap.param.expect_velocity[trap.success_time] * cos(ang2rad(trap.process.dir_ang));	
			trap.status = trap_move;
		break;
		case trap_move:
			//场地速度转化到车体速度
			Field2Car(&front_velocity_car,&left_velocity_car,&front_velocity_field,&left_velocity_field);
			Chassis_Velocity_Out(left_velocity_car,front_velocity_car,Angle_Lock(site.now.r,shoot.deal.opposite_angle,&cr_skill));
			if(trap.flagof.velocity_stable == true) 
				Set_SendMode(Expect_Position(left_velocity_field,front_velocity_field),"once",true);
			
			
			
		break;
		default:
			break;
	
	
	
	
	}
	
	
	
	
	
	
	
	
	
}















/////////////////////////////////////////////////////////////////////////突袭流程//////////////////////////////////////////////////////////////
struct attack_t attack;
void Attack_Flow(void){
	switch(attack.status){
		case attack_init:
			BasketPoint_Init();
			attack.status = attack_runp;
		break;
		case attack_runp:
			PositionWithAngle_Lock(basketlock.now.global,basketlock.target.global,&spot_basket,&cr_basket);
			//if(flow.flagof.R1_Shooted == true) attack.status = attack_jump;
		break;
		case attack_jump:
			Tell_Yao_Xuan("lift");
			Flow_End();
		break;
	}
}






















/////////////////////////////////////////////////////////////////////////////////流程整体控制////////////////////////////////////////////////////////////////
struct Flow flow;
void Back_GamePadControl(void){
	Zero(skill.status);
	Clear(skill.flagof);
	
	Zero(dunk.state);
	Clear(dunk.flagof);
	
	Zero(dribble.status);
	Clear(dribble.flagof);
	
	Zero(attack.status);
	
	Clear(back.flagof);
	Clear(flow.flagof);
	
	Zero(skill.success_time);
	
	//清除自动流程的枚举
	chassis.Control_Status = GamePad_Control;
}
/// @brief 自动流程
void Auto_Flow(void){
	switch(flow.type){
		case dribble_flow:
			Dribble_Flow();
		break;
		case dunk_flow:
			Dunk_Flow();
		break;
		case back_flow:
			Back_Flow();
		break;
		case skill_flow:
			Skill_Flow();
		break;
		case attack_flow:
			Attack_Flow();
		break;
		case trap_flow:
			Trap_Flow();
		break;
	}
	if(flow.flagof.end == true)
		Back_GamePadControl(),flow.flagof.end = false;
#undef Rocker_Move
}
void ControlStatus_Detect(void){
	if((vision.basketlock.online_flag == true) && (basketlock.position.ladar2basketdis < 1200) && (chassis.Control_Status == GamePad_Control) && (GamePad_Data.witch[7] == true))
		chassis.Control_Status = Auto_Control,flow.type = dribble_flow; 
}



float Variance,Variance_Threshold = 0.1;
int last_time;
void Receive_BallCheck(void){
#define NUM 4
	Variance = 0;
	static float gyro_history[NUM];
	for(unsigned char i = 0; i< NUM - 1; i++)
		gyro_history[i] = gyro_history[i + 1];
	gyro_history[NUM - 1] = site.car.xfilter.accel;
	/////计算方差/////////////
	float total = 0;
	for(unsigned char i = 0; i < NUM ; i++)
		total += gyro_history[i];
	float mean = total / NUM;
	for(unsigned char i = 0;i < NUM;i++)
		Variance += pow(fabs(gyro_history[i] - mean),2);
	Variance = Variance / NUM;
	if((Variance > Variance_Threshold) && (HAL_GetTick() - last_time > 1500) && (chassis.lock.flag == true) && (flow.flagof.R1_Shooted == true))
		shoot.deal.ball_fly_time = HAL_GetTick() - shoot.deal.shoot_begin,last_time = HAL_GetTick(),flow.flagof.receive_ball_bygyro = true;
}
void Send_ReceiveBallMessage(void){
	for(unsigned char i = 0; i < 5; i++){
		Send_Put_Data(0,shoot.deal.ball_fly_time);
		Send_Put_Data(1,shoot.deal.distance);
		Send_Put_Data(2,shoot.receive.shoot_rpm);
		Send_Put_Data(3,shoot.receive.shoot_incar_time);
		Send_Float_Data(4);
		osDelay(10);
	}
}
float Variance_Check(float data,char num,float threshold,char * type){
	static float history[50];
	for(unsigned char i = 0; i< num - 1; i++)
		history[i] = history[i + 1];
	history[NUM - 1] = data;
	float total = 0,variance = 0;
	for(unsigned char i = 0; i < NUM ; i++)
		total += history[i];
	float mean = total / NUM;
	for(unsigned char i = 0;i < NUM;i++)
		variance += pow(fabs(history[i] - mean),2);
	variance = variance / NUM;
	
	if(strcmp(type,"check") == 0)
		return (bool)((variance < threshold)?true:false);
	else 
		return variance;
}





































