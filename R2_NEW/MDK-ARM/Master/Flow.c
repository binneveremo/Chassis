#include "Communication.h"
#include "Television.h"
#include "Fake_Rtos.h"
#include "Chassis.h"
#include "Can_Bsp.h"
#include "string.h"
#include "Basket.h"
#include "Flow.h"
#define Flow_End() (flow.flagof.end = true)
#define Shoot_Clear() (flow.flagof.R1_Shooted = false)


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
#define DRIBBLE_RUNPOINT true
struct Spot_t spot_dribble  = {.param.p = 5,.param.i = 1, .param.istart = 30,	.param.iend = 400,	.param.ilimit = 1000,	.param.outlimit = 12000, .param.fade_start = 280, .param.fade_end = 80, .param.lock_dis = 50};
struct dribble_t dribble = {.time.xuan_stamp = 1850,.time.wait = 650,.time.end = 2000,.parameter.dribble_front_velocity = 6300,.parameter.dribble_left_velocity = 300};
void Dribble_PointInit(void){
	dribble.parameter.dribble_point[0].x =  966;
	dribble.parameter.dribble_point[0].y = -2752;
	dribble.parameter.dribble_point[0].r =  0;
	
	dribble.parameter.dribble_point[1].x =  1125;
	dribble.parameter.dribble_point[1].y = -1814;
	dribble.parameter.dribble_point[1].r =  0;
}

void Dribble_Flow(void){
	static int pole_up_begin;
	int now = HAL_GetTick();
	switch(dribble.status){
		case dribble_runp:
			if(DRIBBLE_RUNPOINT == false)
				dribble.status = dribble_prepare;
			if(dribble.flagof.init == false)
				Dribble_PointInit(),dribble.flagof.init = true;
			if(PositionWithAngle_Lock(site.now,dribble.parameter.dribble_point[dribble.flagof.index],&spot_dribble,&cr_skill) == true)
				dribble.status = dribble_prepare;
		break;
		case dribble_prepare:
			Self_Lock_Out("Dribble");
			if(dribble.flagof.prepared == false){
				pole_up_begin = now;
				interact.defend_status = catch_ball;
				Tell_Yao_Xuan("dribble");
				dribble.flagof.prepared = true;
			}
			if(flow.flagof.pole_top == true){
				dribble.time.pole_uptime = now - pole_up_begin;
				dribble.time.begin = now;
				dribble.status = dribble_begin;
			}
		break;
		case dribble_begin:
			if(now - dribble.time.begin > dribble.time.wait)
				Chassis_Velocity_Out(dribble.parameter.dribble_left_velocity,dribble.parameter.dribble_front_velocity,0);
			else
				Self_Lock_Out("WaitDribble");
			if(now - dribble.time.begin > dribble.time.xuan_stamp || now - dribble.time.begin < (dribble.time.wait + 700))
				Tell_Yao_Xuan("catch");
			else 
				Tell_Yao_Xuan("catch");
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
	.target.point[5] = {.x = 3340,.y = -1923,.r = -23},
	.target.point[6] = {.x = 2512,.y = -2693,.r = -42},
	.target.point[7] = {.x = 2490,.y = -6800,.r = -61},

	.param.catch_advanced_dis[0] = 200,
	.param.catch_advanced_dis[1] = 200,
	.param.catch_advanced_dis[2] = 200,
	.param.catch_advanced_dis[3] = 200,
	.param.catch_advanced_dis[4] = 200,
	.param.catch_advanced_dis[5] = 200,
	.param.catch_advanced_dis[6] = 200,
	.param.catch_advanced_dis[7] = 200,

	.param.shoot_advanced_dis[0] = 600,
	.param.shoot_advanced_dis[1] = 2000,
	.param.shoot_advanced_dis[2] = 1500,
	.param.shoot_advanced_dis[3] = 1350,
	.param.shoot_advanced_dis[4] = 1600,
	.param.shoot_advanced_dis[5] = 1600,
	.param.shoot_advanced_dis[6] = 1600,
	.param.shoot_advanced_dis[7] = 1600,

	.param.lock_dis = 70,
	.param.lock_angle = 8,
	.param.net_defend_wait_time = 500,
	
	.param.spot[0] = {.param.p = 5.5,	.param.i = 0.5,	.param.istart = 6,	.param.iend = 300,	.param.ilimit = 1000,	.param.outlimit = 12300, .param.fade_start = 150, .param.fade_end = 100},
	.param.spot[1] = {.param.p = 6.2,	.param.i = 0.3,	.param.istart = 6,	.param.iend = 300,	.param.ilimit = 1000,	.param.outlimit = 12300, .param.fade_start = 150, .param.fade_end = 80},
	.param.spot[2] = {.param.p = 8.2,	.param.i = 1.0,	.param.istart = 6,	.param.iend = 300,	.param.ilimit = 1000,	.param.outlimit = 12800, .param.fade_start = 100, .param.fade_end = 80},
	.param.spot[3] = {.param.p = 6.2,	.param.i = 1.3,	.param.istart = 6,	.param.iend = 300,	.param.ilimit = 1000,	.param.outlimit = 12800, .param.fade_start = 150, .param.fade_end = 80},
	.param.spot[4] = {.param.p = 6.2,	.param.i = 1.1,	.param.istart = 6,	.param.iend = 300,	.param.ilimit = 1000,	.param.outlimit = 12800, .param.fade_start = 200, .param.fade_end = 80},
	.param.spot[5] = {.param.p = 6.2,	.param.i = 1.5,	.param.istart = 6,	.param.iend = 300,	.param.ilimit = 1000,	.param.outlimit = 12300, .param.fade_start = 200, .param.fade_end = 80},
	.param.spot[6] = {.param.p = 6.3,	.param.i = 1.2,	.param.istart = 6,	.param.iend = 300,	.param.ilimit = 1000,	.param.outlimit = 12300, .param.fade_start = 200, .param.fade_end = 80},
	.param.spot[7] = {.param.p = 8.7 ,  .param.i = 0, 	.param.istart = 6,	.param.iend = 300,	.param.ilimit = 1000,	.param.outlimit = 12500, .param.fade_start = 120, .param.fade_end = 80 ,.param.lock_dis = 600},
		
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
	static int change_begin;
 	static char last_success_times;
 	char index = skill.success_time % 8;      
 	switch(skill.status){
		case skill_netc:
//			Tell_Yao_Xuan("defend");
			skill.status = skill_runp;
		break;
 		case skill_runp:
 			if(skill.flagof.init == false)
 				for(unsigned char i = 0; i< 7;i++) Clear(skill.param.spot[i].process),skill.flagof.init = true;
 			Set_SendMode(&skill.target.point[index],"real",false);
 			PositionWithAngle_Lock(site.now,Merge_Point(skill.target.point[index],(struct Point){.r = shoot.deal.opposite_angle}),&skill.param.spot[index],&cr_skill);
// 			if((Point_Distance(site.now,skill.target.point[index]) < skill.param.catch_advanced_dis[index]) && (skill.flagof.net_catched == false))
// 				skill.flagof.net_catched = true,Tell_Yao_Xuan((index <= 8)?"catch":"defend");																																																																																																											
 			if((Point_Distance(site.now,skill.target.point[index]) < skill.param.lock_dis))
 				Self_Lock_Out("SkillFlow");
 			if((Point_Distance(site.now,skill.target.point[index]) < skill.param.shoot_advanced_dis[index]))
 				shoot.send.flagof.request = true;
 			else 
 				shoot.send.flagof.request = false;
			if(HAL_GetTick() - change_begin > 500)
				skill.flagof.change_flag = false;
 			if(last_success_times != skill.success_time)
 				last_success_times = skill.success_time,skill.status = clear,catchbegin_time = HAL_GetTick();
 		break;
 		case clear:
 			if(skill.success_time == 9)
 				Flow_End();
 			Clear(skill.flagof);
 			skill.status = skill_netc;
			skill.flagof.change_flag = true,change_begin = HAL_GetTick();
 		break;
 	}
 }

/*
1.控制网的状态
2.跑点
3.向R1发射请求
4.
*/
void Skill_Flow_Auto(void){
	char index = skill.success_time % 8;
	static int defend_begin;
	switch(skill.status){
		case skill_netc:
			Tell_Yao_Xuan("midcatch");
			defend_begin = HAL_GetTick();
			skill.status = skill_runp;
		break;
		case skill_runp:
			PositionWithAngle_Lock(site.now,Merge_Point(skill.target.point[index],(struct Point){.r = shoot.deal.opposite_angle}),&skill.param.spot[index],&cr_skill);
			if((Point_Distance(site.now,skill.target.point[index]) < skill.param.lock_dis))
				Self_Lock_Out("SkillFlow");
			if((skill.flagof.net_catched == false) && (HAL_GetTick() - defend_begin > skill.param.net_defend_wait_time))
				Tell_Yao_Xuan("catch"),skill.flagof.net_catched = true;
			Set_SendMode(&skill.target.point[index],"once",(bool)(Point_Distance(site.now,skill.target.point[index]) < skill.param.shoot_advanced_dis[index]));
			if(flow.flagof.receive_ball_bygyro == true)
				skill.success_time++,flow.flagof.receive_ball_bygyro = false,skill.status = clear;
		break;	
		case clear:
			if(skill.success_time == 8)
				Flow_End();
			Clear(skill.flagof);
			skill.status = skill_netc;
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

//////////////////舵轮与码盘的比例 1394.7
bool Expect_Position(struct Point start,struct Point end,float dir_ang,float rpm,struct Point * expect){
	float total_dis = Point_Distance(start,end);
	/////////遍历所有点位
	struct Point traverse;
	bool find = false;
	float shoot_distance,arrive_time,ball_time;	
#define step 10
#define threshold 20
	for(float dis = 0; dis < total_dis;dis += step){
		traverse.x = start.x + dis * cos(ang2rad(dir_ang));
		traverse.y = start.y + dis * sin(ang2rad(dir_ang));
		shoot_distance = Point_Distance(traverse,shoot.receive.shoot_pos);
		ball_time = Polynomial_4ExpectBallTotalTime(shoot_distance);
		arrive_time = dis / (rpm / 1394.7);
		if(fabs(ball_time - arrive_time) < threshold){
			expect->x = traverse.x;
			expect->y = traverse.y;
			find = true;
			break;
		}
	}
	return find;
#undef threshold
}
bool Velocity_Stable(void){
	
	return true;
}


void Trap_Flow(void){
	static float velocity_history[10];
	static float left_velocity_car,front_velocity_car,left_velocity_field,front_velocity_field;
	float variance = Variance_Cal(velocity_history,hypot(chassis.expect_status.front_velocity,chassis.expect_status.left_velocity),sizeof(velocity_history) / sizeof(float));
	switch(trap.status){
		case trap_prepare:
			trap.process.dir_ang = rad2ang(atan2f(trap.param.point[trap.success_time].y - site.now.y,trap.param.point[trap.success_time].x - site.now.x));
			left_velocity_field = trap.param.expect_velocity[trap.success_time] * sin(ang2rad(trap.process.dir_ang));
			front_velocity_field = trap.param.expect_velocity[trap.success_time] * cos(ang2rad(trap.process.dir_ang));			
			trap.status = trap_move;
		break;
		case trap_move:
			//场地速度转化到车体速度
			if(variance <= 0.01)
				trap.status = trap_stable;
			goto move;
		case trap_stable:
			if(Expect_Position(site.now,trap.param.point[trap.success_time],trap.process.dir_ang,trap.param.expect_velocity[trap.success_time],&trap.process.cusp) == true)
				Set_SendMode(&trap.process.cusp,"once",true);
			goto move;
			
		
		
			
			
			
			
		break;
		default:
			break;
	}
move:
	Field2Car(&front_velocity_car,&left_velocity_car,&front_velocity_field,&left_velocity_field);
	Chassis_Velocity_Out(left_velocity_car,front_velocity_car,Angle_Lock(site.now.r,shoot.deal.opposite_angle,&cr_skill));
}
void Trap_Flow_Test(struct Point end,float rpm,struct Point * expect){
	float angle_dir = atan2f(end.y - site.now.y,end.x - site.now.x);
	float left_velocity_car,front_velocity_car,left_velocity_field,front_velocity_field;
	left_velocity_field = rpm * sin(angle_dir);
	front_velocity_field = rpm * cos(angle_dir);	
	Field2Car(&front_velocity_car,&left_velocity_car,&front_velocity_field,&left_velocity_field);
	
	if(Expect_Position(site.now,end,Point_Angle(site.now,end),rpm,expect) == true)
		Set_SendMode(expect,"once",true);
	Chassis_Velocity_Out(left_velocity_car,front_velocity_car,Angle_Lock(site.now.r,shoot.deal.opposite_angle,&cr_skill));
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
			if(PositionWithAngle_Lock(basketlock.now.global,basketlock.target.global,&spot_basket,&cr_basket) == true)
				attack.status = attack_lock;
		break;
		case attack_lock:
			Self_Lock_Out("AttackLock");
			if(flow.flagof.R1_Shooted == true)	
				attack.status = attack_jump,Shoot_Clear();
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
	
	Zero(flow.type);
	
	//清除自动流程的枚举
	chassis.Control_Status = GamePad_Control;
	
	Set_SendMode(&site.now,"real",false);
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
	
	
	
	
}


void Receive_BallCheck(void){
	static float v_check;
	static bool v_check_flag;
	
#define NUM 4
#define Variance_Threshold 0.6
	static int last_time;
	static float gyro_history[NUM];
	v_check = Variance_Cal(gyro_history,site.car.xfilter.accel,NUM);
	v_check_flag = (v_check > Variance_Threshold)?true:false;
	
	if((v_check > Variance_Threshold) && (HAL_GetTick() - last_time > shoot.expect.ball_fly_time - 200) && (chassis.lock.flag == true) && (flow.flagof.R1_Shooted == true))
		shoot.deal.ball_fly_time = HAL_GetTick() - shoot.deal.shoot_begin,last_time = HAL_GetTick(),flow.flagof.receive_ball_bygyro = true,flow.flagof.R1_Shooted = false;
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






































