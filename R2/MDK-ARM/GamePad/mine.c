#include "Television.h"
#include "mngCommu.h"
#include "Chassis.h"
#include "Encoder.h"
#include "Correct.h"
#include "Gyro.h"
#include "mine.h"
#include "Nrf.h"

#define DebugPage_Change(x)	(GamePad_Data.Debug_Page = (GamePad_Data.Debug_Page >= 5)?0:GamePad_Data.Debug_Page + x)

struct Game_Pad_Data GamePad_Data;


void Get_GamePad_Data(void){
	for(int i = 0; i< 4;i++)
		GamePad_Data.rocker[i] = get_rocker(i);
	for(int i = 0; i< 22;i++)
		GamePad_Data.key[i] = get_fk_state(i);
	for(int i = 0; i< 10;i++)
	   GamePad_Data.witch[i] = get_sw_state(i);
} 
void GamePad_Data_Cla(void){ 
	//拨码状态切换
	Get_GamePad_Data();
	
	if(GamePad_Data.witch[5] == 1)
		Self_Lock_Out("SafeMode");
#define sudo GamePad_Data.key[4]
#define flow_begin (chassis.Control_Status = Auto_Control)
	if((sudo == 1) && (GamePad_Data.key[20] == 1))
		flow_begin, flow.type = dribble_flow;
	else if((sudo == 1) && (GamePad_Data.key[18] == 1))
		flow_begin, flow.type = dunk_flow;
	else if((sudo == 1) && (GamePad_Data.key[14] == 1))
		flow_begin, flow.type = back_flow;
#undef sudo
	if(((GamePad_Data.witch[8] == 1) || (GamePad_Data.key[21] == 1)) && (chassis.Control_Status == Auto_Control))
		Back_GamePadControl();

	//手柄控制相关标志位
	chassis.flagof.gamepad.standard = GamePad_Data.witch[1];
	chassis.flagof.gamepad.noheader = !chassis.flagof.gamepad.standard;
	chassis.flagof.gamepad.inverse = GamePad_Data.witch[0];
	chassis.flagof.gamepad.accel  = GamePad_Data.key[21];
	chassis.flagof.gamepad.slow  = GamePad_Data.key[8];
	//通信方面
	if(GamePad_Data.key[5] == 1)
 		Tell_Yao_Xuan("fold");
	if(GamePad_Data.key[7] == 1)
 		Tell_Yao_Xuan("catch");
	if(GamePad_Data.key[3] == 1)
 		Tell_Yao_Xuan("predunk");
	if(GamePad_Data.key[1] == 1)
 		Tell_Yao_Xuan("defend");
	
	
	if(GamePad_Data.key[17] == 1)
 		Tell_Yao_Xuan("down");
	if(GamePad_Data.key[16] == 1)
 		Tell_Yao_Xuan("dribble");
	if(GamePad_Data.key[13] == 1)
 		Tell_Yao_Xuan("lift");
	if(GamePad_Data.key[15] == 1)
 		Tell_Yao_Xuan("jump");
	//更改Debug界面
	DebugPage_Change(GamePadKey_FallingCheck(4));
	//无线串口
	send.Debug.send_flag = GamePad_Data.witch[1];
	//加速和减速相关
	//计算朝向
	if(GamePad_Data.key[20])
		chassis.opposite = R1;
	else if(GamePad_Data.key[18])
		chassis.opposite = oppo_basket;
	else if(GamePad_Data.key[14])
		chassis.opposite = self_basket;
	else
		chassis.opposite = none;

	if(GamePadKey_FallingCheck(2) == 1) 
		Vision_Reset();
	if(GamePadKey_FallingCheck(0) == 1) 
		Vision_Flag_Clear();
	//清空码盘
	if((GamePad_Data.key[6] == 1) && (GamePad_Data.key[21] == 1))
 		Odometer_Clear("default"),Gyro_Reset();
	else if((GamePad_Data.key[6] == 1) && (GamePad_Data.key[21] == 0))
 		Odometer_Clear("armor"),Gyro_Reset();
	
	for (char i = 0; i<22;i++)
		GamePad_Data.last[i] = GamePad_Data.key[i];
} 
float Char2float(char str[4]){
	float data;
	memcpy(&data,str,sizeof(float));
	return data;
}


