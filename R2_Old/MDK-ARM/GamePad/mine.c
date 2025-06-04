#include "Communication.h"
#include "Television.h"
#include "mngCommu.h"
#include "recDecode.h"
#include "Encoder.h"
#include "Correct.h"
#include "string.h"
#include "Global.h"
#include "string.h"
#include "Send.h"
#include "RGB.h"
#include "Flow.h"
#include "Gyro.h"
#include "mine.h"
#define DebugPage_Change(last,next)	(GamePad_Data.Debug_Page = Limit(GamePad_Data.Debug_Page - last + next,0,5))
#define GamePadKey_FallingCheck(index) ((GamePad_Data.key[index] == 0) && (GamePad_Data.last[index] == 1))

struct Game_Pad_Data GamePad_Data;


void Get_GamePad_Data(void){
	for(int i = 0; i< 4;i++)
		GamePad_Data.rocker[i] = get_rocker(i);
	for(int i = 0; i< 22;i++)
		GamePad_Data.key[i] = get_fk_state(i);
	for(int i = 0; i< 10;i++)
	   GamePad_Data.witch[i] = get_sw_state(i);
} 
char VisionReset_FallingEdge_Check(void){
	static char last;
	char now = get_fk_state(1);
	char flag = ((now == 0) && (last == 1))?1:0;
	last = now;
	return last;
}
void GamePad_Data_Cla(void){ 
	//拨码状态切换
	Get_GamePad_Data();
	
	if(GamePad_Data.witch[5] == 1)
		chassis.Control_Status = safe;
	else if(GamePad_Data.witch[1] == 1)
		chassis.Control_Status = gamepad_noheader;
	else if(GamePad_Data.witch[3] == 1)
		chassis.Control_Status = gamepad_standard;
	else if(GamePad_Data.witch[8] == 1)
		chassis.Control_Status = basket_lock;
	else if(GamePad_Data.witch[7] == 1)
		chassis.Control_Status = dribble;
	else if((GamePad_Data.witch[6] == 1) || ((GamePad_Data.key[7] == true) && (GamePad_Data.key[20] == true)))
		chassis.Control_Status = back;
	
	else 
		chassis.Control_Status = safe;
	
	chassis.Flagof.GamePad_Inverse = GamePad_Data.witch[0];
	//通信方面
#if false
	if(GamePad_Data.key[4] == 1)
 		Tell_Yao_Xuan("fold");
	if(GamePad_Data.key[6] == 1)
 		Tell_Yao_Xuan("catch");
	if(GamePad_Data.key[2] == 1)
 		Tell_Yao_Xuan("predunk");
	if(GamePad_Data.key[0] == 1)
 		Tell_Yao_Xuan("defend");
	
	
	if(GamePad_Data.key[18] == 1)
 		Tell_Yao_Xuan("down");
	if(GamePad_Data.key[17] == 1)
 		Tell_Yao_Xuan("dribble");
	if(GamePad_Data.key[16] == 1)
 		Tell_Yao_Xuan("lift");
	if(GamePad_Data.key[15] == 1)
 		Tell_Yao_Xuan("jump");
	if(GamePad_Data.key[19] == 1)
 		Tell_Yao_Xuan("reset");
#else
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
	if(GamePad_Data.key[19] == 1)
 		Tell_Yao_Xuan("stick");
#endif

	//更改Debug界面
	DebugPage_Change(GamePadKey_FallingCheck(14),GamePadKey_FallingCheck(13));
	//无线串口
	send.Debug.send_flag = GamePad_Data.witch[1];
	//加速和减速相关
	chassis.Flagof.GamePad_Accel  = GamePad_Data.key[21];
	chassis.Flagof.GamePad_Slow  = GamePad_Data.key[8];
	//chassis.Flagof.GamePad_Inverse = (GamePadKey_FallingCheck(7) == 1)?(!chassis.Flagof.GamePad_Inverse):chassis.Flagof.GamePad_Inverse;
	//vision.reset_flag = (GamePadKey_FallingCheck(1))?1:vision.reset_flag;
	//
	if(GamePadKey_FallingCheck(1) == 1) Vision_Reset(),Vision_Flag_Clear();
	if(GamePadKey_FallingCheck(20) == 1) Wrong_Code_Clear();
	
	//清空码盘
	if((GamePad_Data.key[5] == 1) && (GamePad_Data.key[21] == 1))
 		Odometer_Clear("default");
	else if((GamePad_Data.key[5] == 1) && (GamePad_Data.key[21] == 0))
 		Odometer_Clear("armor");
	if(GamePad_Data.key[5] == 1)
		Gyro_Reset();
	//线程方面
	if(GamePadKey_FallingCheck(3) == 1){
		Gyro_Reset();
		Vision_Reset(),Vision_Flag_Clear();
		Odometer_Clear("default"); 
	}

	for (char i = 0; i<22;i++)
		GamePad_Data.last[i] = GamePad_Data.key[i];
} 
float Char2float(char str[4]){
	float data;
	memcpy(&data,str,sizeof(float));
	return data;
}


