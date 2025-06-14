#include "Correct.h"
#include "Encoder.h"
#include "string.h"
#include "fdcan.h"

#include "Gyro.h"

char wrong_code[10];
char * can_wrong_code = "CanTxFull";
struct Wrong_Code_t Wrong_Code;
void Can_Detect(void){
#define BusyFlagof(x) (x -> TXFQS & 1 << 21)
	if(BusyFlagof(FDCAN1))
	 MX_FDCAN1_Init(),Wrong_Code.HT |=  1;
	if(BusyFlagof(FDCAN2))
	 MX_FDCAN2_Init(),Wrong_Code.HT |=  1;
	if(BusyFlagof(FDCAN3))
	 MX_FDCAN3_Init(),Wrong_Code.HT |=  1;
#undef BusyFlagof
}
void LossConnect_Check(void){
	memset(&Wrong_Code,NONE,sizeof(Wrong_Code));
	
	Wrong_Code.ho7213 |=  (!chassis.motor.turn[front_wheel].online_flag << 0);
	Wrong_Code.ho7213 |=   (!chassis.motor.turn[left_wheel].online_flag << 1);
	Wrong_Code.ho7213 |=  (!chassis.motor.turn[right_wheel].online_flag << 2);
	Wrong_Code.ho7213 |= (!chassis.motor.turn[behind_wheel].online_flag << 3);
	
	Wrong_Code.vesc |=   (!chassis.motor.drive[front_wheel].online_flag << 0);
	Wrong_Code.vesc |=    (!chassis.motor.drive[left_wheel].online_flag << 1);
	Wrong_Code.vesc |=   (!chassis.motor.drive[right_wheel].online_flag << 2);
	Wrong_Code.vesc |=  (!chassis.motor.drive[behind_wheel].online_flag << 3);
	
	Wrong_Code.odom |= (!odometer.xenc_online << 0);
	Wrong_Code.odom |= (!odometer.yenc_online << 1);
	
	Wrong_Code.gyro    |= (!yis506.online_flag << 0);
	Wrong_Code.R1_loss |= (!send.R1_Exchange.get_dataflag << 0);
	
	Wrong_Code.ladar |= (!vision.position.online_flag << 0);
	Wrong_Code.basket_near |= (!vision.basketlock.online_flag << 0);
	
	Wrong_Code.HT |= (1 << (interact.wrongcode.HT_Error - 1));
	
	/////////////清零操作
	for(unsigned char i = 0; i < TURN_NUM;i++)
		chassis.motor.turn[i].online_flag = false;
	for(unsigned char i = 0; i < VESC_NUM;i++)
		chassis.motor.drive[i].online_flag = false;
	yis506.online_flag = false;
	odometer.xenc_online = false;
	odometer.yenc_online = false;
	
	vision.position.online_flag = false;
	vision.basketlock.online_flag = false;
	send.R1_Exchange.get_dataflag = false;
}











               







