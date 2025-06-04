#include "GM6020.h"
#include "Chassis.h"
#include "Global.h"
#include "fdcan.h"
#include "math.h"
unsigned char gm6020_data[3][8];
unsigned char gm6020_send[8];
struct GM6020 gm6020;
int FDCAN1_Send_6020Data(void)
{	
	  FDCAN_TxHeaderTypeDef fdcan1_TxHeader;
		fdcan1_TxHeader.Identifier=0x1FF;
		fdcan1_TxHeader.IdType=FDCAN_STANDARD_ID;                  
		fdcan1_TxHeader.TxFrameType=FDCAN_DATA_FRAME;              //����֡
		fdcan1_TxHeader.DataLength=FDCAN_DLC_BYTES_8;                            //���ݳ���
		fdcan1_TxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
		fdcan1_TxHeader.BitRateSwitch=FDCAN_BRS_OFF;               //�ر������л�
		fdcan1_TxHeader.FDFormat=FDCAN_CLASSIC_CAN;                //��ͳ��CANģʽ
		fdcan1_TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //�޷����¼�
		fdcan1_TxHeader.MessageMarker=0;                           
		if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&fdcan1_TxHeader,gm6020_send)==HAL_OK) return 2;//����
		return 1;	
}
float Normalize_Angle(float angle){
	if(angle > 180)
		return angle - 360;
	else if(angle < -180)
		return angle + 360;
	return angle;
}
void GM6020_Status_Cla(void){
	for(int i = 0;i< 3 ;i++){
		short int angle = (gm6020_data[i][0]<< 8) + gm6020_data[i][1];
		short int v = (gm6020_data[i][2]<< 8) + gm6020_data[i][3];
		gm6020.status[i].v = v;
		gm6020.status[i].angle = angle * 360 / 8192;
	}
}
void GM6020_Velocity_PID(int name, float target){
	float error = target - gm6020.status[name].v;
	//p
	float p = gm6020.velocity[name].p * error;
	//i
	float i = 0;
	if((fabs(error) > gm6020.velocity[name].istart) && (fabs(error) < gm6020.velocity[name].iend))
		i = gm6020.velocity[name].i * error;
	gm6020.velocity[name].itotal = Limit(gm6020.velocity[name].itotal + i,-gm6020.velocity[name].ilimit,gm6020.velocity[name].ilimit);
	//d
	float d = gm6020.velocity[name].d * (gm6020.velocity[name].velocitypre - gm6020.status[name].v);
	gm6020.velocity[name].velocitypre = gm6020.status[name].v;
	
	gm6020.status[name].out = Limit(p + d + gm6020.velocity[name].itotal, -gm6020.velocity[name].outlimit,gm6020.velocity[name].outlimit);
}
void GM6020_Angle_PID(int name, float target){
	float angletemp = target;
	float gain = 1.0f;
	
	for(int i = 0;i < 50; i++){
		if(angletemp < gm6020.angle[name].anglemulti){
			if(fabs(angletemp - gm6020.angle[name].anglemulti) < 100)
				break;
			angletemp = 180*i + target;
		}
		else{
			if(fabs(angletemp - gm6020.angle[name].anglemulti) < 100)
				break;
			angletemp = target - 180*i;
		}
	}
	if (((int)fabs(angletemp - target) / 180) % 2 == 1)   gm6020.wheel[name].dir = -1;    
	else gm6020.wheel[name].dir = 1;
	//计算多圈角度
	float angle_diff = gm6020.status[name].angle - gm6020.angle[name].anglepre;
	if(angle_diff < -180)  angle_diff += 360;
	else if(angle_diff > 180)  angle_diff -= 360;
	gm6020.angle[name].anglemulti += angle_diff;
	gm6020.angle[name].anglepre =  gm6020.status[name].angle;
	//p
	float error = angletemp - gm6020.angle[name].anglemulti;
	float pnow =gm6020.angle[name].p * error;
	//i
	float inow = 0;
	if((fabs(error) > gm6020.angle[name].istart) && (fabs(error) < gm6020.angle[name].iend))
		inow = gm6020.angle[name].i * error;
	gm6020.angle[name].itotal = Limit(gm6020.angle[name].itotal + inow,-gm6020.angle[name].ilimit, gm6020.angle[name].ilimit);
	//g
	if(fabs(error) < gm6020.angle[name].death)  gain = Limit(fabs(error / gm6020.angle[name].death),0.2,1);
	float out = Limit(gain*pnow,-gm6020.angle[name].outlimit,gm6020.angle[name].outlimit);
	GM6020_Velocity_PID(name,out);
}
void GM6020_OutPut(void){
	for(int i = 0; i< 3; i++){
		gm6020_send[2*i] = (short int)gm6020.status[i].out>>8;
	  gm6020_send[2*i + 1] = (short int)gm6020.status[i].out & 0x00FF;
	}
	FDCAN1_Send_6020Data();
}
void GM6020_Par_Init(void){
	gm6020.angle[frontmotor].p = 70;//110
	gm6020.angle[frontmotor].i = 8;
	gm6020.angle[frontmotor].istart = 0.4;
	gm6020.angle[frontmotor].iend= 5;
	gm6020.angle[frontmotor].ilimit = 1600;
	gm6020.angle[frontmotor].outlimit = 6000;
	gm6020.angle[frontmotor].death = 14;
	
	gm6020.velocity[frontmotor].p = 64;
	gm6020.velocity[frontmotor].i = 2;
	gm6020.velocity[frontmotor].d = 3;
	gm6020.velocity[frontmotor].istart = 2;
	gm6020.velocity[frontmotor].iend= 10;
	gm6020.velocity[frontmotor].ilimit= 1000;
	gm6020.velocity[frontmotor].outlimit = 20000;
	
	
	gm6020.angle[leftmotor].p = 50;//85
	gm6020.angle[leftmotor].i = 8;
	gm6020.angle[leftmotor].istart = 0.4;
	gm6020.angle[leftmotor].iend= 5;
	gm6020.angle[leftmotor].ilimit = 1600;
	gm6020.angle[leftmotor].outlimit = 5000;
	gm6020.angle[leftmotor].death = 14;
	
	gm6020.velocity[leftmotor].p = 64;
	gm6020.velocity[leftmotor].i = 2;
	gm6020.velocity[leftmotor].d = 3;
	gm6020.velocity[leftmotor].istart = 2;
	gm6020.velocity[leftmotor].iend= 10;
	gm6020.velocity[leftmotor].ilimit= 1000;
	gm6020.velocity[leftmotor].outlimit = 20000;
	
	gm6020.angle[rightmotor].p = 50;//85
	gm6020.angle[rightmotor].i = 8;
	gm6020.angle[rightmotor].istart = 0.4;
	gm6020.angle[rightmotor].iend= 5;
	gm6020.angle[rightmotor].ilimit = 1600;
	gm6020.angle[rightmotor].outlimit = 5000;
	gm6020.angle[rightmotor].death = 14;
	
	gm6020.velocity[rightmotor].p = 64;
	gm6020.velocity[rightmotor].i = 2;
	gm6020.velocity[rightmotor].d = 3;
	gm6020.velocity[rightmotor].istart = 2;
	gm6020.velocity[rightmotor].iend= 10;
	gm6020.velocity[rightmotor].ilimit= 1000;
	gm6020.velocity[rightmotor].outlimit = 20000;
	
}