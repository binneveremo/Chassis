#include "fdcan.h"
#include "VESC.h"
unsigned char VESC_Send_Buffer[8];
#define VESC_CAN_SendData(id,send) FDCAN_Send(&VESC_CAN,id,"EXT",send,"CLASSIC",4,"OFF")
bool VESC_SetRPM(float value, uint8_t id) {
	unsigned int eid = (id & 0xff) | ((uint32_t)CAN_PACKET_SET_RPM << 8);
	int temp = (int)(value * RPM_SCALE);
	for (char i = 3; i >= 0; i--)
		VESC_Send_Buffer[i] = (temp >> ((3 - i) * 8)) & 0xff;
	VESC_CAN_SendData(eid,VESC_Send_Buffer);
	return true;
}
bool VESC_SetMultiturnPos(float value, unsigned char id) {
	unsigned int eid = (id & 0xff) | ((uint32_t)CAN_PACKET_SET_POS_MULTITURN << 8);
	int temp = (int)value*MULTITURN_POS_SCALE;
	for (char i = 3; i >= 0; i--)
		VESC_Send_Buffer[i] = (temp >> ((3 - i) * 8)) & 0xff;
	VESC_CAN_SendData(eid, VESC_Send_Buffer);
	return true;
}
bool VESC_SetPos(float value, unsigned char id) {
	unsigned int eid = (id & 0xff) | ((uint32_t)CAN_PACKET_SET_POS << 8);
	int temp = (int)value*POS_SCALE;
	for (char i = 3; i >= 0; i--)
		VESC_Send_Buffer[i] = (temp >> ((3 - i) * 8)) & 0xff;
	//send
	VESC_CAN_SendData(eid, VESC_Send_Buffer);
	return true;
}




