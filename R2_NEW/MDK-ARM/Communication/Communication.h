#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H
#include "Location.h"
#include "Can_Bsp.h"
#include "Global.h"
#include "string.h"
#include "Interact.h"
#include "catchball.h"

#define Com_Can hfdcan4


#define stickball_id_recv 0xB2
#define pole_top_id_recv 0xB3
#define poledown_id_send 0xCA
#define dribble_id_send 0xCB
#define lift_id_send 0xCC
#define jump_id_send 0xCD
#define polecheck_id_send 0xCE
#define basketdis_id_send 0xCF

void Tell_Yao_Xuan(char *message);
void Car_State_Decode(int id,unsigned char * data);
void Send_BasketDis(void);
#define Basket_DisData (unsigned char*)(&basketlock.position.ladar2basketdis)
#define Send_BasketDis() FDCAN_Send(&Com_Can,basketdis_id_send,"STD",Basket_DisData,"FD",4,"OFF")
void ChooseCatchBall_StatusAuto(void);



#endif
