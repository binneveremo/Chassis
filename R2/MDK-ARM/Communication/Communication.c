#include "Communication.h"
#define Com_Can hfdcan3

#define poledown_id_send 0xCA
#define dribble_id_send 0xCB
#define lift_id_send 0xCC
#define jump_id_send 0xCD


void Tell_Yao_Xuan(char *message){
	  ////////////防守指令//////////////////
    if(strcmp(message, "fold") == 0)
		   interact.defend_status = fold;
		else if(strcmp(message, "catch") == 0)
       interact.defend_status = catch_ball;
    else if(strcmp(message, "defend") == 0) 
       interact.defend_status = defend;
		else if(strcmp(message, "predunk") == 0)
       interact.defend_status = predunk;
		else if(strcmp(message, "moving") == 0)
       interact.defend_status = moving;
		
		else if(strcmp(message, "down") == 0) 
      FDCAN_Send(&Com_Can,poledown_id_send,"STD",NULL,"FD",0,"OFF");
		else if(strcmp(message, "dribble") == 0)
      FDCAN_Send(&Com_Can,dribble_id_send,"STD",NULL,"FD",0,"OFF");
    else if(strcmp(message, "lift") == 0) {
      FDCAN_Send(&Com_Can,lift_id_send,"STD",NULL,"FD",0,"OFF");
			interact.defend_status = (interact.defend_status == predunk)?interact.defend_status:predunk;
		}
		else if(strcmp(message, "jump") == 0){ 
			interact.defend_status = (interact.defend_status == predunk)?interact.defend_status:predunk;
      FDCAN_Send(&Com_Can,jump_id_send,"STD",NULL,"FD",0,"OFF");
		}
}

void Car_State_Decode(int id,unsigned char * data){
	if(id == receiveball_id_recv)
		flow.flagof.received = true;
	if(id == stickball_id_recv)
		flow.flagof.stick_ball = true;
}


