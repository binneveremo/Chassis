#include "Communication.h"
#define Com_Can hfdcan3

#define poledown_id 0xCA
#define dribble_id 0xCB
#define lift_id 0xCC
#define jump_id 0xCD
#define reset_id 0xCE


#define BytesOf_R1Data 8 
struct R1_Exchange R1;
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
		
		////////////////////
		else if(strcmp(message, "down") == 0) 
      FDCAN_Send(&Com_Can,poledown_id,"STD",NULL,"FD",0,"OFF");
		else if(strcmp(message, "dribble") == 0)
      FDCAN_Send(&Com_Can,dribble_id,"STD",NULL,"FD",0,"OFF");
    else if(strcmp(message, "lift") == 0) {
			interact.defend_status = (interact.defend_status == predunk)?interact.defend_status:predunk;
      FDCAN_Send(&Com_Can,lift_id,"STD",NULL,"FD",0,"OFF");
		}
		else if(strcmp(message, "jump") == 0){ 
			interact.defend_status = (interact.defend_status == predunk)?interact.defend_status:predunk;
      FDCAN_Send(&Com_Can,jump_id,"STD",NULL,"FD",0,"OFF");
		}
		else if(strcmp(message, "reset") == 0)
      FDCAN_Send(&Com_Can,reset_id,"STD",NULL,"FD",0,"OFF");
		strcpy(message,"nothing"); 
}


void Get_R1_Data(unsigned char * data){
	if(data[0] != 0xA5)
		return;
	memcpy(R1.convert.uint8_data,data + 1,BytesOf_R1Data);
	R1.x = R1.convert.float_data[0];
	R1.y = R1.convert.float_data[1];
	R1.oppsite_angle = Limit(rad2ang(atan2f(site.now.y - 275 - R1.y ,site.now.x - R1.x  + 275)), -90,90);
}




