#include "Communication.h"
#include "Basket.h"
#include "Chassis.h"
#include "Flow.h"



void Tell_Yao_Xuan(char *message){	
#define Predunk interact.defend_status = (interact.defend_status == predunk)?interact.defend_status:predunk
	  ////////////防守指令//////////////////
    if(strcmp(message, "fold") == 0)
		   interact.defend_status = fold;
		else if(strcmp(message, "catch") == 0)
       interact.defend_status = catch_ball;
    else if(strcmp(message, "defend") == 0) 
       interact.defend_status = defend;
		else if(strcmp(message, "predunk") == 0)
       interact.defend_status = predunk;
		else if(strcmp(message, "midcatch") == 0)
       interact.defend_status = midcatch;
		else if(strcmp(message, "down") == 0) 
      FDCAN_Send(&Com_Can,poledown_id_send,"STD",NULL,"FD",0,"OFF");
		else if(strcmp(message, "dribble") == 0)
      FDCAN_Send(&Com_Can,dribble_id_send,"STD",NULL,"FD",0,"OFF"),Predunk;
    if(strcmp(message, "lift") == 0)
      FDCAN_Send(&Com_Can,lift_id_send,"STD",Basket_DisData,"FD",4,"OFF"),Predunk;
		else if(strcmp(message, "jump") == 0)
      FDCAN_Send(&Com_Can,jump_id_send,"STD",Basket_DisData,"FD",4,"OFF"),
			interact.flagof.dunk = true;
		else if(strcmp(message, "polecheck") == 0)
			FDCAN_Send(&Com_Can,polecheck_id_send,"STD",NULL,"FD",0,"OFF"),
			interact.defend_status = defend;
}
void Car_State_Decode(int id,unsigned char * data){
	if((id == pole_top_id_recv) && (chassis.Control_Status == Auto_Control))
		flow.flagof.pole_top = true;
	if((id == stickball_id_recv) && (chassis.Control_Status == Auto_Control))
		flow.flagof.stick_ball = true;
}
void ChooseCatchBall_StatusAuto(void){
	if(fabs(site.now.r - shoot.deal.opposite_angle) < 15)
		Tell_Yao_Xuan("defend");
	else 
		Tell_Yao_Xuan("midcatch");
}


