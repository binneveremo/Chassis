#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H
#include "Location.h"
#include "Can_Bsp.h"
#include "Global.h"
#include "string.h"
#include "Interact.h"

#define receiveball_id_recv 0xB1
#define stickball_id_recv 0xB2
                    


void Tell_Yao_Xuan(char *message);
void Car_State_Decode(int id,unsigned char * data);

#endif
