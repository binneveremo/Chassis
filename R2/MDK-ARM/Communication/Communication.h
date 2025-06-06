#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H
#include "Location.h"
#include "Can_Bsp.h"
#include "Global.h"
#include "string.h"
#include "Interact.h"

#define stickball_id_recv 0xB2
#define staffdown_id_recv 0xB3
                    

void Tell_Yao_Xuan(char *message);
void Car_State_Decode(int id,unsigned char * data);

#endif
