#ifndef __RGB_H
#define __RGB_H

#include "tim.h"


#define Red ((int)0x00FF0000)
#define Green ((int)0x0000FF00)
#define Blue ((int)0x000F0FFF)
#define Yellow ((int)0x800080)
#define Purple ((int)0x800080)
#define Orange ((int)0xFFB500)
#define Pink ((int)0xFFC0CB)
#define White ((int)0xFFFFFF)
#define Black ((int)0x000000)
#define NavyBlue ((int)0x0F0FA0)

extern char RGB_Switch;


struct panel_t{
	enum{
		init_msg,
		press_msg,
		flow_msg,
	}display;
	


};


void RGB_Show_Msg(void);
void RGB_Init(void);
void RGB_Show_Color(int color,int bright);
void RGB_Show_Warning(void);
void RGB_Show_Test(int dt);
void RGB_Show_Velocity(void);
void RGB_Test(int index,int color,int bright);
#endif
