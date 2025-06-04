#ifndef __RGB_H
#define __RGB_H
#include "tim.h"

#define Red 0xFF0F0F
#define Green 0x0FAA0F
#define Blue 0x0F0FFF
#define Yellow 0xFFFF00
#define Purple 0x800080
#define Orange 0xFFB500
#define Pink 0xFFC0CB
#define White 0xFFFFFF
#define Black 0x000000
#define NavyBlue 0x000080

extern char RGB_Switch;

void RGB_Init(void);
void RGB_Show_Color(int color,int bright);
void RGB_Show_Warning(void);
void RGB_Show_Test(int dt);
void RGB_Show_Velocity(void);
#endif
