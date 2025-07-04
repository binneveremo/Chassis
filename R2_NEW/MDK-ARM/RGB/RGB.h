#ifndef __RGB_H
#define __RGB_H

#include "tim.h"

#define  LED_NUM 82
#define  main_frequency 275000000
#define  prescaler  0
#define  period  (int)(main_frequency / 800000)
#define  RGB_HIGH  (int)((float)period*0.7)		
#define  RGB_LOW   (int)((float)period*0.3)			
#define  ws2812_tim htim23
#define  ws2812_channel TIM_CHANNEL_1

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


#define RGB(r,g,b) ((unsigned int)((((unsigned char)((float)r * 255.0 / 101.0) / 2 * 2) << 16) + (((unsigned char)((float)g * 255.0 / 101.0) / 2 * 2) << 8) + ((unsigned char)((float)b * 255.0 / 101.0) / 2 * 2)))
#define Faded(color,bright) ((unsigned int)(((unsigned char)((float)(((color)>>0x10) * bright / 100.0)) << 16) + ((unsigned char)((float)((color)>>0x08) * bright / 100.0) << 8) + ((unsigned char)((float)((color)>>0x00) * bright / 100.0))))

struct LED_t{
	enum{
		breath,
		single_wave,
		double_wave,
	}effect;
	struct {
		struct{
			char index_now;
			
		}wave;

	}flagof;
	
	
	
};
void RGB_Reset(void);
void RGB_Show(void);
void RGB_Cal_Color(unsigned short LED_index, unsigned int color);
#define RGB_RESET (RGB_ON = false) 
#define RGB_Clear() {for(char i =0; i < LED_NUM;i ++) RGB_Cal_Color(i,Black);}	
#define RGB_Total(color) {for(char i =0; i < LED_NUM;i ++) RGB_Cal_Color(i,color);}	

void RGB_Show_Msg(void);
void RGB_Init(void);
void RGB_Show_Color(int color,int bright);
void RGB_Show_Warning(void);
void RGB_Show_Test(int dt);
void RGB_Show_Velocity(void);
void RGB_Test(char index,unsigned int color);
#endif
