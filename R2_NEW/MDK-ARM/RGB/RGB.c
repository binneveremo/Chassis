#include "Television.h"
#include "Fake_Rtos.h"
#include "string.h"
#include "Location.h"
#include "Global.h"
#include "Chassis.h"
#include "Encoder.h"
#include "mine.h"
#include "RGB.h"
#include "dma.h"
#include "tim.h"
#include "gyro.h"


struct LED_t led;
unsigned int send_buff[LED_NUM+1][24];		
void RGB_Init(void){
	HAL_TIM_Base_Stop_IT(&ws2812_tim);
	HAL_TIM_Base_Stop(&ws2812_tim);
	__HAL_TIM_CLEAR_IT(&ws2812_tim, TIM_IT_UPDATE);
	__HAL_TIM_CLEAR_FLAG(&ws2812_tim, TIM_FLAG_UPDATE);
	__HAL_TIM_SET_COUNTER(&ws2812_tim, NONE);
	HAL_TIM_Base_DeInit(&ws2812_tim);
	RGB_Clear();
}
void RGB_Cal_Color(unsigned short LED_index, unsigned int color){
	if((LED_index < 0) || (LED_index >= LED_NUM))	return;
	unsigned char R = (unsigned char)(color>>0x10);
	unsigned char G = (unsigned char)(color>>0x08);
	unsigned char B = (unsigned char)(color>>0x00);
	for(signed char i = 7; i>=0; i--)		send_buff[LED_index][7-i] = ((G>>i) & 0x01) ? RGB_HIGH : RGB_LOW;
	for(signed char i = 7; i>=0; i--) 	send_buff[LED_index][15-i] = ((R>>i) & 0x01) ? RGB_HIGH : RGB_LOW;
	for(signed char i = 7; i>=0; i--) 	send_buff[LED_index][23-i] = ((B>>i) & 0x01) ? RGB_HIGH : RGB_LOW;
}
void RGB_OutPut(void){
	HAL_TIM_PWM_Stop_DMA(&ws2812_tim, ws2812_channel); 
	HAL_TIM_PWM_Start_DMA(&ws2812_tim, ws2812_channel, (unsigned int *)send_buff, (LED_NUM+1)*24); 	
}
void RGB_Wave(int wavecolor,int backcolor){
#define Move_Num 7
	static char last,dir;
	RGB_Total(backcolor);
	for(char i=0;i < Move_Num;i++)
		RGB_Cal_Color(led.flagof.wave.index_now + i,wavecolor);
	led.flagof.wave.index_now += (dir == 0)?-1:dir;
	if((led.flagof.wave.index_now == LED_NUM) || (led.flagof.wave.index_now == -1)) dir = !dir;
	last = HAL_GetTick();
}
void RGB_BreathProcessingBar(float percent,unsigned int color){
	static float bright;
	bright = (bright > 100)?0:bright + 8;
	char RGB_Calnum = percent * LED_NUM / 200;
	for(char i= -RGB_Calnum;i < RGB_Calnum;i++)
		RGB_Cal_Color(LED_NUM / 2 + i,color);
}







/////////////////////////////////////////////////////////////////彩虹灯带////////////////////////////////////////////


unsigned int HSV_To_RGB(unsigned short h) {
    h %= 1530; 
    unsigned char region = h / 255;  
    unsigned char val = h % 255;    
    switch(region) {
        case 0: return (255 << 16) | (val << 8);          // 红→黄: (255, val, 0)
        case 1: return ((255 - val) << 16) | (255 << 8);  // 黄→绿: (255-val, 255, 0)
        case 2: return (255 << 8) | val;                  // 绿→青: (0, 255, val)
        case 3: return ((255 - val) << 8) | 255;          // 青→蓝: (0, 255-val, 255) 
        case 4: return (val << 16) | 255;                 // 蓝→紫: (val, 0, 255)
        default: return (255 << 16) | (255 - val);        // 紫→红: (255, 0, 255-val)
    }
}
void RainBow_Effect_Cal(void) {
    unsigned short SPEED = 8; // 速度控制
    static unsigned short hue_offset;
    for(unsigned char i = 0; i < 4; i++) {
        for(unsigned short j = 0; j < 20; j++) {  // 修正：j++而不是i++
            // 计算每个LED在整个80个LED中的位置
            unsigned char pos = i * 20 + j;
            // 在整个灯带（80个LED）上均匀分布色相值
            unsigned short hue = (pos * 1530) / 80 + hue_offset;
            hue %= 1530;  // 确保在0-1529范围内
            unsigned int RGB_Color = HSV_To_RGB(hue);
            RGB_Cal_Color(pos, RGB_Color);
        }
    }
    // 更新全局偏移量
    hue_offset = (hue_offset + SPEED) % 1530;
}
void RainBow_Percent(float percent){
	RainBow_Effect_Cal();
	percent = Limit(percent,0,1);
	short num = percent * LED_NUM / 2;
	for(int i = 0; i < LED_NUM; i++){
		if(fabs((float)i - LED_NUM / 2) > num) 
			RGB_Cal_Color(i,Black);
	}
}




































void RGB_Show(void){
#define Cycle 0
#define Lock (chassis.lock.flag == true)
	static int cnt;
  cnt = (cnt > Cycle)?0:cnt+1;
	if(cnt != 0)
		return;
	RGB_Total(Black);
	switch(chassis.Control_Status){
		case GamePad_Control:
			RGB_Wave(Lock?RGB(9,18,45):RGB(45,4,2),Lock?RGB(7,37,10):RGB(27,10,12));
		break;
		case Auto_Control:
			switch(flow.type){
				case dribble_flow:
					
				break;
				case attack_flow:
				case dunk_flow:					
					//basketlock.position.percent>100?RGB_BreathProcessingBar(200 - basketlock.position.percent,RGB(45,34,2)):RGB_BreathProcessingBar(basketlock.position.percent,RGB(34,23,45));
					basketlock.position.percent>100?RGB_BreathProcessingBar(200 - basketlock.position.percent,RGB(45,34,2)):RGB_BreathProcessingBar(basketlock.position.percent,RGB(34,23,45));
				  //RainBow_Percent(basketlock.position.percent/100);
				break;
				case skill_flow:
					RainBow_Percent((float)skill.success_time/7.0f);
				default:
				break;
			}
		break;
		case Debug_Control:
			                
		break;
	}
//	if(GamePad_Data.witch[1] && (flow.flagof.receive_ball_bygyro == true))
//		RGB_Total(RGB(34,8,2));
	RGB_OutPut();
}
void RGB_Test(char index,unsigned int color){
	RGB_Total(color);
	RGB_OutPut();


}






























//int Faded_Color(int colora,int colorb,float a){
//	float percent = a / 100;
//	unsigned char R = (unsigned char)(((float)(colora>>0x10)*percent) + ((float)(colorb>>0x10)*(1 - percent))) / 2 * 2;
//	unsigned char G = (unsigned char)(((float)(colora>>0x08)*percent) + ((float)(colorb>>0x08)*(1 - percent))) / 2 * 2;
//	unsigned char B = (unsigned char)(((float)(colora>>0x00)*percent) + ((float)(colorb>>0x00)*(1 - percent))) / 2 * 2;
//	return (int)(R << 0x10) + (G << 0x08) + (B << 0x00);
//}
//unsigned int Light_Color(unsigned char color,int bright){
//	float R = (color>>0x10);
//	float G = (color>>0x08);
//	float B = (color>>0x00);
//	return RGB(R / 255 * bright,G / 255 * bright / 100,B / 255 * bright / 100);
//}
//unsigned char RGB_Change_Color(){
//	static int index;
//	const int colors[] = {Green, Blue, Red, Purple, Pink};
//	index = (index >= 5)?0:index;
//	return colors[index];
//}









