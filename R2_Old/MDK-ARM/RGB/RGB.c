#include "RGB.h"
#include "dma.h"
#include "tim.h"
#include "string.h"
#include "Location.h"
#include "Global.h"
#include "Chassis.h"
char RGB_Switch;
#define  LED_NUM 80
#define  main_frequency 275000000
#define  prescaler  0
#define  period  (int)(main_frequency / 800000)
#define  RGB_HIGH  (int)(period*0.7)		
#define  RGB_LOW  (int)(period*0.3)			
#define  ws2812_tim htim4
#define  ws2812_channel TIM_CHANNEL_2

unsigned int send_buff[LED_NUM][3][8];		
void RGB_Init(void){
	TIM_MasterConfigTypeDef sMasterConfig;
	HAL_TIM_Base_Stop_IT(&ws2812_tim);
	HAL_TIM_Base_Stop(&ws2812_tim);
	__HAL_TIM_CLEAR_IT(&ws2812_tim, TIM_IT_UPDATE);
	__HAL_TIM_CLEAR_FLAG(&ws2812_tim, TIM_FLAG_UPDATE);
	__HAL_TIM_SET_COUNTER(&ws2812_tim, 0);

	HAL_TIM_Base_DeInit(&ws2812_tim);
	ws2812_tim.Init.Prescaler = 0; 
	ws2812_tim.Init.CounterMode = TIM_COUNTERMODE_UP;        
	ws2812_tim.Init.Period = 342;                              
	ws2812_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;  
	if (HAL_TIM_Base_Init(&ws2812_tim) != HAL_OK)
			Error_Handler();
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&ws2812_tim, &sMasterConfig) != HAL_OK)
			Error_Handler();
}

void RGB_Cal_Color(unsigned short int LED_index, unsigned int color, unsigned char bright)
{
	if((LED_index < 0) || (LED_index >= LED_NUM))
		return;
	unsigned char R = (uint8_t)(color>>0x10) * ((float)bright/100.0);
	unsigned char G = (uint8_t)(color>>0x08) * ((float)bright/100.0);
	unsigned char B = (uint8_t)(color>>0x00) * ((float)bright/100.0);
	for(int8_t i = 7; i>=0; i--)
			send_buff[LED_index][0][7-i] = ((G>>i) & 0x01) ? RGB_HIGH : RGB_LOW;
	for(int8_t i = 7; i>=0; i--) 
			send_buff[LED_index][1][7-i] = ((R>>i) & 0x01) ? RGB_HIGH : RGB_LOW;
	for(int8_t i = 7; i>=0; i--) 
			send_buff[LED_index][2][7-i] = ((B>>i) & 0x01) ? RGB_HIGH : RGB_LOW;
}
void RGB_Line_Cal(int lineindex,int color,int bright){
	for(int i = lineindex; i < 5; i += 5)
	 RGB_Cal_Color(i,color,bright);
}
void RGB_List_Cal(int listindex,int color,int bright){
	for(int i = 0; i < 5; i += 1)
	 RGB_Cal_Color(listindex*5 + i,color,bright);
}
void RGB_Clear_Cal(void){
	for(int i = 0; i < LED_NUM; i++)
	  RGB_Cal_Color(i,Black,10);
}
void RGB_OutPut(void)
{
	HAL_TIM_PWM_Stop_DMA(&ws2812_tim, ws2812_channel); 
	HAL_TIM_PWM_Start_DMA(&ws2812_tim, ws2812_channel, (unsigned int *)send_buff, LED_NUM*3*8+1); 	
}
void RGB_Total_Cal(int color,int bright){
	for(int i = 0; i< LED_NUM; i++){
		RGB_Cal_Color(i,color,bright);
	}
}
char RGB_Inner_Count(char count_index,int dt){
	int now = HAL_GetTick();
	static int last[5];
	if(count_index == 0){
		if(now - last[0] > dt){
			last[0] = now;
			return 1;
		}
	}
	else if(count_index == 1){
		if(now - last[1] > dt){
			last[1] = now;
			return 1;
		}
	}
	else if(count_index == 2){
		if(now - last[2] > dt){
			last[2] = now;
			return 1;
		}
	}
	else if(count_index == 3){
		if(now - last[3] > dt){
			last[3] = now;
			return 1;
		}
	}
	else if(count_index == 4){
		if(now - last[4] > dt){
			last[4] = now;
			return 1;
		}
	}
	return 0;
}
void RGB_Show_Velocity(void){
	RGB_Clear_Cal();
	float v = hypot(site.car.vx_gyro,site.car.vy_gyro);
	if(v > 0.2)
		RGB_Line_Cal(0,0xFF3F3F,2);
	if(v > 1)
		RGB_Line_Cal(1,0xFF2c2c,6);
	if(v > 1.5)
		RGB_Line_Cal(2,0xFF0000,10);
	if(v > 2)
		RGB_Line_Cal(3,0x8B0000,20);
	if(v > 2.4)
		RGB_Line_Cal(4,0x4B0000,40);
	RGB_OutPut();
}
static char A[25] = {
	0, 0, 1, 0, 0, 
	0, 1, 0, 1, 0,
  0, 1, 1, 1, 0, 
  1, 0, 0, 0, 1,  
  1, 0, 0, 0, 1   
};
static char B[25] = {
	1, 1, 1, 0, 0, 
	1, 0, 0, 1, 0,
  1, 1, 1, 0, 0, 
  1, 0, 0, 1, 0,  
  1, 1, 1, 0, 0   
};
static char C[25] = {
	0, 1, 1, 1, 0, 
	1, 0, 0, 0, 0,
  1, 0, 0, 0, 0, 
  1, 0, 0, 0, 0,  
  0, 1, 1, 1, 0   
};
static char Z[25] = {
	1, 1, 1, 1, 1, 
	0, 0, 0, 1, 0,
  0, 0, 1, 0, 0, 
  0, 1, 0, 0, 0,  
  1, 1, 1, 1, 1   
};

////////////////////////////////////////////////////////////////////一定要记住 先考虑line 在考虑list 对于我们的RGB来说 也就是先考虑堆叠 在考虑侧向 也就是 char * letter[5][3]
char RGB_Order_Convert(char order){
	 return 14 - (order / 3) - (order % 3) * 5;
}
/*
  10 11 12 13 14
   5  6  7  8  9
   0  1  2  3  4
*/ 
/*
14 9 4
13 8 3
12 7 2
11 6 1
10 5 0
*/
char * Letter_Walk(char * input,int begin){
	//从倒数第二个开始
	static char letter[15];
	for(int i = 0; i <5; i++){
	   for(int j = 0; j < 3; j++){
			 if((j + begin > 4) || (j + begin < 0)){
				 letter[3*i + j] = 0;
				 continue;
			 }
		   letter[3*i + j] = input[5*i + j + begin];
		 }
	}
	return letter;
}
void sring_walk(char*a,char*b){
	static int cnt;
	


}

void RGB_Letter_Cal(void){
	

}
void RGB_Show_Letter(char * letter,int color,int bright){
	for(int i = 0; i< 15 ; i++)
	  RGB_Cal_Color(RGB_Order_Convert(i), color*letter[i],bright);
}


//////////////////////////////////////////////////////////////freeRtos里面调用的函数///////////////////////////////
void RGB_Show_Warning(void){
	static int flag;
	static unsigned char pos[15];
	if(hypot(site.car.vx_enc,site.car.vy_enc) > 0.3){
		if(flag == 1)
		  RGB_Total_Cal(Red,3);
		else if(flag == 0)
		  RGB_Total_Cal(Red,40);
		flag =! flag;
	}
}
int RGB_Change_Color(int color){
	if(color == Green)
		return Blue;
	if(color == Blue)
		return Red;
	if(color == Red)
		return Purple;
	if(color == Purple)
		return Pink;
  if(color == Pink)
		return Green;
	if(color == 0)
	  return Green;
	return White;
}
void RGB_Breath(int bright_max,int dt){
	static int color;
	static char bright;
	static char flag;
	if(flag == 0){
		flag = 1;
		color = Green;
	}
	if(RGB_Inner_Count(0,dt) == 1){
		if(bright >= bright_max)
			flag = -1;
		else if(bright <= 0)
			flag = 1;
		bright += flag;
		if(bright == 0)
		  color = RGB_Change_Color(color);
	}
	RGB_Total_Cal((int)color,bright);
}
int RGB_Flow(int bright,char dt,char clear_flag){
	static int cnt;
	static int color;
	static int pos;
	static char flag; 
	if(flag == 0){
		flag = 1;
		color = Green;
	}
	if(RGB_Inner_Count(0,dt) == 1){
		if(pos >= LED_NUM){
			flag = -1;
			color = RGB_Change_Color(color);
		}
		else if(pos < 0){
			flag = 1;
			color = RGB_Change_Color(color);
			cnt++;
		}
		pos += flag;
	}
	if(clear_flag == 1)
		RGB_Clear_Cal();
	RGB_Cal_Color(pos,color,bright);
	RGB_Cal_Color(pos + 1,color,(int)bright);
	RGB_Cal_Color(pos + 2,color,(int)bright);
	RGB_Cal_Color(pos + 3,color,(int)bright);
	RGB_Cal_Color(pos + 4,color,(int)bright);
	return cnt;
}
void RGB_Flow_Circle(void){
	
	


}
void RGB_Show_Test(int dt){
//	if(RGB_Inner_Count(1,5) == 1){
		//RGB_Breath(50,dt);
//	  RGB_Flow(40,dt,1);
  if(RGB_Inner_Count(1,15) == 1){
		if(RGB_Switch == 1){
			RGB_Flow(30,30,0);
		}
		else if(RGB_Switch == 0){
			RGB_Clear_Cal();
		}
		RGB_OutPut();
	}
}

