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


char RGB_Switch;
#define  LED_NUM 15
#define  main_frequency 275000000
#define  prescaler  0
#define  period  (int)(main_frequency / 800000)
#define  RGB_HIGH  (int)(period*0.67)		
#define  RGB_LOW  (int)(period*0.33)			
#define  ws2812_tim htim23
#define  ws2812_channel TIM_CHANNEL_1
struct panel_t panel;
unsigned int send_buff[LED_NUM+1][3][8];		
void RGB_Init(void){
	TIM_MasterConfigTypeDef sMasterConfig;
	HAL_TIM_Base_Stop_IT(&ws2812_tim);
	HAL_TIM_Base_Stop(&ws2812_tim);
	__HAL_TIM_CLEAR_IT(&ws2812_tim, TIM_IT_UPDATE);
	__HAL_TIM_CLEAR_FLAG(&ws2812_tim, TIM_FLAG_UPDATE);
	__HAL_TIM_SET_COUNTER(&ws2812_tim, NONE);

	HAL_TIM_Base_DeInit(&ws2812_tim);
	ws2812_tim.Init.Prescaler = NONE; 
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

void RGB_Cal_Color(unsigned short LED_index, unsigned int color, unsigned char bright)
{
	if((LED_index < 0) || (LED_index >= LED_NUM))
		return;
	unsigned char R = (unsigned char)(color>>0x10) * ((float)bright/100.0);
	unsigned char G = (unsigned char)(color>>0x08) * ((float)bright/100.0);
	unsigned char B = (unsigned char)(color>>0x00) * ((float)bright/100.0);
	for(char i = 7; i>=0; i--)
			send_buff[LED_index][0][7-i] = ((G>>i) & 0x01) ? RGB_HIGH : RGB_LOW;
	for(char i = 7; i>=0; i--) 
			send_buff[LED_index][1][7-i] = ((R>>i) & 0x01) ? RGB_HIGH : RGB_LOW;
	for(char i = 7; i>=0; i--) 
			send_buff[LED_index][2][7-i] = ((B>>i) & 0x01) ? RGB_HIGH : RGB_LOW;
}
/*
   4  3  2  1  0
   9  8  7  6  5
  14 13 12 11 10
*/ 
//三行一个line
void RGB_Line_Cal(int lineindex,int color,int bright){	
	RGB_Cal_Color(lineindex,color,bright);
	RGB_Cal_Color(lineindex + 5,color,bright);
	RGB_Cal_Color(lineindex + 10,color,bright);
}
void RGB_List_Cal(int listindex,int color,int bright){
	for(unsigned char i = 0; i < 5; i ++) 
	  RGB_Cal_Color(listindex*5 + i,color,bright);
}
void RGB_Clear_Cal(void){
	for(int i = 0; i < LED_NUM; i++)
	  RGB_Cal_Color(i,Black,20);
}
void RGB_Color_All(int color,int bright){
	for(int i = 0; i < LED_NUM; i++)
	  RGB_Cal_Color(i,color,bright);
}
void RGB_OutPut(void)
{
	HAL_TIM_PWM_Stop_DMA(&ws2812_tim, ws2812_channel); 
	HAL_TIM_PWM_Start_DMA(&ws2812_tim, ws2812_channel, (unsigned int *)send_buff, sizeof(send_buff) / sizeof(short)); 	
}
void SwitchRGBShowMsg(void){
	//复位相关
	if((GamePad_Data.key[1] == 1) || (GamePad_Data.key[5] == 1) || (GamePad_Data.key[3] == 1))
		panel.display = gamepad_msg;
	//通信相关(防守)
	else if((GamePad_Data.key[0] == 1) || (GamePad_Data.key[2] == 1) || (GamePad_Data.key[4] == 1) || (GamePad_Data.key[6] == 1))
		panel.display = gamepad_msg;
	//通信相关(跳跃)
	else if((GamePad_Data.key[15] == 1) || (GamePad_Data.key[16] == 1) || (GamePad_Data.key[17] == 1) || (GamePad_Data.key[18] == 1) || (GamePad_Data.key[19] == 1))
		panel.display = gamepad_msg;
}
int Faded_Color(int colora,int colorb,float a){
	float percent = a / 100;
	unsigned char R = (unsigned char)(((float)(colora>>0x10)*percent) + ((float)(colorb>>0x10)*(1 - percent)));
	unsigned char G = (unsigned char)(((float)(colora>>0x08)*percent) + ((float)(colorb>>0x08)*(1 - percent)));
	unsigned char B = (unsigned char)(((float)(colora>>0x00)*percent) + ((float)(colorb>>0x00)*(1 - percent)));
	return (int)(R << 0x10) + (G << 0x08) + (B << 0x00);
}
char bright = 12;
char Check_Reset(void){
	char cnt;
//	if(yis506.reset_flag == 1) 					
//		cnt++,RGB_Line_Cal(0,Green,bright );
//	if(odometer.reset_flag == 1) 					
//		cnt++,RGB_Line_Cal(1,Blue,bright );
//	if(vision.position.online_flag == 1) 					
//		cnt++,RGB_Line_Cal(2,Purple,bright );
//	if(vision.basketlock.online_flag == 1) 			
//		cnt++,RGB_Line_Cal(3,Blue,bright );
//	if(send.R1_Exchange.get_dataflag == 1)					
//		cnt++,RGB_Line_Cal(4,Green,bright)
	if(yis506.reset_flag == 1) 					
		cnt++,RGB_Line_Cal(0,Faded_Color(Green,White,60),bright);
	if(odometer.reset_flag == 1) 					
		cnt++,RGB_Line_Cal(1,Faded_Color(Green,White,80),bright);
	if(vision.position.online_flag == 1) 					
		cnt++,RGB_Line_Cal(2,Faded_Color(Green,White,80),bright);
	if(vision.basketlock.online_flag == 1) 			
		cnt++,RGB_Line_Cal(3,Faded_Color(Green,White,100),bright);
	if(send.R1_Exchange.get_dataflag == 1)					
		cnt++,RGB_Line_Cal(4,Faded_Color(Green,White,100),bright);
	return cnt;
}

void RGB_Show_Msg(void){
	//清空显示
	RGB_Clear_Cal();
	
	
	SwitchRGBShowMsg();
	
	switch(panel.display){
		case init_msg:
			Check_Reset();
			RGB_OutPut();
		break;
		case gamepad_msg:
			RGB_Color_All(Purple,30);
			RGB_OutPut();
			osDelay(100);
			panel.display = init_msg;
		break;
		case wrong_msg:
			
		break;
	}
}


















void RGB_Total_Cal(int color,int bright){
	for(int i = 0; i< LED_NUM; i++){
		RGB_Cal_Color(i,color,bright);
	}
}
void RGB_Test(int index,int color,int bright){
	RGB_Clear_Cal();
	RGB_Cal_Color(index,color,bright);
	RGB_Cal_Color(index + 5,color,bright);
	RGB_Cal_Color(index + 10,color,bright);
	RGB_OutPut();
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

