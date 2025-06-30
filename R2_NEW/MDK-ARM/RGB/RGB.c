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
struct LED_t led;
unsigned int send_buff[LED_NUM+1][24];		
unsigned int RGB(float r,float g,float b){
	r = (unsigned char)(r * 255.0 / 101.0) / 2 * 2;
	g = (unsigned char)(g * 255.0 / 101.0) / 2 * 2;
	b = (unsigned char)(b * 255.0 / 101.0) / 2 * 2;
	
	return (unsigned int)(((unsigned char)r << 16) + ((unsigned char)g << 8) + (unsigned char)b);
}
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
	static char last,dir;
	RGB_Total(backcolor);
	for(char i=0;i < 7;i++)
		RGB_Cal_Color(led.flagof.wave.index_now + i,wavecolor);
	led.flagof.wave.index_now += (dir == 0)?-1:dir;
	if((led.flagof.wave.index_now == LED_NUM) || (led.flagof.wave.index_now == -1)) dir = !dir;
	RGB_OutPut();
	last = HAL_GetTick();
}
float ctest[12] = {15,25,10,3,15,25,45,4,2,27,10,12};
//12 56 4 7 37 10

void RGB_Show(void){
	if(chassis.lock.flag == true)
		RGB_Wave(RGB(9,18,45),RGB(7,37,10));
	else 
		RGB_Wave(RGB(45,4,2),RGB(27,10,12));
}
void RGB_Test(char index,unsigned int color){
	RGB_Cal_Color(index,color);
	RGB_OutPut();


}






























int Faded_Color(int colora,int colorb,float a){
	float percent = a / 100;
	unsigned char R = (unsigned char)(((float)(colora>>0x10)*percent) + ((float)(colorb>>0x10)*(1 - percent))) / 2 * 2;
	unsigned char G = (unsigned char)(((float)(colora>>0x08)*percent) + ((float)(colorb>>0x08)*(1 - percent))) / 2 * 2;
	unsigned char B = (unsigned char)(((float)(colora>>0x00)*percent) + ((float)(colorb>>0x00)*(1 - percent))) / 2 * 2;
	return (int)(R << 0x10) + (G << 0x08) + (B << 0x00);
}
unsigned int Light_Color(unsigned char color,int bright){
	float R = (color>>0x10);
	float G = (color>>0x08);
	float B = (color>>0x00);
	return RGB(R / 255 * bright,G / 255 * bright / 100,B / 255 * bright / 100);
}
unsigned char RGB_Change_Color(){
	static int index;
	const int colors[] = {Green, Blue, Red, Purple, Pink};
	index = (index >= 5)?0:index;
	return colors[index];
}









































//void Check_Reset(void){
//	char bright = 12;
//	if(yis506.reset_flag == 1) 					
//		RGB_Line_Cal(0,Faded_Color(Green,White,60),bright);
//	if(odometer.reset_flag == 1) 					
//		RGB_Line_Cal(1,Faded_Color(Green,White,80),bright);
//	if(vision.position.online_flag == 1) 					
//		RGB_Line_Cal(2,Faded_Color(Green,White,80),bright);
//	if(vision.basketlock.online_flag == 1) 			
//		RGB_Line_Cal(3,Faded_Color(Green,White,100),bright);
//	if(send.R1_Exchange.get_dataflag == 1)					
//		RGB_Line_Cal(4,Faded_Color(Green,White,100),bright);
//}
//void RGB_Wave(int color){
//	static int cnt;
//	char bright[5] = {5,7,9,11,13};
//	cnt++;
//	RGB_Line_Cal(0,color,bright[(cnt) % 5]);
//	RGB_Line_Cal(1,color,bright[(cnt + 1) % 5]);
//	RGB_Line_Cal(2,color,bright[(cnt + 2) % 5]);
//	RGB_Line_Cal(3,color,bright[(cnt + 3) % 5]);
//	RGB_Line_Cal(4,color,bright[(cnt + 4) % 5]);
//}
//char RGB_ON;

//void RGB_Show_Msg(void){
//	//清空显示
//	RGB_Clear_Cal();
//	SwitchRGBShowMsg();
//	switch(panel.display){
//		case init_msg:
//			Check_Reset();
//		break;
//		case press_msg:
//			RGB_Color_All(Purple,30);
//			panel.display = init_msg;
//		break;
//		case flow_msg:
//			switch(flow.type){
//				case dribble_flow:
//					RGB_Wave(0xFF0000);
//				break;
//				case dunk_flow:
//					RGB_Wave(0x00FF00);
//				break;
//				case back_flow:
//					RGB_Wave(0x0000FF);
//				break;
//				case skill_flow:
//					RGB_Wave(0x00FFFF);
//				break;
//			}
//		break;
//	}	
//	RGB_OutPut();
//}

//void RGB_Total_Cal(int color,int bright){
//	for(int i = 0; i< LED_NUM; i++){
//		RGB_Cal_Color(i,color);
//	}
//}



//void RGB_Test(int index,int color){
//	RGB_Total_Cal(color,0);
//	RGB_OutPut();
//}












//char RGB_Inner_Count(char count_index,int dt){
//	int now = HAL_GetTick();
//	static int last[5];
//	if(count_index == 0){
//		if(now - last[0] > dt){
//			last[0] = now;
//			return 1;
//		}
//	}
//	else if(count_index == 1){
//		if(now - last[1] > dt){
//			last[1] = now;
//			return 1;
//		}
//	}
//	else if(count_index == 2){
//		if(now - last[2] > dt){
//			last[2] = now;
//			return 1;
//		}
//	}
//	else if(count_index == 3){
//		if(now - last[3] > dt){
//			last[3] = now;
//			return 1;
//		}
//	}
//	else if(count_index == 4){
//		if(now - last[4] > dt){
//			last[4] = now;
//			return 1;
//		}
//	}
//	return 0;
//}
//void RGB_Show_Velocity(void){
//	RGB_Clear_Cal();
//	float v = hypot(site.car.vx_gyro,site.car.vy_gyro);
//	if(v > 0.2)
//		RGB_Line_Cal(0,0xFF3F3F,2);
//	if(v > 1)
//		RGB_Line_Cal(1,0xFF2c2c,6);
//	if(v > 1.5)
//		RGB_Line_Cal(2,0xFF0000,10);
//	if(v > 2)
//		RGB_Line_Cal(3,0x8B0000,20);
//	if(v > 2.4)
//		RGB_Line_Cal(4,0x4B0000,40);
//	RGB_OutPut();
//}
//////////////////////////////////////////////////////////////////////一定要记住 先考虑line 在考虑list 对于我们的RGB来说 也就是先考虑堆叠 在考虑侧向 也就是 char * letter[5][3]
//char RGB_Order_Convert(char order){
//	 return 14 - (order / 3) - (order % 3) * 5;
//}
///*
//14 9 4
//13 8 3
//12 7 2
//11 6 1
//10 5 0
//*/
//char * Letter_Walk(char * input,int begin){
//	//从倒数第二个开始
//	static char letter[15];
//	for(int i = 0; i <5; i++){
//	   for(int j = 0; j < 3; j++){
//			 if((j + begin > 4) || (j + begin < 0)){
//				 letter[3*i + j] = 0;
//				 continue;
//			 }
//		   letter[3*i + j] = input[5*i + j + begin];
//		 }
//	}
//	return letter;
//}
//void sring_walk(char*a,char*b){
//	static int cnt;
//	


//}

//void RGB_Letter_Cal(void){
//	

//}
//void RGB_Show_Letter(char * letter,int color,int bright){
//	for(int i = 0; i< 15 ; i++)
//	  RGB_Cal_Color(RGB_Order_Convert(i), color*letter[i]);
//}


////////////////////////////////////////////////////////////////freeRtos里面调用的函数///////////////////////////////
//void RGB_Show_Warning(void){
//	static int flag;
//	static unsigned char pos[15];
//	if(hypot(site.car.vx_enc,site.car.vy_enc) > 0.3){
//		if(flag == 1)
//		  RGB_Total_Cal(Red,3);
//		else if(flag == 0)
//		  RGB_Total_Cal(Red,40);
//		flag =! flag;
//	}
//}
//int RGB_Change_Color(int color){
//	if(color == Green)
//		return Blue;
//	if(color == Blue)
//		return Red;
//	if(color == Red)
//		return Purple;
//	if(color == Purple)
//		return Pink;
//  if(color == Pink)
//		return Green;
//	if(color == 0)
//	  return Green;
//	return White;
//}
//void RGB_Breath(int bright_max,int dt){
//	static int color;
//	static char bright;
//	static char flag;
//	if(flag == 0){
//		flag = 1;
//		color = Green;
//	}
//	if(RGB_Inner_Count(0,dt) == 1){
//		if(bright >= bright_max)
//			flag = -1;
//		else if(bright <= 0)
//			flag = 1;
//		bright += flag;
//		if(bright == 0)
//		  color = RGB_Change_Color(color);
//	}
//	RGB_Total_Cal((int)color,bright);
//}
//int RGB_Flow(int bright,char dt,char clear_flag){
//	static int cnt;
//	static int color;
//	static int pos;
//	static char flag; 
//	if(flag == 0){
//		flag = 1;
//		color = Green;
//	}
//	if(RGB_Inner_Count(0,dt) == 1){
//		if(pos >= LED_NUM){
//			flag = -1;
//			color = Light_Color(RGB_Change_Color(color),20);
//		}
//		else if(pos < 0){
//			flag = 1;
//			color = Light_Color(RGB_Change_Color(color),20);
//			cnt++;
//		}
//		pos += flag;
//	}
//	if(clear_flag == 1)
//		RGB_Clear_Cal();
//	RGB_Cal_Color(pos,color);
//	RGB_Cal_Color(pos + 1,color);
//	RGB_Cal_Color(pos + 2,color);
//	RGB_Cal_Color(pos + 3,color);
//	RGB_Cal_Color(pos + 4,color);
//	return cnt;
//}
//void RGB_Flow_Circle(void){
//	

//}
//void RGB_Show_Test(int dt){
////	if(RGB_Inner_Count(1,5) == 1){
//		//RGB_Breath(50,dt);
////	  RGB_Flow(40,dt,1);
//  if(RGB_Inner_Count(1,10) == 1){
//		RGB_Flow(30,30,0);
//		RGB_OutPut();
//	}
//}

