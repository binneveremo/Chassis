#include "NetWork.h"


#define send_uart huart1

#ifdef Carbon_Car
#define R1_Exchange_Usart huart3
#define R1_Exchange_Baudrate 115200
#else 
#define R1_Exchange_Usart huart3
#define R1_Exchange_Baudrate 115200
#endif

struct Send send;
struct R1_t shoot;
void Send_Float_Data(char num){
	float total; 
	for(unsigned char i = 0; i < num;i++)
		total += send.convert.float_data[i];
	//计算最后一位 也就是校验位的数值
	send.convert.float_data[num] = total;
	//对发送的字节进行赋值
	memcpy(send.Debug.send,send.convert.uint8_data,(num + 1) * 4);
	//发送
	HAL_UART_Transmit(&send_uart, (unsigned char*)send.Debug.send, (num + 1) * 4, HAL_MAX_DELAY);
}
/////////////////////////////////////////////无线网条所使用的程序////////////////////////////
void Wireless_init(void){
	//R1_Exchange_Usart.Init.BaudRate = R1_Exchange_Baudrate;
	///////////////初始化R1发送的串口
	HAL_UART_Init(&R1_Exchange_Usart);
	__HAL_UART_ENABLE_IT(&R1_Exchange_Usart, UART_IT_IDLE);
	HAL_UARTEx_ReceiveToIdle_DMA(&R1_Exchange_Usart, shoot.receive.buffer, sizeof(shoot.receive.buffer));
	__HAL_DMA_DISABLE_IT(R1_Exchange_Usart.hdmarx, DMA_IT_HT);  // 禁用传输过半中断
}
void R1ExchangeData_Decode(UART_HandleTypeDef *huart){
	static int R1_Shooted_FlagLast;
	if(huart->Instance != R1_Exchange_Usart.Instance)
		return;
	shoot.deal.getdata_flag = true;
	struct __attribute__((packed)) {
		unsigned char header;
		float positionx;
		float positiony;
		float shoot_rpm;
		float ball_incar_time;
		char shoot_flag;
		unsigned char check;
	}format;
	memcpy((unsigned char *)&format,shoot.receive.buffer, sizeof(shoot.receive.buffer));
	HAL_UARTEx_ReceiveToIdle_DMA(&R1_Exchange_Usart, shoot.receive.buffer, sizeof(shoot.receive.buffer));
	
	shoot.receive.shoot_pos.x = format.positionx * 1000;
	shoot.receive.shoot_pos.y = format.positiony * 1000;
	shoot.receive.shoot_rpm = format.shoot_rpm;
	shoot.receive.shoot_incar_time = format.ball_incar_time * 1000;
	shoot.receive.shoot_flag = format.shoot_flag;
	//////////计算部分///////////////////
	shoot.deal.opposite_angle = rad2ang(atan2f(shoot.receive.shoot_pos.y - site.now.y,shoot.receive.shoot_pos.x - site.now.x)) + 180;
	shoot.receive.shoot_pos.r = shoot.deal.opposite_angle;
	shoot.deal.distance = hypot(shoot.receive.shoot_pos.y - site.now.y,shoot.receive.shoot_pos.x - site.now.x);
	flow.flagof.R1_Shooted = (shoot.receive.shoot_flag == 1)?true:flow.flagof.R1_Shooted;
	/////////识别发射//////////////////////
	if((HAL_GetTick() - shoot.deal.shoot_begin > 1500) && (flow.flagof.R1_Shooted == true))
		shoot.deal.shoot_begin = HAL_GetTick(),shoot.expect.ball_fly_time = Polynomial_4ExpectBallFlyingTime(shoot.deal.distance);
	interact.flagof.R1_shooted = (shoot.receive.shoot_flag == 1)?true:interact.flagof.R1_shooted;
}
unsigned char R1Data_Sum(unsigned char * data){
	unsigned char sum = NONE;
	for(unsigned char i = 0;i < R1_Data_Num - 1;i++)
		sum += data[i];
	return sum;
}

void Set_SendMode(struct Point * target,char * mode,bool request){
	static struct Point * send;
	send = target;
	
	shoot.send.target = target;
	
	
	shoot.send.mode = (strcmp(mode,"once") == 0)?send_once:shoot.send.mode;
	shoot.send.mode = (strcmp(mode,"real") == 0)?send_real_time:shoot.send.mode;
	shoot.send.flagof.request = request;
}

void Send_MessageToR1(void){
	struct __attribute__((packed)) {
		unsigned char header;
		float net_x;
		float net_y;
		float basket_x;
		float basket_y;
		unsigned char arrive_dunkpoint:2;
		unsigned char net_status:2;
		unsigned char request_flag:4;
		unsigned char check;
	}format;
	char net_Status = (interact.defend_status == defend)?1:0;
	int net_offset = (interact.defend_status == defend)?322:40;
	float netx = net_offset * cos(ang2rad(site.now.r));
	float nety = net_offset * sin(ang2rad(site.now.r));
	format.header = 0xAA;
	format.net_x = shoot.send.target->x + netx;
	format.net_y = shoot.send.target->y + nety;
	format.basket_x = vision.visual.basket_visual.x;
	format.basket_y = vision.visual.basket_visual.y;
	format.arrive_dunkpoint = (chassis.Control_Status == Auto_Control)?skill.flagof.change_flag:basketlock.flagof.dunk_position;
	format.request_flag = shoot.send.flagof.request;
	format.net_status = (flow.type == skill_flow)?0:net_Status;
	format.check = R1Data_Sum((unsigned char *)&format);
	memcpy(shoot.send.buffer,(unsigned char *)&format,R1_Data_Num);
	
	switch(shoot.send.mode){
		case send_real_time:
			HAL_UART_Transmit(&R1_Exchange_Usart, shoot.send.buffer, R1_Data_Num, HAL_MAX_DELAY);
		break;
		case send_once:
			for(unsigned char i = 0;i < 5; i++) HAL_UART_Transmit(&R1_Exchange_Usart, shoot.send.buffer, R1_Data_Num, HAL_MAX_DELAY);
			shoot.send.mode = send_none;
		break;
		default:
		break;
	}
}
void BallTime_Cal(void){
	shoot.expect.ball_fly_time = Polynomial_4ExpectBallFlyingTime(shoot.deal.distance);
	shoot.expect.ball_incar_time= Polynomial_4ExpectBallIncarTime(shoot.deal.distance);
	shoot.expect.ball_total_time = shoot.expect.ball_incar_time + shoot.expect.ball_fly_time - shoot.expect.ball_offset_time;
}




float Expect_BallFlyingTime(float distance,bool real){
	double k = 0;
	double ratio = 0.009788;
	//计算曾曾拟合的期望速度
	float shoot_velocity;
	float dist_cm = distance / 10;
	if (dist_cm <= 275)
		shoot_velocity = dist_cm + 325;
	else if (dist_cm <= 350)
      shoot_velocity = -0.0056000000000007155 * pow(dist_cm, 2) + 4.3800000000003365 * dist_cm -187.00000000003354;
    else if (dist_cm <= 550)
      shoot_velocity = 0.6 * dist_cm + 450;
    else
      shoot_velocity = 0.48 * dist_cm + 516;
	if(real == true)
		shoot_velocity = shoot.receive.shoot_rpm * ratio;
	return (distance / (shoot_velocity * cos(ang2rad(65))) * (1 + (k * distance) / (pow(shoot_velocity,2))));
}
double Polynomial_4ExpectBallFlyingTime(float distance){
#define P1 5.5411e-09
#define P2 -7.8492e-05
#define P3 0.5138
#define P4 183.4338
	return P1 * pow(distance,3) + P2 * pow(distance,2) + P3 * distance + P4 ;
#undef P1 
#undef P2 
#undef P3 
#undef P4 
}
double Polynomial_4ExpectBallIncarTime(float distance){
#define P1 1.2169e-09
#define P2 -9.1970e-06
#define P3 -0.0191
#define P4 404.6825
	return P1 * pow(distance,3) + P2 * pow(distance,2) + P3 * distance + P4 ;
}
double Polynomial_4ExpectBallTotalTime(float distance){
	return Polynomial_4ExpectBallFlyingTime(distance) + Polynomial_4ExpectBallIncarTime(distance);
}
//void Set_SendModeAuto(void){
//	if((chassis.Control_Status == Auto_Control) && (flow.type) )

//}


















