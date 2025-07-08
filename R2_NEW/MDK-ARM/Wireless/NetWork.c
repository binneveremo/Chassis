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
	HAL_UARTEx_ReceiveToIdle_DMA(&R1_Exchange_Usart, send.R1_Exchange.receive, sizeof(send.R1_Exchange.receive));
	__HAL_DMA_DISABLE_IT(R1_Exchange_Usart.hdmarx, DMA_IT_HT);  // 禁用传输过半中断
}
void R1ExchangeData_Decode(UART_HandleTypeDef *huart){
	static int R1_Shooted_FlagLast;
	if(huart->Instance == R1_Exchange_Usart.Instance){
		send.R1_Exchange.get_dataflag = true;
		HAL_UARTEx_ReceiveToIdle_DMA(&R1_Exchange_Usart, send.R1_Exchange.receive, sizeof(send.R1_Exchange.receive));
		memcpy(send.convert.uint8_data,send.R1_Exchange.receive + 1, sizeof(send.R1_Exchange.receive) - 1);
		send.R1_Exchange.pos.x = send.convert.float_data[0] * 1000;
		send.R1_Exchange.pos.y = send.convert.float_data[1] * 1000;
		send.R1_Exchange.shoot_rpm = send.convert.float_data[2] * 1000;
		send.R1_Exchange.pos.r = rad2ang(atan2f(send.R1_Exchange.pos.y - site.now.y,send.R1_Exchange.pos.x - site.now.x)) + 180;
		send.R1_Exchange.distance = hypot(send.R1_Exchange.pos.y - site.now.y,send.R1_Exchange.pos.x - site.now.x);
		flow.flagof.R1_Shooted = (send.R1_Exchange.receive[13] == 1)?true:flow.flagof.R1_Shooted;
		if((HAL_GetTick() - send.R1_Exchange.shoot_begin > 2000) && (flow.flagof.R1_Shooted == true))  send.R1_Exchange.shoot_begin = HAL_GetTick(),flow.flagof.R1_Shooted = false;
		interact.flagof.R1_shooted = (send.R1_Exchange.receive[13] == 1)?true:interact.flagof.R1_shooted;
	}
}
unsigned char R1Data_Sum(void){
	char sum = NONE;
	for(unsigned char i=0;i < 10;i++)
		sum += send.R1_Exchange.send[i];
	return sum;
}
void Send_MessageToR1(void){
#define Request_Flag 2
#define NetHigh_Flag 1
#define NetLow_Flag 0
	char net_Status = (interact.defend_status == defend)?1:0;
	int net_offset = (interact.defend_status == defend)?322:40;
	float netx = net_offset * cos(ang2rad(site.now.r));
	float nety = net_offset * sin(ang2rad(site.now.r));
	
	send.R1_Exchange.send[0] = 0xAA;
	if((chassis.Control_Status == Auto_Control) && (flow.type == skill_flow)){
		send.R1_Exchange.net.x = skill.target.point[skill.success_time].x + netx;
		send.R1_Exchange.net.y = skill.target.point[skill.success_time].y + nety;
		send.R1_Exchange.send[9] = (send.R1_Exchange.request_flag == true)?Request_Flag:net_Status;
	}
	else if((chassis.Control_Status == Auto_Control) && (flow.type == attack_flow)){
		send.R1_Exchange.net.x = basketlock.target.global.x + netx;
		send.R1_Exchange.net.y = basketlock.target.global.y + nety;
		send.R1_Exchange.send[9] = (send.R1_Exchange.request_flag == true)?Request_Flag:net_Status;
	}
	else{
		send.R1_Exchange.net.x = vision.field.carcenter_fieldinterp.x + netx;
		send.R1_Exchange.net.y = vision.field.carcenter_fieldinterp.y + nety;
		send.R1_Exchange.send[9] = net_Status;
	}
	send.convert.float_data[0] = send.R1_Exchange.net.x;
	send.convert.float_data[1] = send.R1_Exchange.net.y;
	memcpy(&send.R1_Exchange.send[1],send.convert.uint8_data,8);
	send.R1_Exchange.send[10] = R1Data_Sum();
	HAL_UART_Transmit(&R1_Exchange_Usart, send.R1_Exchange.send, R1_Data_Num, HAL_MAX_DELAY);
}

float Expect_BallFlyingTime(float distance){
	double k = 0.093518;
	double ratio = 0.010937;
	float shoot_velocity = send.R1_Exchange.shoot_rpm * ratio;
	return (distance / (shoot_velocity * cos(ang2rad(65))) * (1 + (k * distance) / (pow(shoot_velocity,2))));
}








