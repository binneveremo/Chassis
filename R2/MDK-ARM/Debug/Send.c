#include "Send.h"


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
	//计算前几位的值
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
	if(huart->Instance == R1_Exchange_Usart.Instance){
		send.R1_Exchange.get_dataflag = true;
		HAL_UARTEx_ReceiveToIdle_DMA(&R1_Exchange_Usart, send.R1_Exchange.receive, sizeof(send.R1_Exchange.receive));
		memcpy(send.convert.uint8_data,send.R1_Exchange.receive + 1, sizeof(send.R1_Exchange.receive) - 1);
		send.R1_Exchange.pos.x = send.convert.float_data[0] * 1000;
		send.R1_Exchange.pos.y = send.convert.float_data[1] * 1000;
		send.R1_Exchange.pos.r = rad2ang(atan2f(send.R1_Exchange.pos.y - site.now.y,send.R1_Exchange.pos.x - site.now.x)) + 180;
		flow.flagof.R1_Shooted = (send.R1_Exchange.receive[9] == 1)?true:flow.flagof.R1_Shooted;
	}
}

unsigned char R1Data_Sum(void){
	char sum = NONE;
	for(unsigned char i=0;i < 10;i++)
		sum += send.R1_Exchange.send[i];
	return sum;
}
void Send_MessageToR1(void){	
	char net_Status = (interact.defend_status == defend)?1:0;
	int net_offset = (interact.defend_status == defend)?289:80;
	send.R1_Exchange.send[0] = 0xAA;
	if((chassis.Control_Status == Auto_Control) && (flow.type == skill_flow)){
		send.R1_Exchange.net.x = skill.target.point[skill.success_time].x + net_offset * cos(ang2rad(site.now.r));
		send.R1_Exchange.net.y = skill.target.point[skill.success_time].y + net_offset * sin(ang2rad(site.now.r));
		send.R1_Exchange.send[9] = (send.R1_Exchange.request_flag == true)?2:net_Status;
	}
	else{
		send.R1_Exchange.net.x = vision.field.carcenter_fieldinterp.x + net_offset * cos(ang2rad(site.now.r));
		send.R1_Exchange.net.y = vision.field.carcenter_fieldinterp.y + net_offset * sin(ang2rad(site.now.r));
		send.R1_Exchange.send[9] = net_Status;
	}
	send.convert.float_data[0] = send.R1_Exchange.net.x;
	send.convert.float_data[1] = send.R1_Exchange.net.y;
	memcpy(&send.R1_Exchange.send[1],send.convert.uint8_data,8);
	send.R1_Exchange.send[10] = R1Data_Sum();
	HAL_UART_Transmit(&R1_Exchange_Usart, send.R1_Exchange.send, R1_Data_Num, HAL_MAX_DELAY);
}









