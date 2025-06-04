#ifndef _HO7213_H
#define _HO7213_H

#include "fdcan.h"
#include "Can_Bsp.h"
#include "usart.h"
#include "string.h"
#include "stdbool.h"
#include <stdint.h>
#include <stdio.h>

#define Tqe_Constant 0.23  // 力矩常数
#define Spd_MAX 58.639	   // 使用CAN普通和扩展模式下的速度满量程，单位rad/s
#define Cur_MAX 10.0	   // 电流满量程，单位A
#define Motor_Pole_Pair 21 // 电机极对数
#define pi 3.1416		   // 3.1415926535......
struct HO7213{
	float target_angle;
	float angle_now;
	float offset_angle;
	bool online_flag;
	struct {
		float p;
		float d;
	}param;
};
typedef struct
{
	int64_t Position;	 // 一圈4096
	float Speed;		 // 单位rad/s
	float Torque;		 // 单位N*m
	uint8_t Mode;		 // 仅在FDCAN模式下会回传（0：力矩  1：速度  2：位置）
	uint8_t Run_status;	 // 仅在FDCAN模式下会回传（0：失能  1：使能）
	uint8_t temperature; // 仅在FDCAN模式下会回传 （单位：摄氏度）
	uint8_t ID;			 // 无单位
} HO7213_INFO;

extern uint8_t PMAX; // (范围1-5)
extern uint8_t HO7213_RXData[16]; // 电机信息结构体
extern HO7213_INFO HO7213; // 电机信息结构体

// 串口模式下配置电机选项,掉电不丢失
/*通过串口切换与配置电机上电后的初始模式，不稳定，仅供测试用*/
void Set_Motor_to_Pos_Mode_byU(UART_HandleTypeDef *huart);
void Set_Motor_to_Spd_Mode_byU(UART_HandleTypeDef *huart);
void Set_Motor_to_Tqe_Mode_byU(UART_HandleTypeDef *huart);
/*设置或清空绝对零点，重新上电后不丢失*/
void Set_Motor_ZeroPosition_byU(UART_HandleTypeDef *huart);
void Clr_Motor_ZeroPosition_byU(UART_HandleTypeDef *huart);
/*在CAN普通以及扩展模式下的控制指令中生效，也在CAN普通模式下的返回值中生效(由于CAN普通模式下初始状态只支持返回+-1圈，可修改此值使量程增大，但相应的返回值精度会下降)*/
void Rotation_Increase_byU(uint8_t magnification, UART_HandleTypeDef *huart); // 初始值为1
/*是否使用FDCAN控制*/
void Turn_On_FDCAN_Mode_byU(UART_HandleTypeDef *huart);
void Turn_Off_FDCAN_Mode_byU(UART_HandleTypeDef *huart);
/*设置电机ID与主机ID(电机返回数据的帧ID，使用上位机时需要改回默认值100)*/
void Set_Motor_ID_byU(uint8_t ID, UART_HandleTypeDef *huart);
void Set_Host_ID_byU(uint8_t ID, UART_HandleTypeDef *huart);
/*设置电流环带宽*/
void Set_Current_Bandwidth_byU(uint16_t Bandwidth, UART_HandleTypeDef *huart);
/*更改位置环Kp与Ki与输出电流限幅，仅在PWM输入控制下生效*/
void Set_Position_Kp_byU(float Kp, UART_HandleTypeDef *huart);
void Set_Position_Kd_byU(float Kd, UART_HandleTypeDef *huart);
void Set_Current_Limit_byU(float Kp, UART_HandleTypeDef *huart);

// CAN普通(标准帧)模式下控制电机选项，此模式下的所有配置(机械零点，电机模式，位置环Kp,Kd等)都会掉电丢失,电机返回数据6字节
/*启动电机(按照已经设置好的模式运行，也就是说先设置好电机的运动状态和运动参数再启动)*/
void Start_Motor_byCN(uint8_t ID, FDCAN_HandleTypeDef *hfdcan);
/*停止电机*/
void Stop_Motor_byCN(uint8_t ID, FDCAN_HandleTypeDef *hfdcan);
/*切换电机模式*/
void Set_Motor_to_Tqe_Mode_byCN(uint8_t ID, FDCAN_HandleTypeDef *hfdcan);
void Set_Motor_to_TqewithSpd_Mode_byCN(uint8_t ID, FDCAN_HandleTypeDef *hfdcan);
void Set_Motor_to_PoswithTqewithSpd_Mode_byCN(uint8_t ID, FDCAN_HandleTypeDef *hfdcan);
/*设置机械零位，清除多圈计数，掉电不保存*/
void Set_Mechanical_Zero_Position_byCN(uint8_t ID, FDCAN_HandleTypeDef *hfdcan);
/*电机控制指令*/
void Motor_Control_byCN(uint8_t ID, float Tqe, float Spd, float Pos, uint16_t Pos_Kp, uint16_t Pos_kd, FDCAN_HandleTypeDef *hfdcan);
/*设置电机进入CAN扩展模式(电机上电后初始模式为CAN普通模式)*/
void Set_Motor_to_CE(uint8_t ID, FDCAN_HandleTypeDef *hfdcan);

// CAN扩展模式(扩展帧)下控制电机选项，此模式下的所有配置(速度环、电流环的Kp，Ki,位置环Kp，Kd等)都会掉电丢失，电机返回数据8字节
/*设置电机进入CAN普通模式(电机上电后初始模式为CAN普通模式,返回数据六字节)*/
void Set_Motor_to_CN(uint8_t ID, FDCAN_HandleTypeDef *hfdcan);
/*设置速度环Kp(掉电丢失)*/
void Set_Spd_Kp_byCE(uint8_t ID, float Spd_Kp, FDCAN_HandleTypeDef *hfdcan);
/*设置速度环Ki(掉电丢失)*/
void Set_Spd_Ki_byCE(uint8_t ID, float Spd_Ki, FDCAN_HandleTypeDef *hfdcan);
/*设置电流环Kp(掉电丢失)*/
void Set_Cur_Kp_byCE(uint8_t ID, float Cur_Kp, FDCAN_HandleTypeDef *hfdcan);
/*设置电流环Ki(掉电丢失)*/
void Set_Cur_Ki_byCE(uint8_t ID, float Cur_Ki, FDCAN_HandleTypeDef *hfdcan);

// FDCAN普通模式(标准帧)下控制电机选项，此模式下的所有配置(机械零点，电机模式，位置环Kp，Kd等)都会掉电丢失
/*电机控制指令(含运动模式(0=力矩 1=速度 2=位置)，电机是否启动(先设置好电机的运动状态和运动参数再启动)，位置环Kp,Kd设置)*/
void Motor_Control_byFDCN(uint8_t ID, float Tqe, float Spd, float Pos, uint8_t Mode, uint8_t RunCmd, uint8_t Pos_Kp, uint8_t Pos_Kd, FDCAN_HandleTypeDef *hfdcan);
/*设置机械零位，清除多圈计数，掉电不保存*/
void Set_Mechanical_Zero_Position_byFDCN(uint8_t ID, FDCAN_HandleTypeDef *hfdcan);

// FDCAN扩展模式(扩展帧)下控制电机选项，此模式下的所有配置(速度环、电流环的Kp，Ki)都会掉电丢失
/*设置速度环Kp(掉电丢失)*/
void Set_Spd_Kp_byFDCE(uint8_t ID, float Spd_Kp, FDCAN_HandleTypeDef *hfdcan);
/*设置速度环Ki(掉电丢失)*/
void Set_Spd_Ki_byFDCE(uint8_t ID, float Spd_Ki, FDCAN_HandleTypeDef *hfdcan);
/*设置电流环Kp(掉电丢失)*/
void Set_Cur_Kp_byFDCE(uint8_t ID, float Cur_Kp, FDCAN_HandleTypeDef *hfdcan);
/*设置电流环Ki(掉电丢失)*/
void Set_Cur_Ki_byFDCE(uint8_t ID, float Cur_Ki, FDCAN_HandleTypeDef *hfdcan);

// 发送函数
/*CAN普通模式发送*/
void HO7213_Send_Data_byCN(uint8_t *Txdata, uint8_t Send_ID, FDCAN_HandleTypeDef *hfdcan);
/*CAN扩展模式发送*/
void HO7213_Send_Data_byCE(uint8_t *Txdata, uint8_t Send_ID, FDCAN_HandleTypeDef *hfdcan);
/*FDCAN普通模式发送*/
void HO7213_Send_Data_byFDCN(uint8_t *Txdata, uint8_t Send_ID, FDCAN_HandleTypeDef *hfdcan);
/*FDCAN扩展模式发送*/
void HO7213_Send_Data_byFDCE(uint8_t *Txdata, uint8_t Send_ID, FDCAN_HandleTypeDef *hfdcan);

// 电机回传数据解码函数
/*CAN普通模式下数据解码(6字节)*/
void HO7213_Decode_for_CN(uint8_t *RxData);
/*CAN扩展模式下数据解码(8字节)*/
void HO7213_Decode_for_CE(uint8_t *RxData);
/*FDCAN模式下数据解码(普通和扩展模式下的回传数据均为16字节，解码函数相同)*/
float HO7213_Decode_for_FDC(uint8_t *RxData);

#endif
