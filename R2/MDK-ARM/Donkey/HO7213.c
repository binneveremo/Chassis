#include "HO7213.h"
#include "Global.h"
uint8_t HO7213_RXData[16] = {0}; // 电机回传数据
HO7213_INFO HO7213;				 // 电机信息结构体


/*在CAN普通以及扩展模式下的控制指令中生效，也在CAN普通模式下的返回值中生效(由于CAN普通模式下初始状态只支持返回+-1圈，可修改此值使量程增大，但相应的返回值精度会下降)*/
uint8_t PMAX = 1; //(范围1-5)

void Start_Motor_byCN(uint8_t ID, FDCAN_HandleTypeDef *hfdcan)
{
	uint8_t Txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
	HO7213_Send_Data_byCN(Txdata, ID, hfdcan);
}

/*停止电机*/
void Stop_Motor_byCN(uint8_t ID, FDCAN_HandleTypeDef *hfdcan)
{
	uint8_t Txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
	HO7213_Send_Data_byCN(Txdata, ID, hfdcan);
}

/*切换电机模式*/
void Set_Motor_to_Tqe_Mode_byCN(uint8_t ID, FDCAN_HandleTypeDef *hfdcan)
{
	uint8_t Txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF9};
	HO7213_Send_Data_byCN(Txdata, ID, hfdcan);
}

void Set_Motor_to_TqewithSpd_Mode_byCN(uint8_t ID, FDCAN_HandleTypeDef *hfdcan)
{
	uint8_t Txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFA};
	HO7213_Send_Data_byCN(Txdata, ID, hfdcan);
}

void Set_Motor_to_PoswithTqewithSpd_Mode_byCN(uint8_t ID, FDCAN_HandleTypeDef *hfdcan)
{
	uint8_t Txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
	HO7213_Send_Data_byCN(Txdata, ID, hfdcan);
}

/*设置机械零位，清除多圈计数，掉电不保存*/
void Set_Mechanical_Zero_Position_byCN(uint8_t ID, FDCAN_HandleTypeDef *hfdcan)
{
	uint8_t Txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
	HO7213_Send_Data_byCN(Txdata, ID, hfdcan);
}

/*电机控制指令(Tqe单位N*M,Spd单位rad/s,Pos单位1/4096)*/
void Motor_Control_byCN(uint8_t ID, float Tqe, float Spd, float Pos, uint16_t Pos_Kp, uint16_t Pos_kd, FDCAN_HandleTypeDef *hfdcan)
{
	uint16_t Tx_Cur = ((Tqe / Tqe_Constant) / 10) * 0x800 + 0x800;
	uint16_t Tx_Spd = (Spd / Spd_MAX) * 0x800 + 0x800;
	uint16_t Tx_Pos = (Pos / (PMAX * 4096)) * 0x8000 + 0x8000;
	uint8_t Txdata[8] = {0};
	Txdata[0] = Tx_Pos >> 8;
	Txdata[1] = Tx_Pos & 0xFF;
	Txdata[2] = Tx_Spd >> 4;
	Txdata[3] = ((Tx_Spd & 0x0F) << 4) | (Pos_Kp >> 8);
	Txdata[4] = Pos_Kp & 0xFF;
	Txdata[5] = Pos_kd >> 4;
	Txdata[6] = ((Pos_kd & 0x0F) << 4) | (Tx_Cur >> 8);
	Txdata[7] = Tx_Cur & 0xFF;
	HO7213_Send_Data_byCN(Txdata, ID, hfdcan);
}

/*设置电机进入CAN扩展模式(电机上电后初始模式为CAN普通模式)*/
void Set_Motor_to_CE(uint8_t ID, FDCAN_HandleTypeDef *hfdcan)
{
	uint8_t Txdata[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	HO7213_Send_Data_byCE(Txdata, ID, hfdcan);
}

// CAN扩展模式(扩展帧)下控制电机选项，此模式下的所有配置(如速度环、电流环的Kp，Ki)都会掉电丢失
/*设置电机进入CAN普通模式(电机上电后初始模式为CAN普通模式)*/
void Set_Motor_to_CN(uint8_t ID, FDCAN_HandleTypeDef *hfdcan)
{
	uint8_t Txdata[8] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	HO7213_Send_Data_byCE(Txdata, ID, hfdcan);
}

/*设置速度环Kp(掉电丢失)*/
void Set_Spd_Kp_byCE(uint8_t ID, float Spd_Kp, FDCAN_HandleTypeDef *hfdcan)
{
	uint32_t I754_Val = 0;
	memcpy(&I754_Val, &Spd_Kp, sizeof(Spd_Kp));
	uint8_t Txdata[8] = {0};
	Txdata[0] = 0x00;
	Txdata[1] = 0x01;
	Txdata[2] = (I754_Val >> 24);
	Txdata[3] = ((I754_Val >> 16) & 0xFF);
	Txdata[4] = ((I754_Val >> 8) & 0xFF);
	Txdata[5] = (I754_Val & 0xFF);
	Txdata[6] = 0x00;
	Txdata[7] = 0x00;
	HO7213_Send_Data_byCE(Txdata, ID, hfdcan);
}

/*设置速度环Ki(掉电丢失)*/
void Set_Spd_Ki_byCE(uint8_t ID, float Spd_Ki, FDCAN_HandleTypeDef *hfdcan)
{
	uint32_t I754_Val = 0;
	memcpy(&I754_Val, &Spd_Ki, sizeof(Spd_Ki));
	uint8_t Txdata[8] = {0};
	Txdata[0] = 0x00;
	Txdata[1] = 0x02;
	Txdata[2] = (I754_Val >> 24);
	Txdata[3] = ((I754_Val >> 16) & 0xFF);
	Txdata[4] = ((I754_Val >> 8) & 0xFF);
	Txdata[5] = (I754_Val & 0xFF);
	Txdata[6] = 0x00;
	Txdata[7] = 0x00;
	HO7213_Send_Data_byCE(Txdata, ID, hfdcan);
}

/*设置电流环Kp(掉电丢失)*/
void Set_Cur_Kp_byCE(uint8_t ID, float Cur_Kp, FDCAN_HandleTypeDef *hfdcan)
{
	uint32_t I754_Val = 0;
	memcpy(&I754_Val, &Cur_Kp, sizeof(Cur_Kp));
	uint8_t Txdata[8] = {0};
	Txdata[0] = 0x00;
	Txdata[1] = 0x03;
	Txdata[2] = (I754_Val >> 24);
	Txdata[3] = ((I754_Val >> 16) & 0xFF);
	Txdata[4] = ((I754_Val >> 8) & 0xFF);
	Txdata[5] = (I754_Val & 0xFF);
	Txdata[6] = 0x00;
	Txdata[7] = 0x00;
	HO7213_Send_Data_byCE(Txdata, ID, hfdcan);
}

/*设置电流环Ki(掉电丢失)*/
void Set_Cur_Ki_byCE(uint8_t ID, float Cur_Ki, FDCAN_HandleTypeDef *hfdcan)
{
	uint32_t I754_Val = 0;
	memcpy(&I754_Val, &Cur_Ki, sizeof(Cur_Ki));
	uint8_t Txdata[8] = {0};
	Txdata[0] = 0x00;
	Txdata[1] = 0x04;
	Txdata[2] = (I754_Val >> 24);
	Txdata[3] = ((I754_Val >> 16) & 0xFF);
	Txdata[4] = ((I754_Val >> 8) & 0xFF);
	Txdata[5] = (I754_Val & 0xFF);
	Txdata[6] = 0x00;
	Txdata[7] = 0x00;
	HO7213_Send_Data_byCE(Txdata, ID, hfdcan);
}

// FDCAN普通模式(标准帧)下控制电机选项，此模式下的所有配置(机械零点，电机模式，位置环Kp，Kd等)都会掉电丢失
/*电机控制指令(含运动模式(Mode0=力矩 1=速度 2=位置)，电机启动(RunCmd=0停止 1启动)，位置环Kp,Kd设置)(Tqe单位N*M,Spd单位rad/s,Pos单位1/4096)*/
void Motor_Control_byFDCN(uint8_t ID, float Tqe, float Spd, float Pos, uint8_t Mode, uint8_t RunCmd, uint8_t Pos_Kp, uint8_t Pos_Kd, FDCAN_HandleTypeDef *hfdcan)
{
	uint8_t Txdata[16] = {0};
	int64_t Tx_Pos = (Pos / 360) * (1 << 20);
	int32_t Tx_Spd = ((Motor_Pole_Pair * Spd) / (2000 * pi)) * (1 << 23);
	int16_t Tx_Tqe = (Tqe / (Tqe_Constant * 100)) * (1 << 15);
	Txdata[0] = Tx_Pos >> 32;
	Txdata[1] = (Tx_Pos >> 24) & 0xFF;
	Txdata[2] = (Tx_Pos >> 16) & 0xFF;
	Txdata[3] = (Tx_Pos >> 8) & 0xFF;
	Txdata[4] = Tx_Pos & 0xFF;
	Txdata[5] = Tx_Spd >> 24;
	Txdata[6] = (Tx_Spd >> 16) & 0xFF;
	Txdata[7] = (Tx_Spd >> 8) & 0xFF;
	Txdata[8] = Tx_Spd & 0xFF;
	Txdata[9] = Tx_Tqe >> 8;
	Txdata[10] = Tx_Tqe & 0xFF;
	Txdata[11] = Mode;
	Txdata[12] = RunCmd;
	Txdata[13] = Pos_Kp;
	Txdata[14] = Pos_Kd;
	Txdata[15] = 0;
	HO7213_Send_Data_byFDCN(Txdata, ID, hfdcan);
}

/*设置机械零位，清除多圈计数，掉电不保存*/
void Set_Mechanical_Zero_Position_byFDCN(uint8_t ID, FDCAN_HandleTypeDef *hfdcan)
{
	uint8_t Txdata[16] = {0};
	Txdata[15] = 1;
	HO7213_Send_Data_byFDCN(Txdata, ID, hfdcan);
}

// FDCAN扩展模式(扩展帧)下控制电机选项，此模式下的所有配置(速度环、电流环的Kp，Ki)都会掉电丢失
/*设置速度环Kp(掉电丢失)*/
void Set_Spd_Kp_byFDCE(uint8_t ID, float Spd_Kp, FDCAN_HandleTypeDef *hfdcan)
{
	uint32_t I754_Val = 0;
	memcpy(&I754_Val, &Spd_Kp, sizeof(Spd_Kp));
	uint8_t Txdata[8] = {0};
	Txdata[0] = 0x00;
	Txdata[1] = 0x01;
	Txdata[2] = (I754_Val >> 24);
	Txdata[3] = ((I754_Val >> 16) & 0xFF);
	Txdata[4] = ((I754_Val >> 8) & 0xFF);
	Txdata[5] = (I754_Val & 0xFF);
	Txdata[6] = 0x00;
	Txdata[7] = 0x00;
	HO7213_Send_Data_byFDCE(Txdata, ID, hfdcan);
}

/*设置速度环Ki(掉电丢失)*/
void Set_Spd_Ki_byFDCE(uint8_t ID, float Spd_Ki, FDCAN_HandleTypeDef *hfdcan)
{
	uint32_t I754_Val = 0;
	memcpy(&I754_Val, &Spd_Ki, sizeof(Spd_Ki));
	uint8_t Txdata[8] = {0};
	Txdata[0] = 0x00;
	Txdata[1] = 0x02;
	Txdata[2] = (I754_Val >> 24);
	Txdata[3] = ((I754_Val >> 16) & 0xFF);
	Txdata[4] = ((I754_Val >> 8) & 0xFF);
	Txdata[5] = (I754_Val & 0xFF);
	Txdata[6] = 0x00;
	Txdata[7] = 0x00;
	HO7213_Send_Data_byFDCE(Txdata, ID, hfdcan);
}

/*设置电流环Kp(掉电丢失)*/
void Set_Cur_Kp_byFDCE(uint8_t ID, float Cur_Kp, FDCAN_HandleTypeDef *hfdcan)
{
	uint32_t I754_Val = 0;
	memcpy(&I754_Val, &Cur_Kp, sizeof(Cur_Kp));
	uint8_t Txdata[8] = {0};
	Txdata[0] = 0x00;
	Txdata[1] = 0x03;
	Txdata[2] = (I754_Val >> 24);
	Txdata[3] = ((I754_Val >> 16) & 0xFF);
	Txdata[4] = ((I754_Val >> 8) & 0xFF);
	Txdata[5] = (I754_Val & 0xFF);
	Txdata[6] = 0x00;
	Txdata[7] = 0x00;
	HO7213_Send_Data_byFDCE(Txdata, ID, hfdcan);
}

/*设置电流环Ki(掉电丢失)*/
void Set_Cur_Ki_byFDCE(uint8_t ID, float Cur_Ki, FDCAN_HandleTypeDef *hfdcan)
{
	uint32_t I754_Val = 0;
	memcpy(&I754_Val, &Cur_Ki, sizeof(Cur_Ki));
	uint8_t Txdata[8] = {0};
	Txdata[0] = 0x00;
	Txdata[1] = 0x04;
	Txdata[2] = (I754_Val >> 24);
	Txdata[3] = ((I754_Val >> 16) & 0xFF);
	Txdata[4] = ((I754_Val >> 8) & 0xFF);
	Txdata[5] = (I754_Val & 0xFF);
	Txdata[6] = 0x00;
	Txdata[7] = 0x00;
	HO7213_Send_Data_byFDCE(Txdata, ID, hfdcan);
}

// 发送函数
/*CAN普通模式发送*/
void HO7213_Send_Data_byCN(uint8_t *Txdata, uint8_t Send_ID, FDCAN_HandleTypeDef *hfdcan)
{
	FDCAN_Send(hfdcan,Send_ID,"STD",Txdata,"CLASSIC",8,"OFF");
}

/*CAN扩展模式发送*/
void HO7213_Send_Data_byCE(uint8_t *Txdata, uint8_t Send_ID, FDCAN_HandleTypeDef *hfdcan)
{
	FDCAN_Send(hfdcan,Send_ID,"EXT",Txdata,"CLASSIC",8,"OFF");
}

/*FDCAN普通模式发送*/
void HO7213_Send_Data_byFDCN(uint8_t *Txdata, uint8_t Send_ID, FDCAN_HandleTypeDef *hfdcan)
{
	FDCAN_Send(hfdcan,Send_ID,"STD",Txdata,"FD",16,"OFF");
}

/*FDCAN扩展模式发送*/
void HO7213_Send_Data_byFDCE(uint8_t *Txdata, uint8_t Send_ID, FDCAN_HandleTypeDef *hfdcan)
{
	FDCAN_Send(hfdcan,Send_ID,"EXT",Txdata,"FD",16,"OFF");
}

// 电机回传数据解码函数
/*CAN普通模式下数据解码(6字节)*/
void HO7213_Decode_for_CN(uint8_t *RxData)
{
	memcpy(HO7213_RXData, RxData, 6);
	HO7213.Position = ((float)(((HO7213_RXData[1] << 8) + HO7213_RXData[2]) - 0x8000) / 0x8000) * 4096 * PMAX;
	HO7213.Speed = ((float)((HO7213_RXData[3] << 4) + (HO7213_RXData[4] >> 4) - 0x800) / 0x800) * Spd_MAX;
	HO7213.Torque = ((float)(((HO7213_RXData[4] & 0x0F) << 8) + HO7213_RXData[5] - 0x800) / 0x800) * Cur_MAX * Tqe_Constant;
	HO7213.Mode = NONE;
	HO7213.Run_status = NONE;
	HO7213.temperature = NONE;
	HO7213.ID = HO7213_RXData[0];
}
/*CAN扩展模式下数据解码(8字节)*/
void HO7213_Decode_for_CE(uint8_t *RxData)
{
	memcpy(HO7213_RXData, RxData, 8);
	HO7213.Position = (((float)((HO7213_RXData[6] << 24) + (HO7213_RXData[7] << 16) + (HO7213_RXData[1] << 8) + HO7213_RXData[2]) / (1 << 20)) - 2048) * 4096;
	HO7213.Speed = ((float)((HO7213_RXData[3] << 4) + (HO7213_RXData[4] >> 4) - 0x800) / 0x800) * Spd_MAX;
	HO7213.Torque = ((float)(((HO7213_RXData[4] & 0x0F) << 8) + HO7213_RXData[5] - 0x800) / 0x800) * Cur_MAX * Tqe_Constant;
	HO7213.Mode = NONE;
	HO7213.Run_status = NONE;
	HO7213.temperature = NONE;
	HO7213.ID = HO7213_RXData[0];
}
/*FDCAN模式下数据解码(普通和扩展模式下的回传数据均为16字节，解码函数相同)*/
float HO7213_Decode_for_FDC(unsigned char *RxData)
{
	memcpy(HO7213_RXData, RxData, 16);
	if ((HO7213_RXData[0] >> 7) == 0)
		HO7213.Position = (long double)(((int64_t)0x000000 << 40) + ((int64_t)HO7213_RXData[0] << 32) + ((int64_t)HO7213_RXData[1] << 24) + ((int64_t)HO7213_RXData[2] << 16) + ((int64_t)HO7213_RXData[3] << 8) + ((int64_t)HO7213_RXData[4])) / (1 << 20) * 360;
	else if ((HO7213_RXData[0] >> 7) == 1)
		HO7213.Position = (long double)(((int64_t)0xFFFFFF << 40) + ((int64_t)HO7213_RXData[0] << 32) + ((int64_t)HO7213_RXData[1] << 24) + ((int64_t)HO7213_RXData[2] << 16) + ((int64_t)HO7213_RXData[3] << 8) + ((int64_t)HO7213_RXData[4])) / (1 << 20) * 360;
	HO7213.Speed = (((float)(((int32_t)HO7213_RXData[5] << 24) + ((int32_t)HO7213_RXData[6] << 16) + ((int32_t)HO7213_RXData[7] << 8) + (int32_t)HO7213_RXData[8]) / (1 << 23)) * 1000 * 60 / Motor_Pole_Pair) * (2 * pi / 60);
	HO7213.Torque = (int16_t)((HO7213_RXData[9] << 8) + HO7213_RXData[10]) / 32768.f * 100 * Tqe_Constant;
	HO7213.Mode = HO7213_RXData[11];
	HO7213.Run_status = HO7213_RXData[12];
	HO7213.temperature = HO7213_RXData[14];
	HO7213.ID = HO7213_RXData[15];
	return HO7213.Position;
}