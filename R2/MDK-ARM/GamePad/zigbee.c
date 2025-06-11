#include "zigbee.h"
#include <string.h>
#include <stdlib.h>

//#include "Config.h"

//zigbee_state_t zigbee_state = {.huart = &TX_Uart};
//struct zigbee_conf_t zigbee_conf;

uint8_t data[100];

/* 通信操作 */
void zigbee_transmit(uint8_t* Tdata, uint8_t lenth, uint16_t target_addr)
{
//	if(lenth > 94) return;
	data[0] = 0xAA; data[lenth+4] = 0x55;	// 设置起始标志符
	data[1] = MASTER_COMMU;		// 数据发送帧
	// 设置发送地址
	data[2] = target_addr >> 8; data[3] = target_addr & 0xFF;
	// 拷贝发送数据
	memcpy(&data[4], Tdata, lenth);
	//HAL_UART_Transmit_DMA(zigbee_state.huart, data, lenth+5);
}

void get_signal_strength(uint16_t target_addr)
{
	data[0] = 0xAA; data[4] = 0x55;
	data[1] = SIG_STRENGTH;		// 信号强度查询帧
	HAL_UART_Transmit_DMA(zigbee_state.huart, data, 5);
}

// val: bit0->IO0  bit1->IO1  0=Input  1=Output
void set_IO_conf(uint16_t target_addr, uint8_t val)
{
	uint8_t data[6];
	data[0] = 0xAA; data[5] = 0x55;
	data[1] = SET_IO_CONF;	// 目标IO配置帧
	// 设置发送地址
	data[2] = target_addr >> 8; data[3] = target_addr & 0xFF;
	data[4] = val;
	HAL_UART_Transmit_DMA(zigbee_state.huart, data, 6);
}

// val: bit0->IO0  bit1->IO1
void set_IO_state(uint16_t target_addr, uint8_t val)
{
	data[0] = 0xAA; data[5] = 0x55;
	data[1] = SET_IO_STATE;	// 目标IO配置帧
	// 设置发送地址
	data[2] = target_addr >> 8; data[3] = target_addr & 0xFF;
	data[4] = val;
	HAL_UART_Transmit_DMA(zigbee_state.huart, data, 6);
}

// 阻塞形式收发
// return: bit0->IO0  bit1->IO1
uint8_t get_IO_state(uint16_t target_addr)
{
	data[0] = 0xAA; data[4] = 0x55;
	data[1] = READ_IO_STATE;	// 目标IO配置帧
	// 设置发送地址
	data[2] = target_addr >> 8; data[3] = target_addr & 0xFF;
	HAL_UART_Transmit(zigbee_state.huart, data, 5, 5);
	uint8_t rec_data[6];
	HAL_UART_Receive(zigbee_state.huart, rec_data, 6, 10);
	if(rec_data[1] == READ_IO_STATE) return rec_data[4];
	else return 0xFF;
}

void module_sleep(uint32_t sleep_ms)
{
	if(sleep_ms > 100000) sleep_ms = 0;	//过大时间直接无限休眠
	data[0] = 0xAA; data[5] = 0x55;
	data[1] = DO_SLEEP;
	data[2] = 0x01;	//只支持睡眠模式1
	// 设置睡眠时间
	uint16_t sleep_10ms = sleep_ms / 10;
	data[3] = sleep_ms >> 8; data[4] = sleep_ms & 0xFF;
	zigbee_msg.is_sleeping = 1;
	HAL_UART_Transmit_DMA(zigbee_state.huart, data, 6);
}

uint8_t zigbee_decode(uint8_t* Rdata, uint8_t lenth)
{
	if(Rdata[0] == 0xAA)
	{
		if(Rdata[1] == MASTER_COMMU)	// 接收通讯数据
		{
			uint8_t datalen = lenth - 5;
			zigbee_msg.datalen = datalen;
			zigbee_state.source_addr = ((uint16_t)Rdata[2] << 8) + Rdata[3];
			memcpy(zigbee_msg.RxData, &Rdata[4], datalen);
			return MASTER_COMMU;
		}
		if(Rdata[1] == SIG_STRENGTH)	// 连接信号强度
		{
			zigbee_state.source_addr = ((uint16_t)Rdata[2] << 8) + Rdata[3];
			if(zigbee_state.source_addr == zigbee_state.MyAddr) return 0;		// 解决查询到自己的bug
			zigbee_msg.signal_strength = 0xff - Rdata[4] + 1;	// 取负后单位为dbm
			return SIG_STRENGTH;
		}
		if(Rdata[1] == DO_SLEEP)
		{
			zigbee_msg.is_sleeping = 0;
			return DO_SLEEP;
		}
	}
	else if(Rdata[0] == 0xAB)
	{
		if(Rdata[1] == READ_LOCAL_INFO && Rdata[lenth-1] == 0xBA)
		{
			zigbee_msg.info_got = 1;
			// 获取模块ID
			zigbee_state.MyAddr = (Rdata[2]<<8) + Rdata[3];
		}
		if(Rdata[1] == GET_CONF_INFO && Rdata[lenth-1] == 0xBA)
		{
			zigbee_msg.config_got = 1;
			// 拷贝结构信息
			memcpy(&zigbee_conf, &Rdata[6+Rdata[4]], Rdata[5]);
		}
	}
	return 0;
}

/* 配置操作 */
void get_localInfo(void)
{
	data[0] = 0xAB; data[1] = READ_LOCAL_INFO; data[2] = 0xBA;
	HAL_UART_Transmit_DMA(zigbee_state.huart, data, 3);
}
void get_Conf(void)
{
	// 帧头帧尾
	data[0] = 0xAB; data[6] = 0xBA;
	// 操作类型
	data[1] = GET_CONF_INFO;
	// 操作网络号
	data[2] = zigbee_state.MyAddr >> 8; data[3] = zigbee_state.MyAddr & 0xFF;
	// 偏移地址
	data[4] = 0x00;
	// 读取长度
	data[5] = sizeof(zigbee_conf);
	HAL_UART_Transmit_DMA(zigbee_state.huart, data, 7);
}
void write_Conf(void)
{
	// 帧头帧尾
	data[0] = 0xAB; data[6+sizeof(zigbee_conf)] = 0xBA;
	// 操作类型
	data[1] = WRITE_CONF;
	// 操作网络号
	data[2] = zigbee_state.MyAddr >> 8; data[3] = zigbee_state.MyAddr & 0xFF;
	// 偏移地址
	data[4] = 0x00;
	// 写入长度
	data[5] = sizeof(zigbee_conf);
	// 写入数据
	memcpy(&data[6], &zigbee_conf, sizeof(zigbee_conf));
	HAL_UART_Transmit_DMA(zigbee_state.huart, data, 7+sizeof(zigbee_conf));
}
void write_MyAddr(uint16_t MyAddr)
{
	// 帧头帧尾
	data[0] = 0xAB; data[8] = 0xBA;
	// 操作类型
	data[1] = WRITE_CONF;
	// 操作网络号
	data[2] = zigbee_state.MyAddr >> 8; data[3] = zigbee_state.MyAddr & 0xFF;
	// 偏移地址
	data[4] = 0x24;
	// 写入长度
	data[5] = 0x02;
	// 写入数据
	data[6] = MyAddr>>8; data[7] = MyAddr&0xFF;
	HAL_UART_Transmit_DMA(zigbee_state.huart, data, 9);
}
void load_Conf(void)
{
	// 配置数据
	zigbee_conf.DevMode = zigbee_state.DevMode;
	zigbee_conf.Serial_Rate = zigbee_state.Serial_Rate;
	zigbee_conf.PanID_oppo = ((zigbee_state.PanID&0xFF)<<8) + (zigbee_state.PanID>>8);
	zigbee_conf.MyAddr_oppo = ((zigbee_state.SetMyAddr&0xFF)<<8) + (zigbee_state.SetMyAddr>>8);
	zigbee_conf.DstAddr_oppo = ((zigbee_state.DstAddr&0xFF)<<8) + (zigbee_state.DstAddr>>8);
	zigbee_conf.Chan = zigbee_state.Chan;
	zigbee_conf.PowerLevel = zigbee_state.PowerLevel;
	zigbee_conf.DataRate = zigbee_state.DataRate;
	zigbee_conf.RetryNum = zigbee_state.RetryNum;
	zigbee_conf.RetryTimeout_oppo = ((zigbee_state.RetryTimeout&0xFF)<<8) + (zigbee_state.RetryTimeout>>8);
}
// 读取并写入配置，写入后重启模块生效；并且只有在460800及以下波特率才能够写入成功
void get_write_Conf(void)
{
	get_localInfo();	// 获取通信模块信息
	HAL_Delay(50);
	if(zigbee_msg.info_got == 1)	// 读取到模块信息
	{
		get_Conf();			// 获取配置
		// 下面的写入配置只有在使用波特率460800及以下时才能配置成功
		HAL_Delay(50);
		load_Conf();		// 加载配置
		write_Conf();		// 写入配置
		HAL_Delay(100);
		get_Conf();			// 再读取配置
		HAL_Delay(50);
	}
}
