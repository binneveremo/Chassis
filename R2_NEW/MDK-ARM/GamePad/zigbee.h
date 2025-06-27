#pragma once
#include "main.h"
#include <stdio.h>

//#define peer_addr 0x8001
//#define peer_addr 0x8002

// frame type
#define GET_STATE		0xD0
#define MASTER_COMMU	0xD1
#define TEMP_RATE		0xD2
#define SIG_STRENGTH	0xD3
#define SET_IO_CONF		0xD4
#define SET_IO_STATE	0xD5
#define READ_IO_STATE	0xD6
#define READ_AD			0xD7
#define DO_SLEEP		0xD8

// state callback
#define COMMAND_OK		0x00
#define ADDRESS_FAUSE	0x01
#define LENGTH_FAUSE	0x02
#define CHECK_FAUSE		0x03
#define WRITE_FAUSE		0x04
#define OTHER_FAUSE		0x05
#define OTHER_ERR		0x06
#define CHAN_ERR		0x07
#define RATE_ERR		0x08
#define ID_ERR			0x09
#define WORKMODE_ERR	0x0A
#define PARAMETER_ERR	0x0B
#define SEND_DATA_TIMEOUT	0xA4
#define DEVICE_BUSY		0xA5S

// config
#define READ_LOCAL_INFO	0xD1
#define GET_CONF_INFO	0xD2
#define WRITE_CONF		0xD3
#define SET_CHAN_RATE	0xD4
#define SEARCH_DEVICE	0xD5
#define PING_ID			0xD6
#define RESET			0xD7
#define RESTORE			0xDB

typedef struct
{
	uint8_t RxData[100];
	uint8_t datalen;
	uint8_t signal_strength;
	uint8_t is_sleeping;
	uint8_t info_got;
	uint8_t config_got;
}zigbee_msg_t;
extern zigbee_msg_t zigbee_msg;

typedef struct
{
	UART_HandleTypeDef* huart;
	uint8_t DevMode;
	uint8_t Serial_Rate;
	uint16_t PanID;
	uint16_t SetMyAddr;
	uint16_t MyAddr;
	uint16_t DstAddr;
	uint16_t source_addr;
	uint8_t Chan;
	uint8_t DataRate;
	uint8_t RetryNum;
	uint16_t RetryTimeout;
	uint8_t PowerLevel;
}zigbee_state_t;
extern zigbee_state_t zigbee_state;

// 读取到的u16是大端在前，然而stm32储存是小端在前，所以结构体中的u16数据都是反的
struct zigbee_conf_t
{
	char DevName[16];
	char DevPwd[16];
	uint8_t DevMode;
	uint8_t Chan;
	uint16_t PanID_oppo;
	uint16_t MyAddr_oppo;
	uint16_t DstAddr_oppo;
	uint8_t DataRate;
	uint8_t PowerLevel;
	uint8_t RetryNum;
	uint16_t RetryTimeout_oppo;
	uint8_t Serial_Rate;
	uint8_t Serial_DataB;
	uint8_t Serial_StopB;
	uint8_t Serial_ParityB;
	uint8_t Serial_Timeout;
	uint8_t Serial_Byteout;
	uint8_t SendMode;
}__attribute__((packed));
extern struct zigbee_conf_t zigbee_conf;

/* 通信操作 */
void zigbee_transmit(uint8_t* Tdata, uint8_t lenth, uint16_t target_addr);
void get_signal_strength(uint16_t target_addr);
// val: bit0->IO0  bit1->IO1  0=Input  1=Output
void set_IO_conf(uint16_t target_addr, uint8_t val);
// val: bit0->IO0  bit1->IO1
void set_IO_state(uint16_t target_addr, uint8_t val);
// 阻塞形式收发
// return: bit0->IO0  bit1->IO1
uint8_t get_IO_state(uint16_t target_addr);
void module_sleep(uint32_t sleep_ms);
uint8_t zigbee_decode(uint8_t* Rdata, uint8_t lenth);

/* 配置操作 */
void get_localInfo(void);
void get_Conf(void);
void write_Conf(void);
void write_MyAddr(uint16_t MyAddr);
void load_Conf(void);
// 读取并写入配置，写入后重启模块生效；并且只有在460800及以下波特率才能够写入成功
void get_write_Conf(void);
