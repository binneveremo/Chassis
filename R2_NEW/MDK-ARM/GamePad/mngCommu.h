#pragma once
#include "main.h"
#include <stdio.h>
#include "recDecode.h"
#include "Global.h"
//#include "Config.h"
#define INFINITE_LOOP_START for(;;){osDelay(1);
#define INFINITE_LOOP_END }

//#define USE_UART
#define USE_NRF


#include "usart.h"
#include "mySerial.h"
#define TX_Uart huart4
extern myUartStruct commuUart;
#include "zigbee.h"
#define peer_addr 0x8003
extern zigbee_state_t zigbee_state;
extern zigbee_msg_t zigbee_msg;
extern struct zigbee_conf_t zigbee_conf;
/*
DevMode 0：主机模式 1：透传模式
Serial_Rate 波特率 0x0B： 1.5M 
SetMyAddr 主机地址
DstAddr 透传模式目标地址
Chan 通道
PowerLevel 功率
DataRate 空中速率
RetryNum 自动重发次数
RetryTimeout 自动重发等待时间
*/
#define ZIGBEE_STATE_INIT (zigbee_state_t) {							\
	.huart = &TX_Uart,																			\
	.DevMode = 0,																						\
	.Serial_Rate = 0x0B,																		\
	.PanID = 0x0081,																				\
	.SetMyAddr = 0x8004,																		\
	.DstAddr = 0x8003,																			\
	.Chan = 0x23,																						\
	.PowerLevel = 0x09,																			\
	.DataRate = 0x04,																				\
	.RetryNum = 0x01,																				\
	.RetryTimeout = 0x0002,																	\
}


#ifdef USE_NRF
#include "Nrf.h"
#ifdef Carbon_Car
	#define NRF_INIT (Nrf_t) {                                                \
			.hspi = &hspi4,                                                     \
			.ce  = {.port = GPIOE, .pin = GPIO_PIN_15},                         \
			.nss = {.port = GPIOE, .pin = GPIO_PIN_11},                         \
			.irq = {.port = GPIOB, .pin = GPIO_PIN_10},                         \
			.rst = {.port = GPIOB, .pin = GPIO_PIN_11},                         \
			.address_transmit = {0x0, 0x66, 0x77, 0x88, 0x99},                  \
			.address_receive =  {                                               \
					.high_addr = {0x6, 0x7, 0x8, 0x9},                              \
					.p1 = 0x0, .p2 = 0x1, .p3 = 0x2, .p4 = 0x3, .p5 = 0x4           \
			},                                                                  \
			.rf_channel = 3,                                                    \
			.nrf_rx_callback = NrfCommu_EXTI_Callback,                          \
		}
#else 
	#define NRF_INIT (Nrf_t) {                                              \
			.hspi = &hspi4,                                                     \
			.ce  = {.port = GPIOE, .pin = GPIO_PIN_15},                         \
			.nss = {.port = GPIOE, .pin = GPIO_PIN_11},                         \
			.irq = {.port = GPIOB, .pin = GPIO_PIN_10},                         \
			.rst = {.port = GPIOB, .pin = GPIO_PIN_11},                         \
			.address_transmit = {0x03, 0xAA, 0xBB, 0xCC, 0xDD},                 \
			.address_receive =  {                                               \
					.high_addr = {0xA, 0xB, 0xC, 0xD},                              \
					.p1 = 0x0, .p2 = 0x1, .p3 = 0x2, .p4 = 0x3, .p5 = 0x4           \
			},                                                                  \
			.rf_channel = 5,                                                    \
			.nrf_rx_callback = NrfCommu_EXTI_Callback,                          \
	}
#endif
#endif

struct transmit_package_struct
{
	uint8_t crc;						//0 crc校验码
	uint8_t lenth;						//1 总长度-2
	//以下部分使用crc校验
	uint8_t rec_cnt;				//2 接收包的编号
										//3+
	float debug_data[5];				// 任意回传数据
	struct{
		unsigned char data[2];
	}Chassis_err;
} __attribute__((packed));	// 对齐设置,强制不进行补位操作
extern struct transmit_package_struct debugData_pkg;

/* 统计丢包情况 */
#define MAX_PACKETS 256			// 统计丢包率的总数

typedef struct {
	int packets[MAX_PACKETS]; // 包的接收状态
	int index;                // 当前索引
} PacketStats;

extern PacketStats stats_toReceiver; // 数据的丢包情况结构体
extern uint8_t GamepadLostConnection;

void NrfCommu_EXTI_Callback(uint8_t channel, uint8_t *data, uint8_t len);
void Commu_init(void);
void Transmit_task(void);

void initStats(PacketStats *stats);
void ArrangeSerialList(void);
void sendSuccess(PacketStats *stats, int n);
void sendFail(PacketStats *stats, int n);
uint8_t getLossRate(PacketStats *stats);
