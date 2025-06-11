#pragma once
#include "main.h"
#include <stdio.h>
#include "recDecode.h"
#define INFINITE_LOOP_START for(;;){osDelay(1);
#define INFINITE_LOOP_END }

// #define USE_UART
// // #define USE_UDP
#define USE_NRF


#ifdef USE_UART
#include "usart.h"
#include "mySerial.h"
#define TX_UART huart1
extern myUartStruct commuUart;
#endif

#ifdef USE_UDP
#include "myUdp.h"
#define DES_IP_DEF 	{192,168,1,11}
#define DES_PORT	11
#define MY_PORT		12
#endif

#ifdef USE_NRF
#include "Global.h"
#include "Nrf.h"
extern Nrf_t nrf;

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

extern uint8_t GamepadLostConnection;

void NrfCommu_EXTI_Callback(uint8_t channel, uint8_t *data, uint8_t len);
void Commu_init(void);

