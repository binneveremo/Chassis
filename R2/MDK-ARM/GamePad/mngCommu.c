#include "mngCommu.h"
#include <string.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "recDecode.h"
#include "crc8.h"
// other
#include "Television.h"
#include "CPU_Load.h"
#include "Location.h"
#include "Correct.h"
#include "Global.h"
#include "Flow.h"

#define RecPkg_state	stats_toReceiver.packets[stats_toReceiver.index]
#define RecPkg_indexpp 	stats_toReceiver.index = (stats_toReceiver.index+1) % MAX_PACKETS;			// 递增数据包编号

#ifdef USE_UART
myUartStruct commuUart;
zigbee_state_t zigbee_state;
zigbee_msg_t zigbee_msg;
struct zigbee_conf_t zigbee_conf;
#endif

#ifdef USE_NRF
Nrf_t nrf;
#endif

// 丢包统计结构体定义
PacketStats stats_toReceiver; // 接收机数据的丢包情况
uint32_t last_received_tick;
uint8_t GamepadLostConnection;

struct transmit_package_struct debugData_pkg;

uint16_t pkg_cnt_uart, pkg_cnt_nrf;

void Commu_init(void)
{
#ifdef USE_UART
	zigbee_state = ZIGBEE_STATE_INIT;
	Serial_init(&commuUart, &TX_Uart);
	get_write_Conf();
#endif
#ifdef USE_NRF
	nrf = NRF_INIT;
	_Nrf_CheckConnectivity(&nrf);
//	Nrf_Reset(&nrf);
	Nrf_Init(&nrf);
#endif
}

void Transmit_task(void)
{
	static uint8_t cnt;
#ifdef USE_NRF
	// 处理接收数据
	if(HAL_GPIO_ReadPin(nrf.irq.port, nrf.irq.pin) == GPIO_PIN_RESET)
	{
		Nrf_EXTI_Callback(&nrf, nrf.irq.pin);
	}
#endif

	// 计算包长
	debugData_pkg.lenth = sizeof(debugData_pkg) - 2;

	if(++cnt == 100)
	{cnt = 0;
#ifdef USE_UART
		debugData_pkg.rec_cnt = pkg_cnt_uart;
		debugData_pkg.crc = crc8_generate((const uint8_t*)&debugData_pkg+2, sizeof(debugData_pkg)-2);		// 生成crc校验码
//		HAL_UART_Transmit_DMA(commuUart.huart, (uint8_t *)&debugData_pkg, sizeof(debugData_pkg));
		zigbee_transmit((uint8_t *)&debugData_pkg, sizeof(debugData_pkg), peer_addr);
		// 清除已回报的包
		pkg_cnt_uart = 0;
#endif
#ifdef USE_NRF
		debugData_pkg.rec_cnt = pkg_cnt_nrf;
		debugData_pkg.crc = crc8_generate((const uint8_t*)&debugData_pkg+2, sizeof(debugData_pkg)-2);		// 生成crc校验码
		_Nrf_CheckConnectivity(&nrf);
		Nrf_Transmit(&nrf, (uint8_t *)&debugData_pkg, sizeof(debugData_pkg));
		// 清除已回报的包
		pkg_cnt_nrf = 0;
#endif
	}
	
	// 判断断连
	if(HAL_GetTick() - last_received_tick > 350)
	{
		GamepadLostConnection = 1;
#ifdef USE_NRF
		Nrf_Reset(&nrf);
#endif
	}
}

// 定时任务,freertos执行
void StartTransmit_task(void const * argument) {
	osDelay(200);
INFINITE_LOOP_START
	// 处理要发送的数据
	switch(GamePad_Data.Debug_Page){
		case 0:
			debugData_pkg.debug_data[0] = Char2float("ODOM");
			debugData_pkg.debug_data[2] = site.field.x_enc;
			debugData_pkg.debug_data[3] = site.field.y_enc;
			debugData_pkg.debug_data[4] = site.gyro.r;
			break;
		case 1:
			debugData_pkg.debug_data[0] = Char2float("LADA");
			debugData_pkg.debug_data[1] = vision.visual.ladar_visual.x;
			debugData_pkg.debug_data[2] = vision.visual.ladar_visual.y;
			debugData_pkg.debug_data[3] = vision.visual.ladar_visual.r;
			debugData_pkg.debug_data[4] = vision.field.height;
			break;
		case 2:
			debugData_pkg.debug_data[0] = Char2float("BASK");
			debugData_pkg.debug_data[1] = vision.basketlock.online_flag * 6.66;
			debugData_pkg.debug_data[2] = vision.visual.basket_visual.x;
			debugData_pkg.debug_data[3] = vision.visual.basket_visual.y;
			debugData_pkg.debug_data[4] = 0;
			break;
		case 3:
			debugData_pkg.debug_data[0] = Char2float("PROG");
			
			break;
		case 4:
			debugData_pkg.debug_data[0] = Char2float("SEND");
			debugData_pkg.debug_data[1] = send.R1_Exchange.net.x;
			debugData_pkg.debug_data[2] = send.R1_Exchange.net.y;
			debugData_pkg.debug_data[3] = vision.field.carcenter_field.r;
			debugData_pkg.debug_data[4] = 0;
			break;
		default:
			break;
	}
	memcpy(&debugData_pkg.Chassis_err,&Wrong_Code,sizeof(Wrong_Code));
	
	osDelay(10);
INFINITE_LOOP_END
}

void Mng_RxData(uint8_t *pdata, uint16_t data_length)
{
	if(data_length != get_lenth(pdata) + 2) return;	// 接收包位数检验
	if(!crc8_check(pdata+2, get_lenth(pdata), get_crc(pdata))) return;		// crc校验
	recDecodeData(pdata);

	// 判断断连
	last_received_tick = HAL_GetTick();
	GamepadLostConnection = 0;
	
//	// 发送循环
//	static uint8_t cnt;
//	if(++cnt == 20)
//	{
//		cnt = 0;
//		Transmit_task();
//	}
}


#ifdef USE_UART
// 接收任务,串口接收中断中调用
void ArrangeSerialList(void) {
#define commuRxbuff		commuUart.Uart_RxBuff[commuUart.Uart_RxBuffIndex]
#define commuRxCnt		commuUart.Uart_Rx_Cnt
//	if(commuUart.Uart_RxFlag == 1) {
//		Mng_RxData(commuRxbuff, commuRxCnt);
//		pkg_cnt_uart++;
//	}
	if(commuUart.Uart_RxFlag) {
		if(zigbee_decode(commuRxbuff, commuRxCnt) == MASTER_COMMU)
		{
			if(zigbee_state.source_addr == peer_addr)
			{
				Mng_RxData(zigbee_msg.RxData, zigbee_msg.datalen);
				pkg_cnt_uart++;
			}
			
			// 这里写与其它zigbee模块通信的接收数据处理
		}
	}
}
#endif

#ifdef USE_NRF
uint8_t rec_channel;
void NrfCommu_EXTI_Callback(uint8_t channel, uint8_t *data, uint8_t len)
{
	rec_channel = channel;
	Mng_RxData(data, len);
	pkg_cnt_nrf++;
}
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	Nrf_EXTI_Callback(&nrf, GPIO_Pin);  
//}
#endif


/* 丢包统计函数 */
void initStats(PacketStats *stats) {
	stats->index = 0;
	for(int j=0;j<MAX_PACKETS;j++) {
		stats->packets[j] = 0; // 初始化包状态
	}
}
void sendSuccess(PacketStats *stats, int n) {
	stats->packets[n] = 1; // 设置为发送成功标志
}
void sendFail(PacketStats *stats, int n) {
	stats->packets[n] = 0; // 设置为发送失败标志
}
uint8_t getLossRate(PacketStats *stats) {
	uint16_t count_s = 0;
	uint16_t count_f= 0;
	for (int i = 0; i < MAX_PACKETS; i++) {
			if (stats->packets[i] == 0) {
					count_f++;
			}
			if (stats->packets[i] == 1) {
					count_s++;
			}
	}
	return (double)count_f/(double)(count_s+count_f)*100.0f;
}
