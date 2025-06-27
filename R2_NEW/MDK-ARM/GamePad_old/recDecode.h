#pragma once
#include "main.h"

struct receivedPackage
{
	uint8_t crc;						//0 crc校验码
	uint8_t lenth;						//1 总长度-2
	//以下部分使用crc校验
	uint8_t send_index;					//2 发送包的编号
										//3+
	int8_t rockers[4];
	//rockers[0]:LX rockers[1]:lY rockers[2]:RX rockers[3]:RY
	uint32_t FunctionKeys;
	uint32_t switchs;
} __attribute__((packed));  // 对齐设置,强制不进行补位操作
extern struct receivedPackage recPkg;

uint8_t recDecodeData(uint8_t *pdata);  // 返回pdata的第0位数据(即数据包类型标识符)

int8_t get_rocker(uint8_t n);
uint8_t get_fk_state(uint8_t n);
uint8_t get_sw_state(uint8_t n);

/* 从数据包获取参数的函数 */
uint8_t get_crc(uint8_t *data);
uint8_t get_lenth(uint8_t *data);
//uint8_t get_recIndex(uint8_t *data);
//uint8_t get_tytle(uint8_t *data);
//uint8_t get_tsmIndex(uint8_t *data);
uint8_t get_pkgIndex();
