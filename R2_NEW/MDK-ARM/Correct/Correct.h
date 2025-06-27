#ifndef __CORRECT_H
#define __CORRECT_H

#include "stm32h723xx.h"
#include "Television.h"
#include "Chassis.h"
#include "Global.h"
#include "string.h"
#include "Basket.h"
#include "send.h"


void Can_Detect(void);
struct __attribute__((packed)) Wrong_Code_t {
	char ho7213:4;
	char vesc:4;
	char R1_loss:1;
	char odom:2;
	char gyro:1;
	char ladar:1;
	char HT:2;
	char basket_near:1;
};
extern struct Wrong_Code_t Wrong_Code;
void LossConnect_Check(void);
#define Wrong_Code_Clear() memset(&Wrong_Code,NONE,sizeof(Wrong_Code))
	

#endif
