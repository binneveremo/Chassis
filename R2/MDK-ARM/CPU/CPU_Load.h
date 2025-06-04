#ifndef _CPU_UTILS_H__
#define _CPU_UTILS_H__

extern long long CPU_Tick;
#include "main.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "tim.h"
#define CPU_Usage_Init() {HAL_TIM_Base_Start_IT(&htim3);}

extern float CPU_USAGE_PERCENT;
void CPU_Usage_Test(void);

#endif 

