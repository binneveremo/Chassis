/********************** NOTES **********************************************
1、STM32CubeMX软件中找到FREERTOS。选择Config parameters项,找到[USE_IDLE_HOOK][USE_TICK_HOOK]并使能。
                                  选择Include parameters项,找到[vTaskDelayUntil]并使能。
2、在FreeOS_Config.h中定义以下宏定义:  注意！！！是在用户代码区定义，否则会被清除。！！！
#define traceTASK_SWITCHED_IN() extern void StartIdleMonitor(void);StartIdleMonitor()
#define traceTASK_SWITCHED_OUT() extern void EndIdleMonitor(void);EndIdleMonitor()
************************************/
#include "CPU_Load.h"
long long CPU_Tick;
float CPU_USAGE_PERCENT;
xTaskHandle    xIdleHandle = NULL;


#define Record_Peiod 500
struct CPU_Usage{
	int idlebegintick;
	int idleendtick;
	int idletotaltick;
	
	/////////
	int lastrecordtick;
	int totaltick;
	/////////
	float usage;
};
struct CPU_Usage cu;
void vApplicationIdleHook(void)
{
	if(xIdleHandle == NULL)
			xIdleHandle = xTaskGetCurrentTaskHandle();
}
void vApplicationTickHook (void)
{
	if(CPU_Tick - cu.lastrecordtick > Record_Peiod){
		cu.usage = (float)((float)cu.idletotaltick / ((float)CPU_Tick - (float)cu.lastrecordtick));
		cu.idletotaltick = 0;
		cu.lastrecordtick = CPU_Tick;
		CPU_USAGE_PERCENT = 100*(1 - cu.usage);
	}
}
void StartIdleMonitor (void)
{
	if(xTaskGetCurrentTaskHandle() == xIdleHandle )
		cu.idlebegintick = CPU_Tick;
}
void EndIdleMonitor (void)
{
	if( xTaskGetCurrentTaskHandle() == xIdleHandle )
		cu.idletotaltick += (CPU_Tick - cu.idlebegintick);
    
}
void CPU_Usage_Test(void){
	float a = 1.2345;
	for(int i = 0; i < 100000; i++){
		a *= 28956.34;
		a /= 28956.34;
	}
}
/////////////////////////////////////////////////////////////CPU超频代码//////////////////////////////////////////
void SystemClock_SwitchToHSI(void)
{
    // 将系统时钟切换到 HSI
    __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_HSI);
 
    // 等待时钟切换完成
    while (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_HSI);
}
void SystemClock_SwitchToPLL(void)
{
    // 将系统时钟切换到 PLL
    __HAL_RCC_SYSCLK_CONFIG(RCC_SYSCLKSOURCE_PLLCLK);
 
    // 等待时钟切换完成
    while (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_PLLCLK);
}






















