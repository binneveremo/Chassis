/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId Motor_ControlHandle;
osThreadId CommunicationHandle;
osThreadId LocationHandle;
osThreadId myTransmit_taskHandle;
osThreadId Detect_WrongHandle;
osThreadId Defense1Handle;
osThreadId Defense2Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void motor_control(void const * argument);
void communication(void const * argument);
void location(void const * argument);
void StartTransmit_task(void const * argument);
void Detect(void const * argument);
void HTMotorControl(void const * argument);
void ParamsChange(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationTickHook(void);

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
__weak void vApplicationTickHook( void )
{
   /* This function will be called by each tick interrupt if
   configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
   added here, but the tick hook is called from an interrupt context, so
   code must not attempt to block, and only the interrupt safe FreeRTOS API
   functions can be used (those that end in FromISR()). */
}
/* USER CODE END 3 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Motor_Control */
  osThreadDef(Motor_Control, motor_control, osPriorityNormal, 0, 1024);
  Motor_ControlHandle = osThreadCreate(osThread(Motor_Control), NULL);

  /* definition and creation of Communication */
  osThreadDef(Communication, communication, osPriorityNormal, 0, 1024);
  CommunicationHandle = osThreadCreate(osThread(Communication), NULL);

  /* definition and creation of Location */
  osThreadDef(Location, location, osPriorityNormal, 0, 512);
  LocationHandle = osThreadCreate(osThread(Location), NULL);

  /* definition and creation of myTransmit_task */
  osThreadDef(myTransmit_task, StartTransmit_task, osPriorityRealtime, 0, 512);
  myTransmit_taskHandle = osThreadCreate(osThread(myTransmit_task), NULL);

  /* definition and creation of Detect_Wrong */
  osThreadDef(Detect_Wrong, Detect, osPriorityNormal, 0, 512);
  Detect_WrongHandle = osThreadCreate(osThread(Detect_Wrong), NULL);

  /* definition and creation of Defense1 */
  osThreadDef(Defense1, HTMotorControl, osPriorityNormal, 0, 512);
  Defense1Handle = osThreadCreate(osThread(Defense1), NULL);

  /* definition and creation of Defense2 */
  osThreadDef(Defense2, ParamsChange, osPriorityIdle, 0, 512);
  Defense2Handle = osThreadCreate(osThread(Defense2), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_motor_control */
/**
  * @brief  Function implementing the Motor_Control thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_motor_control */
__weak void motor_control(void const * argument)
{
  /* USER CODE BEGIN motor_control */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END motor_control */
}

/* USER CODE BEGIN Header_communication */
/**
* @brief Function implementing the Communication thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_communication */
__weak void communication(void const * argument)
{
  /* USER CODE BEGIN communication */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END communication */
}

/* USER CODE BEGIN Header_location */
/**
* @brief Function implementing the Location thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_location */
__weak void location(void const * argument)
{
  /* USER CODE BEGIN location */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END location */
}

/* USER CODE BEGIN Header_StartTransmit_task */
/**
* @brief Function implementing the myTransmit_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTransmit_task */
__weak void StartTransmit_task(void const * argument)
{
  /* USER CODE BEGIN StartTransmit_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTransmit_task */
}

/* USER CODE BEGIN Header_Detect */
/**
* @brief Function implementing the Detect_Wrong thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Detect */
__weak void Detect(void const * argument)
{
  /* USER CODE BEGIN Detect */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Detect */
}

/* USER CODE BEGIN Header_HTMotorControl */
/**
* @brief Function implementing the Defense1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HTMotorControl */
__weak void HTMotorControl(void const * argument)
{
  /* USER CODE BEGIN HTMotorControl */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END HTMotorControl */
}

/* USER CODE BEGIN Header_ParamsChange */
/**
* @brief Function implementing the Defense2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ParamsChange */
__weak void ParamsChange(void const * argument)
{
  /* USER CODE BEGIN ParamsChange */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ParamsChange */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
