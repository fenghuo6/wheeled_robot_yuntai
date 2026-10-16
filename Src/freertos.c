/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "laser_task.hpp"

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
osThreadId defaultTaskHandle;
osThreadId imuTaskHandle;
osThreadId uartTaskHandle;
osThreadId canTaskHandle;
osThreadId ledTaskHandle;
osThreadId buzzerTaskHandle;
osThreadId plotterTaskHandle;
osThreadId laserTaskHandle;
osThreadId trajectoryTaskHandle;
osThreadId motorTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
extern void imu_task(void const * argument);
extern void uart_task(void const * argument);
extern void can_task(void const * argument);
extern void led_task(void const * argument);
extern void buzzer_task(void const * argument);
extern void plotter_task(void const * argument);
void laser_task(void const * argument);
extern void trajectory_task(void const * argument);
extern void motor_task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of imuTask */
  osThreadDef(imuTask, imu_task, osPriorityRealtime, 0, 256);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

  /* definition and creation of uartTask */
  osThreadDef(uartTask, uart_task, osPriorityHigh, 0, 128);
  uartTaskHandle = osThreadCreate(osThread(uartTask), NULL);

  /* definition and creation of canTask */
  osThreadDef(canTask, can_task, osPriorityHigh, 0, 128);
  canTaskHandle = osThreadCreate(osThread(canTask), NULL);

  /* definition and creation of ledTask */
  osThreadDef(ledTask, led_task, osPriorityIdle, 0, 128);
  ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);

  /* definition and creation of buzzerTask */
  osThreadDef(buzzerTask, buzzer_task, osPriorityIdle, 0, 128);
  buzzerTaskHandle = osThreadCreate(osThread(buzzerTask), NULL);

  /* definition and creation of plotterTask */
  osThreadDef(plotterTask, plotter_task, osPriorityLow, 0, 256);
  plotterTaskHandle = osThreadCreate(osThread(plotterTask), NULL);

  /* definition and creation of laserTask */
  osThreadDef(laserTask, laser_task, osPriorityNormal, 0, 256);
  laserTaskHandle = osThreadCreate(osThread(laserTask), NULL);

  /* definition and creation of trajectoryTask */
  osThreadDef(trajectoryTask, trajectory_task, osPriorityHigh, 0, 256);
  trajectoryTaskHandle = osThreadCreate(osThread(trajectoryTask), NULL);

  /* definition and creation of motorTask */
  osThreadDef(motorTask, motor_task, osPriorityHigh, 0, 128);
  motorTaskHandle = osThreadCreate(osThread(motorTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_laser_task */
/**
* @brief Function implementing the laserTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_laser_task */
void laser_task(void const * argument)
{
  /* USER CODE BEGIN laser_task */
  gy56_laser_task(argument);
  /* USER CODE END laser_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
