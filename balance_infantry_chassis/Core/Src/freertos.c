/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
typedef StaticTask_t osStaticThreadDef_t;
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ImuTask */
osThreadId_t ImuTaskHandle;
uint32_t ImuTaskBuffer[ 1024 ];
osStaticThreadDef_t ImuTaskControlBlock;
const osThreadAttr_t ImuTask_attributes = {
  .name = "ImuTask",
  .cb_mem = &ImuTaskControlBlock,
  .cb_size = sizeof(ImuTaskControlBlock),
  .stack_mem = &ImuTaskBuffer[0],
  .stack_size = sizeof(ImuTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for EstimatorTask */
osThreadId_t EstimatorTaskHandle;
uint32_t EstimatorTaskBuffer[ 1024 ];
osStaticThreadDef_t EstimatorTaskControlBlock;
const osThreadAttr_t EstimatorTask_attributes = {
  .name = "EstimatorTask",
  .cb_mem = &EstimatorTaskControlBlock,
  .cb_size = sizeof(EstimatorTaskControlBlock),
  .stack_mem = &EstimatorTaskBuffer[0],
  .stack_size = sizeof(EstimatorTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ChassisTask */
osThreadId_t ChassisTaskHandle;
uint32_t ChassisTaskBuffer[ 2048 ];
osStaticThreadDef_t ChassisTaskControlBlock;
const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .cb_mem = &ChassisTaskControlBlock,
  .cb_size = sizeof(ChassisTaskControlBlock),
  .stack_mem = &ChassisTaskBuffer[0],
  .stack_size = sizeof(ChassisTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for RefereeTask */
osThreadId_t RefereeTaskHandle;
uint32_t RefereeTaskBuffer[ 1024 ];
osStaticThreadDef_t RefereeTaskControlBlock;
const osThreadAttr_t RefereeTask_attributes = {
  .name = "RefereeTask",
  .cb_mem = &RefereeTaskControlBlock,
  .cb_size = sizeof(RefereeTaskControlBlock),
  .stack_mem = &RefereeTaskBuffer[0],
  .stack_size = sizeof(RefereeTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CansendTask */
osThreadId_t CansendTaskHandle;
uint32_t CansendTaskBuffer[ 1024 ];
osStaticThreadDef_t CansendTaskControlBlock;
const osThreadAttr_t CansendTask_attributes = {
  .name = "CansendTask",
  .cb_mem = &CansendTaskControlBlock,
  .cb_size = sizeof(CansendTaskControlBlock),
  .stack_mem = &CansendTaskBuffer[0],
  .stack_size = sizeof(CansendTaskBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void ImuTask_Entry(void *argument);
void EstimatorTask_Entry(void *argument);
void ChassisTask_Entry(void *argument);
void RefereeTask_Entry(void *argument);
void CansendTask_Entry(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ImuTask */
  ImuTaskHandle = osThreadNew(ImuTask_Entry, NULL, &ImuTask_attributes);

  /* creation of EstimatorTask */
  EstimatorTaskHandle = osThreadNew(EstimatorTask_Entry, NULL, &EstimatorTask_attributes);

  /* creation of ChassisTask */
  ChassisTaskHandle = osThreadNew(ChassisTask_Entry, NULL, &ChassisTask_attributes);

  /* creation of RefereeTask */
  RefereeTaskHandle = osThreadNew(RefereeTask_Entry, NULL, &RefereeTask_attributes);

  /* creation of CansendTask */
  CansendTaskHandle = osThreadNew(CansendTask_Entry, NULL, &CansendTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ImuTask_Entry */
/**
* @brief Function implementing the ImuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ImuTask_Entry */
__weak void ImuTask_Entry(void *argument)
{
  /* USER CODE BEGIN ImuTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ImuTask_Entry */
}

/* USER CODE BEGIN Header_EstimatorTask_Entry */
/**
* @brief Function implementing the EstimatorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EstimatorTask_Entry */
__weak void EstimatorTask_Entry(void *argument)
{
  /* USER CODE BEGIN EstimatorTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END EstimatorTask_Entry */
}

/* USER CODE BEGIN Header_ChassisTask_Entry */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisTask_Entry */
__weak void ChassisTask_Entry(void *argument)
{
  /* USER CODE BEGIN ChassisTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ChassisTask_Entry */
}

/* USER CODE BEGIN Header_RefereeTask_Entry */
/**
* @brief Function implementing the RefereeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RefereeTask_Entry */
__weak void RefereeTask_Entry(void *argument)
{
  /* USER CODE BEGIN RefereeTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RefereeTask_Entry */
}

/* USER CODE BEGIN Header_CansendTask_Entry */
/**
* @brief Function implementing the CansendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CansendTask_Entry */
__weak void CansendTask_Entry(void *argument)
{
  /* USER CODE BEGIN CansendTask_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CansendTask_Entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

