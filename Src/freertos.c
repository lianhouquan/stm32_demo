/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "can.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int16_t set_spd[4];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */



/* USER CODE END Variables */
osThreadId StartTaskHandle;
osThreadId TaskMoveHandle;
osThreadId TaskHolderHandle;
osThreadId TaskShotHandle;
osThreadId TaskGetInfoHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void TaskMoveFun(void const * argument);
void TaskHolderFun(void const * argument);
void TaskShotFun(void const * argument);
void TaskGetInfoFun(void const * argument);

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

  /* Create the thread(s) */
  /* definition and creation of StartTask */
  osThreadDef(StartTask, StartDefaultTask, osPriorityHigh, 0, 128);
  StartTaskHandle = osThreadCreate(osThread(StartTask), NULL);

  /* definition and creation of TaskMove */
  osThreadDef(TaskMove, TaskMoveFun, osPriorityLow, 0, 256);
  TaskMoveHandle = osThreadCreate(osThread(TaskMove), NULL);

  /* definition and creation of TaskHolder */
  osThreadDef(TaskHolder, TaskHolderFun, osPriorityLow, 0, 256);
  TaskHolderHandle = osThreadCreate(osThread(TaskHolder), NULL);

  /* definition and creation of TaskShot */
  osThreadDef(TaskShot, TaskShotFun, osPriorityLow, 0, 256);
  TaskShotHandle = osThreadCreate(osThread(TaskShot), NULL);

  /* definition and creation of TaskGetInfo */
  osThreadDef(TaskGetInfo, TaskGetInfoFun, osPriorityLow, 0, 256);
  TaskGetInfoHandle = osThreadCreate(osThread(TaskGetInfo), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the StartTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_1);
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_TaskMoveFun */
/**
* @brief Function implementing the TaskMove thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskMoveFun */
void TaskMoveFun(void const * argument)
{
  /* USER CODE BEGIN TaskMoveFun */
	//初始化PID
	for(uint8_t i=0; i<4; i++)
	{
		//设定PID的最大输出在16384
		PID_struct_init(&pid_spd[i], POSITION_PID, 16384, 16384,1.5f,	0.1f,	0.0f	);  //4 motos angular rate closeloop.
	}
  /* Infinite loop */
  for(;;)
  {
		
		cal_speed( set_spd);
		//计算PID
		for(int i=0; i<4; i++)
		{
			pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, set_spd[i]);
		}
		//输出
		set_moto_current(&hcan1, pid_spd[0].pos_out,pid_spd[1].pos_out,	pid_spd[2].pos_out,pid_spd[3].pos_out);
		osDelay(50);
    
  }

  /* USER CODE END TaskMoveFun */
}

/* USER CODE BEGIN Header_TaskHolderFun */
/**
* @brief Function implementing the TaskHolder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskHolderFun */
void TaskHolderFun(void const * argument)
{
  /* USER CODE BEGIN TaskHolderFun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TaskHolderFun */
}

/* USER CODE BEGIN Header_TaskShotFun */
/**
* @brief Function implementing the TaskShot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskShotFun */
void TaskShotFun(void const * argument)
{
  /* USER CODE BEGIN TaskShotFun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TaskShotFun */
}

/* USER CODE BEGIN Header_TaskGetInfoFun */
/**
* @brief Function implementing the TaskGetInfo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskGetInfoFun */
void TaskGetInfoFun(void const * argument)
{
  /* USER CODE BEGIN TaskGetInfoFun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TaskGetInfoFun */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
