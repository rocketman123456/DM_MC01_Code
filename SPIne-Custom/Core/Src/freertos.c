/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "motor_control.h"
#include "message_ops.h"
#include "crc.h"

#include "event_groups.h"
#include "semphr.h"

#include "can.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_gpio.h"

#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
spine_cmd_t   g_cmd;
spine_state_t g_state;

#define EVENT1 (0x01 << 1)
#define EVENT2 (0x01 << 2)

uint8_t spi_tx[140] = {0};
uint8_t spi_rx[140] = {0};

CAN_RxHeaderTypeDef rx_header1;
CAN_RxHeaderTypeDef rx_header2;
uint8_t can_rx_msg_1[8] = {0};
uint8_t can_rx_msg_2[8] = {0};

uint8_t can_tx_data[8] = {1, 2, 3, 4, 5, 6, 7, 8};

uint8_t motor_enable[2] = {0};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for mainTask */
osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
  .name = "mainTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Can1Task */
osThreadId_t Can1TaskHandle;
const osThreadAttr_t Can1Task_attributes = {
  .name = "Can1Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Can2Task */
osThreadId_t Can2TaskHandle;
const osThreadAttr_t Can2Task_attributes = {
  .name = "Can2Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DataMutex */
osMutexId_t DataMutexHandle;
const osMutexAttr_t DataMutex_attributes = {
  .name = "DataMutex"
};
/* Definitions for DataReMutex */
osMutexId_t DataReMutexHandle;
const osMutexAttr_t DataReMutex_attributes = {
  .name = "DataReMutex",
  .attr_bits = osMutexRecursive,
};
/* Definitions for SPI_Event */
osEventFlagsId_t SPI_EventHandle;
const osEventFlagsAttr_t SPI_Event_attributes = {
  .name = "SPI_Event"
};
/* Definitions for CAN_Event */
osEventFlagsId_t CAN_EventHandle;
const osEventFlagsAttr_t CAN_Event_attributes = {
  .name = "CAN_Event"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void delay_us(uint32_t nus)
{
  static uint32_t fac_us = 168;
  uint32_t told, tnow;
  uint32_t tcnt = 0;
  uint32_t reload = SysTick->LOAD;
  uint32_t ticks = nus * fac_us;
  told = SysTick->VAL;
  while(1)
  {
    tnow = SysTick->VAL;
    if(tnow != told)
    {
      if(tnow < told)
        tcnt += told - tnow;
      else
        tcnt += reload - tnow + told;
      told = tnow;
      if(tcnt >= ticks)
        break;
    }
  }
}

void motor_control(CAN_HandleTypeDef* hcan, uint8_t can_id)
{
  if(motor_enable[can_id] == 0 && g_cmd.cmd[can_id].flag == 1)
  {
    motor_enable[can_id] = g_cmd.cmd[can_id].flag;
    enter_motor_mode(can_tx_data);
    CANx_SendStdData(hcan, 0x01, can_tx_data, 8);
    delay_us(20);
    CANx_SendStdData(hcan, 0x02, can_tx_data, 8);
    delay_us(20);
    CANx_SendStdData(hcan, 0x03, can_tx_data, 8);
    delay_us(20);
  }
  else if(motor_enable[can_id] == 1 && g_cmd.cmd[can_id].flag == 0)
  {
    motor_enable[can_id] = g_cmd.cmd[can_id].flag;
    exit_motor_mode(can_tx_data);
    CANx_SendStdData(hcan, 0x01, can_tx_data, 8);
    delay_us(20);
    CANx_SendStdData(hcan, 0x02, can_tx_data, 8);
    delay_us(20);
    CANx_SendStdData(hcan, 0x03, can_tx_data, 8);
    delay_us(20);
  }

  pack_cmd(can_tx_data, &g_cmd.cmd[can_id].cmd[0]);
  CANx_SendStdData(hcan, 0x01, can_tx_data, 8);
  delay_us(20);

  pack_cmd(can_tx_data, &g_cmd.cmd[can_id].cmd[1]);
  CANx_SendStdData(hcan, 0x02, can_tx_data, 8);
  delay_us(20);

  pack_cmd(can_tx_data, &g_cmd.cmd[can_id].cmd[2]);
  CANx_SendStdData(hcan, 0x03, can_tx_data, 8);
  delay_us(20);
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Can1TaskFunc(void *argument);
void Can2TaskFunc(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of DataMutex */
  DataMutexHandle = osMutexNew(&DataMutex_attributes);

  /* Create the recursive mutex(es) */
  /* creation of DataReMutex */
  DataReMutexHandle = osMutexNew(&DataReMutex_attributes);

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
  /* creation of mainTask */
  mainTaskHandle = osThreadNew(StartDefaultTask, NULL, &mainTask_attributes);

  /* creation of Can1Task */
  Can1TaskHandle = osThreadNew(Can1TaskFunc, NULL, &Can1Task_attributes);

  /* creation of Can2Task */
  Can2TaskHandle = osThreadNew(Can2TaskFunc, NULL, &Can2Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of SPI_Event */
  SPI_EventHandle = osEventFlagsNew(&SPI_Event_attributes);

  /* creation of CAN_Event */
  CAN_EventHandle = osEventFlagsNew(&CAN_Event_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the mainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  EventBits_t r_event = pdPASS;
  /* Infinite loop */
  for(;;)
  {
    r_event = osEventFlagsWait(CAN_EventHandle, EVENT1 | EVENT2, osFlagsWaitAll, portMAX_DELAY);
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    g_state.crc = calculate((uint8_t*)&g_state, sizeof(spine_state_t) - 4);
    memcpy(spi_tx, &g_state, sizeof(spine_state_t));
    //HAL_UART_Transmit(&huart2, tx, sizeof(spine_state_t), 10000);

    //osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Can1TaskFunc */
/**
* @brief Function implementing the Can1Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Can1TaskFunc */
void Can1TaskFunc(void *argument)
{
  /* USER CODE BEGIN Can1TaskFunc */
  EventBits_t r_event = pdPASS;
  /* Infinite loop */
  for(;;)
  {
    r_event = osEventFlagsWait(SPI_EventHandle, EVENT1, osFlagsWaitAny, portMAX_DELAY);

    motor_control(&hcan1, 0);

    osEventFlagsSet(CAN_EventHandle, EVENT1);
  }
  /* USER CODE END Can1TaskFunc */
}

/* USER CODE BEGIN Header_Can2TaskFunc */
/**
* @brief Function implementing the Can2Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Can2TaskFunc */
void Can2TaskFunc(void *argument)
{
  /* USER CODE BEGIN Can2TaskFunc */
  EventBits_t r_event = pdPASS;
  /* Infinite loop */
  for(;;)
  {
    r_event = osEventFlagsWait(SPI_EventHandle, EVENT2, osFlagsWaitAny, portMAX_DELAY);

    motor_control(&hcan2, 1);

    osEventFlagsSet(CAN_EventHandle, EVENT2);
  }
  /* USER CODE END Can2TaskFunc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(hcan->Instance==CAN1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_CAN_GetRxMessage(&hcan1, CAN_FILTER_FIFO0, &rx_header1, can_rx_msg_1);
    // unpack data
    unpack_reply(can_rx_msg_2, &g_state.state[0]);
  }
  if(hcan->Instance==CAN2)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_CAN_GetRxMessage(&hcan2, CAN_FILTER_FIFO0, &rx_header2, can_rx_msg_2);
    // unpack data
    unpack_reply(can_rx_msg_2, &g_state.state[1]);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == SPI_CS_Pin)
  {
    //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_SPI_TransmitReceive(&hspi1, spi_tx, spi_rx, 132, 100);

    hspi1.Instance->DR = 0x00;

    // decode rx
    memcpy(&g_cmd, spi_rx, sizeof(spine_cmd_t));
    uint32_t crc = calculate((uint8_t*)&g_cmd, sizeof(spine_cmd_t) - 4);

    if(crc != g_cmd.crc)
    {
      HAL_UART_Transmit(&huart2, "crc error", 10, 10000);
      HAL_UART_Transmit(&huart2, (uint8_t*) &g_cmd, sizeof(spine_cmd_t), 10000);
      // skip error command
      return;
    }

    // trigger event to main task
    osEventFlagsSet(SPI_EventHandle, EVENT1);
    osEventFlagsSet(SPI_EventHandle, EVENT2);

    // copy data to tx
    //g_state.state[0].state[0].p = 1;
    //g_state.state[0].state[0].v = 2;
    //g_state.state[0].state[0].t = 3;
    g_state.crc = calculate((uint8_t*)&g_state, sizeof(spine_state_t) - 4);
    memcpy(spi_tx, &g_state, sizeof(spine_state_t));
  }
}
/* USER CODE END Application */

