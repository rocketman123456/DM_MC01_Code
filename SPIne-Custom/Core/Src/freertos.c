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
#define EVENT3 (0x01 << 3)
#define EVENT4 (0x01 << 4)
#define EVENT5 (0x01 << 5)
#define EVENT6 (0x01 << 6)
#define EVENT7 (0x01 << 7)
#define EVENT8 (0x01 << 8)

uint8_t spi_tx[140] = {0};
uint8_t spi_rx[140] = {0};

CAN_RxHeaderTypeDef rx_header1;
uint8_t can_rx_msg_1[8] = {0};

CAN_RxHeaderTypeDef rx_header2;
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
/* Definitions for CanRxTask */
osThreadId_t CanRxTaskHandle;
const osThreadAttr_t CanRxTask_attributes = {
  .name = "CanRxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CmdMutex */
osMutexId_t CmdMutexHandle;
const osMutexAttr_t CmdMutex_attributes = {
  .name = "CmdMutex"
};
/* Definitions for StateMutex */
osMutexId_t StateMutexHandle;
const osMutexAttr_t StateMutex_attributes = {
  .name = "StateMutex"
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
  if(HAL_CAN_IsTxMessagePending(hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2)) {
    HAL_CAN_AbortTxRequest(hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
  }

  uint8_t delay = 150;
  if(motor_enable[can_id] == 0 && g_cmd.leg[can_id].flag == 1)
  {
    motor_enable[can_id] = g_cmd.leg[can_id].flag;
    enter_motor_mode(can_tx_data);

    CANx_SendStdData(hcan, 0x01, can_tx_data, 8);
    delay_us(delay);

    CANx_SendStdData(hcan, 0x02, can_tx_data, 8);
    delay_us(delay);

    CANx_SendStdData(hcan, 0x03, can_tx_data, 8);
    delay_us(delay);
  }
  else if(motor_enable[can_id] == 1 && g_cmd.leg[can_id].flag == 0)
  {
    motor_enable[can_id] = g_cmd.leg[can_id].flag;
    exit_motor_mode(can_tx_data);

    CANx_SendStdData(hcan, 0x01, can_tx_data, 8);
    delay_us(delay);

    CANx_SendStdData(hcan, 0x02, can_tx_data, 8);
    delay_us(delay);

    CANx_SendStdData(hcan, 0x03, can_tx_data, 8);
    delay_us(delay);
  }

  pack_cmd(can_tx_data, &g_cmd.leg[can_id].motor[0]);
  CANx_SendStdData(hcan, 0x01, can_tx_data, 8);
  delay_us(delay);

  pack_cmd(can_tx_data, &g_cmd.leg[can_id].motor[1]);
  CANx_SendStdData(hcan, 0x02, can_tx_data, 8);
  delay_us(delay);

  pack_cmd(can_tx_data, &g_cmd.leg[can_id].motor[2]);
  CANx_SendStdData(hcan, 0x03, can_tx_data, 8);
  delay_us(delay);
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Can1TaskFunc(void *argument);
void Can2TaskFunc(void *argument);
void CanRxTaskFunc(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  g_state.crc = calculate((uint8_t*)&g_state, sizeof(spine_state_t) - 4);
  memcpy(spi_tx, &g_state, sizeof(spine_state_t));
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of CmdMutex */
  CmdMutexHandle = osMutexNew(&CmdMutex_attributes);

  /* creation of StateMutex */
  StateMutexHandle = osMutexNew(&StateMutex_attributes);

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

  /* creation of CanRxTask */
  CanRxTaskHandle = osThreadNew(CanRxTaskFunc, NULL, &CanRxTask_attributes);

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
    g_state.crc = calculate((uint8_t*)&g_state, sizeof(spine_state_t) - 4);
    memcpy(spi_tx, &g_state, sizeof(spine_state_t));
    //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    //delay_us(50);
    //HAL_UART_Transmit(&huart2, tx, sizeof(spine_state_t), 10000);
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

/* USER CODE BEGIN Header_CanRxTaskFunc */
/**
* @brief Function implementing the CanRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CanRxTaskFunc */
void CanRxTaskFunc(void *argument)
{
  /* USER CODE BEGIN CanRxTaskFunc */
  /* Infinite loop */
  for(;;)
  {
    osEventFlagsWait(CAN_EventHandle, EVENT3 | EVENT4 | EVENT5 | EVENT6 | EVENT7 | EVENT8, osFlagsWaitAll, portMAX_DELAY);
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    //HAL_CAN_GetRxMessage(&hcan1, CAN_FILTER_FIFO0, &rx_header1, can_rx_msg_1);
    //unpack_reply(can_rx_msg_1, &g_state.leg[0]);
    //HAL_CAN_GetRxMessage(&hcan2, CAN_FILTER_FIFO0, &rx_header2, can_rx_msg_2);
    //unpack_reply(can_rx_msg_2, &g_state.leg[1]);
    //delay_us(10);
    //osDelay(1000);
  }
  /* USER CODE END CanRxTaskFunc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(hcan->Instance==CAN1)
  {
    //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    // unpack data
    HAL_CAN_GetRxMessage(&hcan1, CAN_FILTER_FIFO0, &rx_header1, can_rx_msg_1);
    uint16_t id = unpack_reply(can_rx_msg_1, &g_state.leg[0]);
    osEventFlagsSet(CAN_EventHandle, 1 << (id + 0 + 2));
  }
  if(hcan->Instance==CAN2)
  {
    //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    // unpack data
    HAL_CAN_GetRxMessage(&hcan2, CAN_FILTER_FIFO0, &rx_header2, can_rx_msg_2);
    uint16_t id = unpack_reply(can_rx_msg_2, &g_state.leg[1]);
    osEventFlagsSet(CAN_EventHandle, 1 << (id + 3 + 2));
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
    osEventFlagsSet(SPI_EventHandle, EVENT1 | EVENT2);

    // copy data to tx
    g_state.crc = calculate((uint8_t*)&g_state, sizeof(spine_state_t) - 4);
    memcpy(spi_tx, &g_state, sizeof(spine_state_t));
  }
}
/* USER CODE END Application */

