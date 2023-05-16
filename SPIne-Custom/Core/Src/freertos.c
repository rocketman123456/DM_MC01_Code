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
#include "crc.h"

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

int state = 1;
uint8_t tx[140] = {0};
uint8_t rx[140] = {0};
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
uint8_t tx_data[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};

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
  // uint32_t temp;                  
  // SysTick->LOAD = nus * fac_us;
  // SysTick->VAL = 0x00;
  // SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;    
  // do
  // {
  //   temp = SysTick->CTRL;
  // } while ((temp & 0x01) && !(temp & (1<<16)));
  // SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
  // SysTick->VAL =0X00;
}

uint8_t CANx_SendStdData(CAN_HandleTypeDef* hcan, uint16_t ID, uint8_t *pData, uint16_t Len)
{
  static CAN_TxHeaderTypeDef Tx_Header;

  Tx_Header.StdId=ID;
  Tx_Header.ExtId=0;
  Tx_Header.IDE=0;
  Tx_Header.RTR=0;
  Tx_Header.DLC=Len;

  HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX0);
  //if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
  //{
  //  if(HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
  //  {
  //    HAL_CAN_AddTxMessage(hcan, &Tx_Header, pData, (uint32_t*)CAN_TX_MAILBOX2);
  //  }
  //}
  return 0;
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

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
  /* creation of mainTask */
  mainTaskHandle = osThreadNew(StartDefaultTask, NULL, &mainTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  /* Infinite loop */
  for(;;)
  {
    //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    CANx_SendStdData(&hcan1, 0x01, tx_data, 8);
    delay_us(1000);
    //osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
CAN_RxHeaderTypeDef rx_header1;
CAN_RxHeaderTypeDef rx_header2;
uint8_t rx_msg_1[8] = {0};
uint8_t rx_msg_2[8] = {0};

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(hcan->Instance==CAN1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_CAN_GetRxMessage(&hcan1, CAN_FILTER_FIFO0, &rx_header1, rx_msg_1);
    //HAL_UART_Transmit(&huart2, rx_msg_1, 8, 10000);
    // delay 20 us
    //delay_us(20);
    //CANx_SendStdData(&hcan1, 0x01, tx_data, 8);
    // unpack data
  }
  if(hcan->Instance==CAN2)
  {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_CAN_GetRxMessage(&hcan2, CAN_FILTER_FIFO0, &rx_header2, rx_msg_2);
    //HAL_UART_Transmit(&huart2, rx_msg_2, 8, 10000);
    // delay 20 us
    //delay_us(20);
    //CANx_SendStdData(&hcan2, 0x01, tx_data, 8);
    // unpack data
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == SPI_CS_Pin)
  {
    //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 132, 100);

    hspi1.Instance->DR = 0x00;

    // decode rx
    memcpy(&g_cmd, rx, sizeof(spine_cmd_t));
    uint32_t crc = calculate((uint8_t*)&g_cmd, sizeof(spine_cmd_t) - 4);
    if(crc != g_cmd.crc)
    {
      //HAL_UART_Transmit(&huart2, "crc error", 10, 10000);
      HAL_UART_Transmit(&huart2, (uint8_t*) &g_cmd, sizeof(spine_cmd_t), 10000);
    }
    // copy data to tx
    g_state.state[0].state[0].p = 1;
    g_state.state[0].state[0].v = 2;
    g_state.state[0].state[0].t = 3;
    g_state.crc = calculate((uint8_t*)&g_state, sizeof(spine_state_t) - 4);
    memcpy(tx, &g_state, sizeof(spine_state_t));

    HAL_UART_Transmit(&huart2, tx, sizeof(spine_state_t), 10000);
    // trigger event to main task
  }
}
/* USER CODE END Application */

