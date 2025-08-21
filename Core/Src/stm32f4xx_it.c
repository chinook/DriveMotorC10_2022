/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "motor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

uint8_t timer_1ms_flag = 0;
uint8_t timer_50ms_flag = 0;
uint8_t timer_100ms_flag = 0;
uint8_t timer_250ms_flag = 0;
uint8_t timer_500ms_flag = 0;

uint8_t step_flag = 0;

uint8_t timer_50ms_counter = 0;
uint8_t timer_100ms_counter = 0;
uint8_t timer_250ms_counter = 0;
uint16_t timer_500ms_counter = 0;

uint8_t pb1_value = 0;
uint8_t pb2_value = 0;

volatile uint8_t sub_step = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
	while (1) {
	}
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 RX0 interrupt.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX1_IRQn 0 */

  /* USER CODE END CAN1_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX1_IRQn 1 */

  /* USER CODE END CAN1_RX1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(PB2_Pin);
  HAL_GPIO_EXTI_IRQHandler(PB1_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

	if (HAL_GPIO_ReadPin(PB1_GPIO_Port, PB1_Pin) == GPIO_PIN_SET) {
		pb1_value = 0; //bouton NO
	} else {
		pb1_value = 1; //bouton NO
	}
	if (HAL_GPIO_ReadPin(PB2_GPIO_Port, PB2_Pin) == GPIO_PIN_SET) {
		pb2_value = 0; //bouton NO
	} else {
		pb2_value = 1; //bouton NO
	}

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  if (sub_step) {
	  sub_step = 0;
	  if (motor_pitch_on) {
		  HAL_GPIO_WritePin(drive_ports[DRIVE1][DRIVE_STEP],
		  					drive_pins[DRIVE1][DRIVE_STEP], GPIO_PIN_SET);
		  //uint16_t reg_config = ReadRegConfig(DRIVE1, 0);
		  //reg_config = reg_config | 0x0004;

		  //WriteSPI(DRIVE1, DRV8711_CTRL_REG, reg_config);
	  }
  } else {
	  sub_step = 1;
	  HAL_GPIO_WritePin(drive_ports[DRIVE1][DRIVE_STEP],
	  				drive_pins[DRIVE1][DRIVE_STEP], GPIO_PIN_RESET);
  }


  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

	if (timer_50ms_counter > 50) {
		timer_50ms_counter = 0;
		timer_50ms_flag = 1;
	} else {
		timer_50ms_counter++;
	}

	if (timer_100ms_counter > 100) {
		timer_100ms_counter = 0;
		timer_100ms_flag = 1;
	} else {
		timer_100ms_counter++;
	}

	if (timer_250ms_counter > 250) {
		timer_250ms_counter = 0;
		timer_250ms_flag = 1;
	} else {
		timer_250ms_counter++;
	}

	if (timer_500ms_counter > 500) {
		timer_500ms_counter = 0;
		timer_500ms_flag = 1;
	} else {
		timer_500ms_counter++;
	}

  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/*
 // EXTI Line External Interrupt ISR Handler CallBack
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

 // bouton normalement ouvert GPIO relié au 3V3
 // quand le bouton est appuyé, le 3V3 devient GND

 if (GPIO_Pin == PB1_Pin) { // PushButton 1
 if (HAL_GPIO_ReadPin(PB1_GPIO_Port, PB1_Pin) == GPIO_PIN_SET) {
 pb1_value = 0; //bouton NO
 } else {
 pb1_value = 1; //bouton NO
 }
 } else if (GPIO_Pin == PB2_Pin) { // PushButton 2
 if (HAL_GPIO_ReadPin(PB2_GPIO_Port, PB2_Pin) == GPIO_PIN_SET) {
 pb2_value = 0; //bouton NO
 } else {
 pb2_value = 1; //bouton NO
 }
 }
 }
 */

/* USER CODE END 1 */
