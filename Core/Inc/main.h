/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

enum MOTOR_MODES
{
	MODE_MANUAL = 0,
	MODE_AUTOMATIC
};

extern uint8_t b_timer500ms_flag;
extern uint8_t b_timer50ms_flag;

extern uint8_t pitch_mode;
extern uint8_t mast_mode;

extern int32_t pitch_cmd_nbr_steps;
extern int32_t mast_cmd_dir;


extern uint8_t can1_recv_flag;

extern CAN_TxHeaderTypeDef pTxHeader;
extern CAN_RxHeaderTypeDef pRxHeader;

extern uint32_t txMailbox;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define nSTALL2_Pin GPIO_PIN_3
#define nSTALL2_GPIO_Port GPIOE
#define nFAULT2_Pin GPIO_PIN_4
#define nFAULT2_GPIO_Port GPIOE
#define FT_RESET_Pin GPIO_PIN_5
#define FT_RESET_GPIO_Port GPIOE
#define SPI_CS2_Pin GPIO_PIN_6
#define SPI_CS2_GPIO_Port GPIOE
#define BIN2_2_Pin GPIO_PIN_0
#define BIN2_2_GPIO_Port GPIOC
#define BIN1_2_Pin GPIO_PIN_1
#define BIN1_2_GPIO_Port GPIOC
#define DIR2_Pin GPIO_PIN_2
#define DIR2_GPIO_Port GPIOC
#define nSLEEP2_Pin GPIO_PIN_4
#define nSLEEP2_GPIO_Port GPIOC
#define RESET2_Pin GPIO_PIN_5
#define RESET2_GPIO_Port GPIOC
#define nSTALL1_Pin GPIO_PIN_1
#define nSTALL1_GPIO_Port GPIOB
#define nFAULT1_Pin GPIO_PIN_2
#define nFAULT1_GPIO_Port GPIOB
#define SPI_CS1_Pin GPIO_PIN_7
#define SPI_CS1_GPIO_Port GPIOE
#define PB2_Pin GPIO_PIN_8
#define PB2_GPIO_Port GPIOE
#define PB2_EXTI_IRQn EXTI9_5_IRQn
#define PB1_Pin GPIO_PIN_9
#define PB1_GPIO_Port GPIOE
#define PB1_EXTI_IRQn EXTI9_5_IRQn
#define DIR1_Pin GPIO_PIN_12
#define DIR1_GPIO_Port GPIOE
#define STEP1_Pin GPIO_PIN_13
#define STEP1_GPIO_Port GPIOE
#define RESET1_Pin GPIO_PIN_14
#define RESET1_GPIO_Port GPIOE
#define nSLEEP1_Pin GPIO_PIN_15
#define nSLEEP1_GPIO_Port GPIOE
#define LED_WARNING_Pin GPIO_PIN_0
#define LED_WARNING_GPIO_Port GPIOD
#define LED_ERROR_Pin GPIO_PIN_1
#define LED_ERROR_GPIO_Port GPIOD
#define LED_CANB_Pin GPIO_PIN_2
#define LED_CANB_GPIO_Port GPIOD
#define LED_CANA_Pin GPIO_PIN_3
#define LED_CANA_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_6
#define LED4_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_7
#define LED3_GPIO_Port GPIOD
#define STEP2_Pin GPIO_PIN_1
#define STEP2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
