/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h> // memcpy

#include "chinook_can_ids.h"


#include "motor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

enum STATES
{
	STATE_INIT = 0,
	STATE_PITCH_CONTROL,
	STATE_MAST_CONTROL,
	STATE_CAN,

	STATE_ROPS,
	STATE_EMERGENCY_STOP,


	STATE_ERROR = 0xFF
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Emergency flags
uint8_t b_rops = 0;
uint8_t b_emergency_stop = 0;

uint8_t b_timer_flag = 0;

int32_t pitch_cmd_nbr_steps = 0;
int32_t mast_cmd_dir = 0;


uint8_t txData[8];
uint8_t rxData[8];

uint8_t can1_recv_flag = 0;
CAN_TxHeaderTypeDef pTxHeader;
CAN_RxHeaderTypeDef pRxHeader;
uint32_t txMailbox;

// Motor modes
uint8_t pitch_mode = MODE_MANUAL;
uint8_t mast_mode = MODE_MANUAL;

uint32_t current_state = STATE_INIT;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void ExecuteStateMachine();

uint32_t DoStateInit();
uint32_t DoStatePitchControl();
uint32_t DoStateMastControl();
uint32_t DoStateCAN();

uint32_t DoStateROPS();
uint32_t DoStateEmergencyStop();

void DoStateError();


HAL_StatusTypeDef TransmitCAN(uint8_t id, uint8_t* buf, uint8_t size);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void ExecuteStateMachine()
{
	// Check for ROPS or emergency stop flags
	if (b_rops)
	{
		current_state = STATE_ROPS;
	}
	if (b_emergency_stop)
	{
		current_state = STATE_EMERGENCY_STOP;
	}

	switch (current_state)
	{
	case STATE_INIT:
		current_state = DoStateInit();
		break;

	case STATE_PITCH_CONTROL:
		current_state = DoStatePitchControl();
		break;

	case STATE_MAST_CONTROL:
		current_state = DoStateMastControl();
		break;

	case STATE_CAN:
		current_state = DoStateCAN();
		break;

	case STATE_ROPS:
		current_state = DoStateROPS();
		break;

	case STATE_EMERGENCY_STOP:
		current_state = DoStateEmergencyStop();
		break;

	case STATE_ERROR:
		DoStateError();
		// In case we exit error handler, restart the state machine
		current_state = DoStateInit();
		break;

	default:
		current_state = DoStateInit();
		break;
	};
}

uint32_t DoStateInit()
{
	b_rops = 0;
	b_emergency_stop = 0;

	b_timer_flag = 0;

	pitch_cmd_nbr_steps = 0;
	mast_cmd_dir = 0;

	pitch_mode = MODE_MANUAL;
	mast_mode = MODE_MANUAL;

	can1_recv_flag = 0;

	InitDrives(&hspi1, &htim1);

	return STATE_PITCH_CONTROL;
}

uint32_t DoStatePitchControl()
{
	if (pitch_cmd_nbr_steps != 0)
	{
		for (int i = 0; i < pitch_cmd_nbr_steps; ++i)
		{
			Step(DRIVE_PITCH);
		}
	}
	return STATE_MAST_CONTROL;
}

uint32_t DoStateMastControl()
{
	if (mast_cmd_dir == 0)
	{
		// TODO: (Marc)  MastStop();
	}
	else if (mast_cmd_dir > 0)
	{

	}
	return STATE_CAN;
}

uint32_t DoStateCAN()
{
	return STATE_PITCH_CONTROL;
}

uint32_t DoStateROPS()
{
	while (b_rops)
	{

	}

	return STATE_PITCH_CONTROL;
}

uint32_t DoStateEmergencyStop()
{
	while (b_emergency_stop)
	{

	}

	return STATE_PITCH_CONTROL;
}

void DoStateError()
{
	Error_Handler();
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	typedef union RxToInt_
	{
		struct
		{
			uint8_t bytes[4];
		};
		int32_t int_val;
	} RxToInt;
	static RxToInt rxToInt;

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &pRxHeader, rxData) != HAL_OK)
	{
		Error_Handler();
	}

	if (pRxHeader.StdId == MARIO_PITCH_MODE_CMD)
	{
		pitch_mode = rxData[0];
	}
	else if (pRxHeader.StdId == MARIO_MAST_MODE_CMD)
	{
		mast_mode = rxData[0];
	}
	else if (pRxHeader.StdId == MARIO_ROPS_CMD)
	{
		b_rops = 1;
	}
	else if (pRxHeader.StdId == MARIO_PITCH_EMERGENCY_STOP)
	{
		b_emergency_stop = 1;
	}
	else if (pRxHeader.StdId == MARIO_MAST_EMERGENCY_STOP)
	{
		b_emergency_stop = 1;
	}
	else if (pRxHeader.StdId == MARIO_DRIVE_MOTOR_RESET)
	{
		// TODO: (Marc) Implement soft reset
	}
	else if (pRxHeader.StdId == MARIO_PITCH_CMD)
	{
		memcpy(rxToInt.bytes, rxData, 4);
		pitch_cmd_nbr_steps = rxToInt.int_val;
	}
	else if (pRxHeader.StdId == MARIO_MAST_CMD)
	{
		memcpy(rxToInt.bytes, rxData, 4);
		mast_cmd_dir = rxToInt.int_val;
	}
	// Volant commands
	else if (pRxHeader.StdId == VOLANT_PITCH_MODE_CMD)
	{
		pitch_mode = rxData[0];
	}
	else if (pRxHeader.StdId == VOLANT_MAST_MODE_CMD)
	{
		mast_mode = rxData[0];
	}
	else if (pRxHeader.StdId == VOLANT_MANUAL_PITCH_DIR)
	{
		uint32_t manual_pitch_dir;
		memcpy(&manual_pitch_dir, rxData, 4);
	}
	else if (pRxHeader.StdId == VOLANT_MANUAL_MAST_DIR)
	{
		uint32_t manual_mast_dir;
		memcpy(&manual_mast_dir, rxData, 4);
	}
	else if (pRxHeader.StdId == VOLANT_MANUAL_ROPS_CMD)
	{
		b_rops = 1;
	}
	else
	{
		// Unknown CAN ID
	}
}
/*
// Motor modes
#define VOLANT_PITCH_MODE_CMD 0x31
#define VOLANT_MAST_MODE_CMD 0x32

// Motor Manual Control commands
#define VOLANT_MANUAL_PITCH_DIR 0x33
#define VOLANT_MANUAL_MAST_DIR 0x34
#define VOLANT_MANUAL_ROPS_CMD 0x35
*/

HAL_StatusTypeDef TransmitCAN(uint8_t id, uint8_t* buf, uint8_t size)
{
	// CAN_TxHeaderTypeDef msg;
	pTxHeader.StdId = id;
	pTxHeader.IDE = CAN_ID_STD;
	pTxHeader.RTR = CAN_RTR_DATA;
	pTxHeader.DLC = size; // Number of bytes to send
	pTxHeader.TransmitGlobalTime = DISABLE;

	uint32_t mb;
	HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, buf, &mb);
	if (ret != HAL_OK)
		return ret;

	// Update the CAN led
	// ToggleLed(LED_CAN);
	return ret;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim3);

  current_state = STATE_INIT;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_Delay(100);
  DoStateInit();

  HAL_GPIO_WritePin(LED_CANA_GPIO_Port, LED_CANA_Pin, 0);
  HAL_GPIO_WritePin(LED_CANB_GPIO_Port, LED_CANB_Pin, 0);
  //HAL_GPIO_WritePin(LED_CANA_GPIO_Port, LED_CANA_Pin, 1);
  //HAL_GPIO_WritePin(LED_CANB_GPIO_Port, LED_CANB_Pin, 1);

  //SetDirection(DRIVE_PITCH, DIR_FORWARD);
  //SetDirection(DRIVE_MAST, DIR_FORWARD);
  HAL_Delay(10);

  uint8_t mot_direction = 0;
  SetDirection(DRIVE_MAST, mot_direction);

  while (1)
  {
	  // HAL_GPIO_TogglePin(LED_CANA_GPIO_Port, LED_CANA_Pin);
	  // HAL_GPIO_TogglePin(LED_CANB_GPIO_Port, LED_CANB_Pin);

	  if (b_timer_flag)
	  {
		  b_timer_flag = 0;

		  HAL_GPIO_TogglePin(LED_CANA_GPIO_Port, LED_CANA_Pin);
	  }


	  // ExecuteStateMachine();
	  //InitDrives(&hspi1, &htim1);
	  //DEBUG_SPI(DRIVE_PITCH)
	  //HAL_Delay(10);

	  //for (int i = 0; i < 1000; ++i) {}
	  //Step(DRIVE_MAST);
	  //Step(DRIVE_PITCH);

	  for (int i = 0; i < 1000; ++i) {}
	  // DEBUG_SPI(DRIVE_MAST);

	  uint8_t update_dir = 0;
	  uint8_t step = 0;

	  /*
	  if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(PB1_GPIO_Port, PB1_Pin))
	  {
		  if (mot_direction != 0)
			  update_dir = 1;
		  mot_direction = 0;

		  step = 1;
	  }*/

	  if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(PB2_GPIO_Port, PB2_Pin))
	  {
		  if (mot_direction != 1)
			  update_dir = 1;
		  mot_direction = 1;

		  step = 1;
	  }

	  if (update_dir)
	  {
		  SetDirection(DRIVE_MAST, mot_direction);
		  for (int i = 0; i < 2000; ++i) {}
	  }

	  if (step)
	  {
		  Step(DRIVE_MAST);
		  for (int i = 0; i < 750; ++i) {}
	  }

	  //Step(DRIVE_PITCH);

	  uint8_t stall1 = HAL_GPIO_ReadPin(nSTALL1_GPIO_Port, nSTALL1_Pin);
	  uint8_t fault1 = HAL_GPIO_ReadPin(nFAULT1_GPIO_Port, nFAULT1_Pin);

	  uint8_t fault2 = HAL_GPIO_ReadPin(nFAULT2_GPIO_Port, nFAULT2_Pin);
	  if (fault2 == 0)
	  {
		  HAL_GPIO_WritePin(LED_CANB_GPIO_Port, LED_CANB_Pin, GPIO_PIN_SET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(LED_CANB_GPIO_Port, LED_CANB_Pin, GPIO_PIN_RESET);
	  }

	  // HAL_Delay(250);
	  // ExecuteStateMachine();
	  // TODO: (Marc) Better with timer resolution
	  //HAL_Delay(5);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

	CAN_FilterTypeDef sf_mario;
	// All common bits go into the ID register
	sf_mario.FilterIdHigh = MARIO_RX_FILTER_ID_HIGH;
	sf_mario.FilterIdLow = MARIO_RX_FILTER_ID_LOW;

	// Which bits to compare for filter
	sf_mario.FilterMaskIdHigh = MARIO_RX_FILTER_MASK_HIGH;
	sf_mario.FilterMaskIdLow = MARIO_RX_FILTER_MASK_LOW;

	sf_mario.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sf_mario.FilterBank = 18; // Which filter to use from the assigned ones
	sf_mario.FilterMode = CAN_FILTERMODE_IDMASK;
	sf_mario.FilterScale = CAN_FILTERSCALE_32BIT;
	sf_mario.FilterActivation = CAN_FILTER_ENABLE;
	sf_mario.SlaveStartFilterBank = 20; // How many filters to assign to CAN1
	if (HAL_CAN_ConfigFilter(&hcan1, &sf_mario) != HAL_OK)
	{
	  Error_Handler();
	}

	CAN_FilterTypeDef sf_volant;
	// All common bits go into the ID register
	sf_volant.FilterIdHigh = VOLANT_RX_FILTER_ID_HIGH;
	sf_volant.FilterIdLow = VOLANT_RX_FILTER_ID_LOW;

	// Which bits to compare for filter
	sf_volant.FilterMaskIdHigh = VOLANT_RX_FILTER_MASK_HIGH;
	sf_volant.FilterMaskIdLow = VOLANT_RX_FILTER_MASK_LOW;

	sf_volant.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sf_volant.FilterBank = 19; // Which filter to use from the assigned ones
	sf_volant.FilterMode = CAN_FILTERMODE_IDMASK;
	sf_volant.FilterScale = CAN_FILTERSCALE_32BIT;
	sf_volant.FilterActivation = CAN_FILTER_ENABLE;
	sf_volant.SlaveStartFilterBank = 20; // How many filters to assign to CAN1
	if (HAL_CAN_ConfigFilter(&hcan1, &sf_mario) != HAL_OK)
	{
	  Error_Handler();
	}

	//if (HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, can_irq))
	//{
	//	  Error_Handler();
	//}
	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		Error_Handler();
	}

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 16;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 48;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 500;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  // HAL_TIM_PWM_Start(&htim1, channel);

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 480;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, FT_RESET_Pin|SPI_CS2_Pin|SPI_CS1_Pin|DIR1_Pin
                          |STEP1_Pin|RESET1_Pin|nSLEEP1_Pin|STEP2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BIN2_2_Pin|BIN1_2_Pin|DIR2_Pin|nSLEEP2_Pin
                          |RESET2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_WARNING_Pin|LED_ERROR_Pin|LED_CANB_Pin|LED_CANA_Pin
                          |LED1_Pin|LED2_Pin|LED4_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : nSTALL2_Pin nFAULT2_Pin */
  GPIO_InitStruct.Pin = nSTALL2_Pin|nFAULT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : FT_RESET_Pin SPI_CS2_Pin SPI_CS1_Pin DIR1_Pin
                           STEP1_Pin RESET1_Pin nSLEEP1_Pin STEP2_Pin */
  GPIO_InitStruct.Pin = FT_RESET_Pin|SPI_CS2_Pin|SPI_CS1_Pin|DIR1_Pin
                          |STEP1_Pin|RESET1_Pin|nSLEEP1_Pin|STEP2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : BIN2_2_Pin BIN1_2_Pin DIR2_Pin nSLEEP2_Pin
                           RESET2_Pin */
  GPIO_InitStruct.Pin = BIN2_2_Pin|BIN1_2_Pin|DIR2_Pin|nSLEEP2_Pin
                          |RESET2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : nSTALL1_Pin nFAULT1_Pin */
  GPIO_InitStruct.Pin = nSTALL1_Pin|nFAULT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2_Pin */
  GPIO_InitStruct.Pin = PB2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_WARNING_Pin LED_ERROR_Pin LED_CANB_Pin LED_CANA_Pin
                           LED1_Pin LED2_Pin LED4_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED_WARNING_Pin|LED_ERROR_Pin|LED_CANB_Pin|LED_CANA_Pin
                          |LED1_Pin|LED2_Pin|LED4_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

// EXTI Line External Interrupt ISR Handler CallBack
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_9) // PushButton 2
    {
    	//HAL_GPIO_TogglePin(LED_CANA_GPIO_Port, LED_CANA_Pin);
    }

    // DISABLED DISABLED DISABLED
    /*
    else if (GPIO_Pin == GPIO_PIN_8) // PushButton 1
    {
    	//HAL_GPIO_TogglePin(LED_CANB_GPIO_Port, LED_CANB_Pin);
    }
    */
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

