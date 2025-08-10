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

#include <stdlib.h>
#include <stdio.h>
#include <string.h> // memcpy

#include "chinook_can_ids.h"

#include "motor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

enum STATES {
	STATE_INIT = 0,
	STATE_ASSESS_PUSH_BUTTONS,
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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */

// Emergency flags
uint8_t b_rops = 0;
uint8_t b_emergency_stop = 0;

uint8_t b_timer500ms_flag = 0;
uint8_t b_timer50ms_flag = 0;
uint8_t b_timer250ms_flag = 0;

uint8_t flag_can_tx_send = 0;
uint8_t flag_pitch_control = 0;
uint8_t flag_mast_control = 0;

uint8_t flag_send_drive_pitch_config = 0;
uint8_t flag_send_drive_mast_config = 0;

uint32_t speed_stepper_motor_pitch = 0; //0-100%
uint32_t can_pitch_motor_direction = 0;

uint8_t txData[8];
uint8_t rxData[8];

uint8_t can1_recv_flag = 0;
CAN_TxHeaderTypeDef pTxHeader;
CAN_RxHeaderTypeDef pRxHeader;
uint32_t txMailbox;

uint32_t current_state = STATE_INIT;

// CAN data/info to send
typedef struct CAN_TX_Data_ {
	uint32_t pitch_motor_bemf;
	uint32_t mast_motor_bemf;

	uint32_t pitch_motor_fault_stall;
	uint32_t mast_motor_fault_stall;

	uint32_t pitch_motor_mode_feedback;
	uint32_t mast_motor_mode_feedback;

	uint32_t pitch_done;

} CAN_TX_Data;
CAN_TX_Data can_tx_data;

//
// Motor control
//

Motorss motorss;

// TEMP TEMP

uint8_t pb1_value, pb2_value;
uint8_t pb1_update, pb2_update;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */

void ExecuteStateMachine();

uint32_t DoStateInit();
uint32_t DoStateAssessPushButtons();

uint8_t CheckEnableDisableMotor(DRIVE_MOTOR motor);
uint8_t CheckChangeDirectionMotor(DRIVE_MOTOR motor);

uint32_t DoStatePitchControl();
uint32_t DoStateMastControl();
uint32_t DoStateCAN();

uint32_t DoStateROPS();
uint32_t DoStateEmergencyStop();

void DoStateError();

// void SetPWM(uint32_t pwm, uint16_t value);

void SetMotorMode(DRIVE_MOTOR motor, uint32_t can_value);
void SetMotorDirection(DRIVE_MOTOR motor, int32_t can_value);

void ProcessCanMessage();
void CAN_ReceiveFifoCallback(CAN_HandleTypeDef *hcan, uint32_t fifo);

HAL_StatusTypeDef TransmitCAN(uint32_t id, uint8_t *buf, uint8_t size,
		uint8_t with_priority);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay_us(uint16_t delay_us) {
	    // Reset le compteur
	    __HAL_TIM_SET_COUNTER(&htim5, 0);

	    // Démarre le timer
	    HAL_TIM_Base_Start(&htim5);

	    // Attend que le compteur atteigne la durée
	    while (__HAL_TIM_GET_COUNTER(&htim5) < delay_us);

	    // Stoppe le timer (optionnel)
	    HAL_TIM_Base_Stop(&htim5);
}

void delay_ms(uint16_t delay16_ms) {
	while (delay16_ms > 0) {
		htim5.Instance->CNT = 0;
		delay16_ms--;
		while (htim5.Instance->CNT < 1000)
			;
	}
}

uint8_t flag_buttons = 0;
uint8_t timer50ms_flag = 0;
uint8_t timer250ms_counter = 0;
uint8_t timer500ms_counter = 0;

void ExecuteStateMachine() {
	// Check timers
	if (timer500ms_counter >= 10) {

		timer500ms_counter = 0;

		if (flag_drive_fault == 0) {
			HAL_GPIO_TogglePin(LED_CANA_GPIO_Port, LED_CANA_Pin);
		}

		/*

		 DEBUG_SPI_CHATGPT(DRIVE_MAST);

		 uint8_t tx[2] = {0xAA, 0x55}; // Donnée de test
		 uint8_t rx[2] = {0};

		 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
		 HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
		 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);

		 printf("Echo SPI : %02X %02X\n", rx[0], rx[1]);
		 */
	}
	if (timer50ms_flag) {
		timer50ms_flag = 0;
		timer250ms_counter++;
		timer500ms_counter++;

		flag_buttons = 1;
		flag_can_tx_send = 1;
		flag_pitch_control = 1;
		flag_mast_control = 1;

		if (flag_drive_fault == 1) {
			HAL_GPIO_TogglePin(LED_CANA_GPIO_Port, LED_CANA_Pin);
		}
	}
	if (timer250ms_counter >= 5) {
		timer250ms_counter = 0;

		//flag_can_tx_send = 1;

		flag_send_drive_pitch_config = 1;
		flag_send_drive_mast_config = 1;

	}

	// Check for ROPS or emergency stop flags
	if (b_rops) {
		current_state = STATE_ROPS;
	}
	if (b_emergency_stop) {
		current_state = STATE_EMERGENCY_STOP;
	}

	switch (current_state) {
	case STATE_INIT:
		current_state = DoStateInit();
		break;

	case STATE_ASSESS_PUSH_BUTTONS:
		current_state = DoStateAssessPushButtons();
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

uint32_t DoStateInit() {
	b_rops = 0;
	b_emergency_stop = 0;

	b_timer500ms_flag = 0;
	b_timer50ms_flag = 0;
	flag_can_tx_send = 0;

	can1_recv_flag = 0;
	flag_can_tx_send = 0;

	flag_send_drive_pitch_config = 0;
	flag_send_drive_mast_config = 0;

	memset(&can_tx_data, 0, sizeof(CAN_TX_Data));

	InitDrives();

	// Initialize the motor control values
	motorss.motors[DRIVE1].enabled = 0;
	motorss.motors[DRIVE1].request_enable = 0;
	motorss.motors[DRIVE1].request_disable = 0;
	motorss.motors[DRIVE1].mode = MODE_MANUAL;
	motorss.motors[DRIVE1].auto_command = 0;
	motorss.motors[DRIVE1].manual_command = 0;
	motorss.motors[DRIVE1].direction = DIR_STOP;
	motorss.motors[DRIVE1].prev_direction = DIR_STOP;

	motorss.motors[DRIVE2].enabled = 0;
	motorss.motors[DRIVE2].request_enable = 0;
	motorss.motors[DRIVE2].request_disable = 0;
	motorss.motors[DRIVE2].mode = MODE_MANUAL;
	motorss.motors[DRIVE2].auto_command = 0;
	motorss.motors[DRIVE2].manual_command = 0;
	motorss.motors[DRIVE2].direction = DIR_STOP;
	motorss.motors[DRIVE2].prev_direction = DIR_STOP;

	//HAL_Delay(10);
	//EnableDriveExternalPWM(DRIVE_MAST);
	//HAL_Delay(10);

	//SetDirection(DRIVE_PITCH, DIR_FORWARD);
	//SetDirection(DRIVE_MAST, DIR_FORWARD);

	// HAL_GPIO_WritePin(TEST_BIN1_GPIO_Port, TEST_BIN1_Pin, GPIO_PIN_SET);
	// HAL_GPIO_WritePin(TEST_BIN2_GPIO_Port, TEST_BIN2_Pin, GPIO_PIN_SET);

	//HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	//HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);

	motorss.motors[DRIVE1].enabled = 0;

	//SetDirection(DRIVE_PITCH, motors.pitch_motor.manual_direction);
	//delay_us(10);
	DisableDrive(DRIVE1);
	//delay_us(10);
	DisableDrive(DRIVE2);

	//delay_us(10);
	//ResetStatusRegisters(DRIVE_PITCH);
	//delay_us(10);

	return STATE_ASSESS_PUSH_BUTTONS;
}

uint32_t DoStateAssessPushButtons() {
	if (1) {
		flag_buttons = 0;

		CheckDriveStatusRegister(DRIVE1);
		if (HAL_GPIO_ReadPin(PB1_GPIO_Port, PB1_Pin) == GPIO_PIN_RESET) {
			motor_pitch_on = 0;

			WriteSPI(DRIVE1, 7, 0);
			DisableDrive(DRIVE1);
			//ResetDrive(DRIVE1);

			InitDriveMotor(DRIVE1);

		} else if (HAL_GPIO_ReadPin(PB2_GPIO_Port, PB2_Pin) == GPIO_PIN_RESET) {
			EnableDrive(DRIVE1);

			CheckDriveStatusRegister(DRIVE1);

			uint16_t reg_config = ReadSPI(DRIVE1, 0);
			reg_config = ReadSPI(DRIVE1, 2);

			DirectionDrive(DRIVE1, 1);

			motor_pitch_on = 1;

			ReadAndVerifyDriveRegisters(DRIVE1);
		}

		/*
		 if (HAL_GPIO_ReadPin(PB1_GPIO_Port, PB1_Pin) == GPIO_PIN_RESET) {
		 speed_stepper_motor_pitch++;
		 }
		 //175 maximum speed
		 if (HAL_GPIO_ReadPin(PB2_GPIO_Port, PB2_Pin) == GPIO_PIN_RESET) {
		 if (speed_stepper_motor_pitch > 2) {
		 speed_stepper_motor_pitch--;
		 }
		 }*/
	}

	return STATE_PITCH_CONTROL;
}

uint8_t CheckEnableDisableMotor(DRIVE_MOTOR motor) {
	if (motor != DRIVE1 && motor != DRIVE2)
		return 0;

	// Check if requested disable of drive
	if (motorss.motors[motor].request_disable) {
		DisableDrive(motor);
		//delay_us(20);

		// On disable, we reset the commands
		motorss.motors[motor].manual_command = 0;
		motorss.motors[motor].auto_command = 0;
		motorss.motors[motor].prev_direction = DIR_INVALID;

		motorss.motors[motor].request_disable = 0;
		// Make sure we do not reactive drive right after
		motorss.motors[motor].request_enable = 0;

		motorss.motors[motor].enabled = 0;

		return 1; // Indicates enable/disable status changed
	}
	// Check if requested enable of drive
	else if (motorss.motors[motor].request_enable) {
		EnableDrive(motor);
		//delay_us(20);

		motorss.motors[motor].request_enable = 0;

		motorss.motors[motor].enabled = 1;

		return 1; //Indicates enable/disable status changed
	}

	return 0; // Indicates nothing changed
}

uint8_t CheckChangeDirectionMotor(DRIVE_MOTOR motor) {
	if (motor != DRIVE1 && motor != DRIVE2)
		return 0;

	// Check for change of direction
	if (motorss.motors[motor].direction
			!= motorss.motors[motor].prev_direction) {
		//SetDirection(motor, motorss.motors[motor].direction);
		//delay_us(20);

		motorss.motors[motor].prev_direction = motorss.motors[motor].direction;

		return 1; // Indicates direction changed
	}

	return 0; // Indicates direction did not change
}


uint32_t DoStatePitchControl() {
	return STATE_MAST_CONTROL;
	/*
	 if (flag_pitch_control == 1) {
	 flag_pitch_control = 0;
	 }
	 else {
	 return STATE_MAST_CONTROL;
	 }*/

	/*
	 // Periodically re-send the config registers to make sure drive has correct values
	 if (flag_send_drive_pitch_config)
	 {
	 flag_send_drive_pitch_config = 0;
	 SendConfigRegisters(DRIVE_PITCH);
	 }
	 */
	// Check if requested disable of drive
	//if (motorss.motors[DRIVE_PITCH].enabled == 1) {
	//motorss.motors[DRIVE_PITCH].request_disable = 1;
	//}
	/*if (motorss.motors[DRIVE_PITCH].enabled != 1) {
	 motorss.motors[DRIVE_PITCH].request_enable = 1;
	 } */
	CheckEnableDisableMotor(DRIVE2);

	if (motorss.motors[DRIVE2].enabled) {
		speed_stepper_motor_pitch = speed_stepper_motor_pitch;

		// Only try to turn the motor if the pitch drive is enabled
		//if ((motors.motors[DRIVE_PITCH].mode == MODE_MANUAL) && (!b_rops))
		//{
		// Check if manual command was set
		//if (motors.pitch_motor.manual_command)
		//{
		//Step(DRIVE_PITCH);
		//}
		//else {
		//}
		//}
		//else if (motors.motors[DRIVE_PITCH].mode == MODE_AUTOMATIC || b_rops == 1)
		//{
		// Check if automatic command was set
		//if (motors.pitch_motor.auto_command)
		//{
		//Step(DRIVE_PITCH);
		// delay_ms(2);
		//for (int i = 0; i < 2000; ++i);
		// Decrease number of steps to do
		//--motors.pitch_motor.auto_command;
		// Check if command is done
		//if (motors.pitch_motor.auto_command == 0)
		//{
		// Pitch done
		//can_tx_data.pitch_done = 1;
		//motors.pitch_motor.request_disable = 1;
		//motors.pitch_motor.prev_auto_direction = DIR_INVALID;
		//}
		//}
		//}
	} else {
	}

	can_tx_data.pitch_motor_mode_feedback = motorss.motors[DRIVE2].mode;

	// Check for faults or stall errors
	uint8_t stall = !HAL_GPIO_ReadPin(nSTALL2_GPIO_Port, nSTALL2_Pin);
	uint8_t fault = !HAL_GPIO_ReadPin(nFAULT2_GPIO_Port, nFAULT2_Pin);
	can_tx_data.pitch_motor_fault_stall = (fault + (stall << 1));

	if (fault) {
		ResetStatusRegisters(DRIVE2); //nul
	}

	return STATE_MAST_CONTROL;
}

uint32_t DoStateMastControl() {
	return STATE_CAN;
	if (flag_mast_control == 1) {
		flag_mast_control = 0;
	} else {
		return STATE_CAN;
	}
	/*
	 // Periodically re-send the config registers to make sure drive has correct values
	 if (flag_send_drive_mast_config)
	 {
	 flag_send_drive_mast_config = 0;

	 SendConfigRegisters(DRIVE_MAST);
	 }
	 */
	// Check if requested disable of drive
	if (motorss.motors[DRIVE1].enabled != 1) {
		motorss.motors[DRIVE1].request_enable = 1;
	}
	CheckEnableDisableMotor(DRIVE1);

	uint8_t enableChanged = CheckEnableDisableMotor(DRIVE2);
	if (enableChanged) {
		// Make sure to disable the PWMs if drive was disabled
		if (motorss.motors[DRIVE1].enabled == 0) {
			DriveMastStop();
			//delay_us(20);
		}
	}

	//Check change of direction
	uint8_t directionChanged = CheckChangeDirectionMotor(DRIVE1);

	if (motorss.motors[DRIVE1].enabled) {
		if (motorss.motors[DRIVE1].direction == DIR_STOP) {
			DriveMastStop();
			//delay_us(20);
		} else if (motorss.motors[DRIVE1].direction == DIR_LEFT) {
			DriveMastLeft();
			//delay_us(20);
		} else if (motorss.motors[DRIVE1].direction == DIR_RIGHT) {
			DriveMastRight();
			//delay_us(20);
		}
	}

	/*
	 if (motors.mast_motor.enabled)
	 {
	 // Only try to turn the mast motor if mast drive is enabled
	 if (motors.mast_motor.mode == MODE_MANUAL)
	 {
	 // Check if manual command was set
	 if (motors.mast_motor.manual_command)
	 {
	 for (int i = 0; i < 400; ++i);
	 }
	 }
	 else if (motors.mast_motor.mode == MODE_AUTOMATIC)
	 {
	 // Check if automatic command was set
	 if (motors.mast_motor.auto_command)
	 {
	 for (int i = 0; i < 400; ++i);
	 }
	 }
	 }
	 */

	can_tx_data.mast_motor_mode_feedback = motorss.motors[DRIVE1].mode;

	// Check for faults or stall errors
	uint8_t stall = !HAL_GPIO_ReadPin(nSTALL1_GPIO_Port, nSTALL1_Pin);
	uint8_t fault = !HAL_GPIO_ReadPin(nFAULT1_GPIO_Port, nFAULT1_Pin);
	can_tx_data.mast_motor_fault_stall = (fault + (stall << 1));

	if (fault) {
		ResetStatusRegisters(DRIVE1); //nul
	}

	return STATE_CAN;
}

uint32_t DoStateCAN() {
	if (flag_can_tx_send) // Sent every 50ms
	{
		flag_can_tx_send = 0;

		uint32_t pitch_mode = can_tx_data.pitch_motor_mode_feedback;
		uint32_t pitch_mode_msg = (
				(pitch_mode == MODE_MANUAL) ?
						MOTOR_MODE_MANUAL : MOTOR_MODE_AUTOMATIC);
		TransmitCAN(CAN_ID_STATE_DRIVEMOTOR_PITCH_MODE,
				(uint8_t*) &pitch_mode_msg, 4, 0);

		uint32_t mast_mode = can_tx_data.mast_motor_mode_feedback;
		uint32_t mast_mode_msg = (
				(mast_mode == MODE_MANUAL) ?
						MOTOR_MODE_MANUAL : MOTOR_MODE_AUTOMATIC);
		TransmitCAN(CAN_ID_STATE_DRIVEMOTOR_MAST_MODE,
				(uint8_t*) &mast_mode_msg, 4, 0);

		static float test = 0;
		static float debug_log_4_value = 0;
		debug_log_4_value = debug_log_4_value + test;
		TransmitCAN(CAN_ID_MARIO_VAL_DEBUG_LOG_4, (uint8_t*) &debug_log_4_value,
				4, 0);
		test++;

		//TransmitCAN(CAN_ID_DRIVEMOTOR_PITCH_MOVE_DONE, (uint8_t*)&can_tx_data.pitch_done, 4, 0);

		/*
		 TransmitCAN(DRIVEMOTOR_PITCH_FAULT_STALL, (uint8_t*)&can_tx_data.pitch_motor_fault_stall, 4, 0);
		 delay_us(50);

		 TransmitCAN(DRIVEMOTOR_MAST_FAULT_STALL, (uint8_t*)&can_tx_data.mast_motor_fault_stall, 4, 0);
		 delay_us(50);

		 TransmitCAN(DRIVEMOTOR_ROPS_FEEDBACK, (uint8_t*)&b_rops, 4, 0);
		 delay_us(50);
		 */

	}

	// return STATE_PITCH_CONTROL;
	return STATE_ASSESS_PUSH_BUTTONS;
}

uint32_t DoStateROPS() {
	while (b_rops) {
		//delay_us(10);

		// Check timers
		if (b_timer500ms_flag) {
			b_timer500ms_flag = 0;
			HAL_GPIO_TogglePin(LED_CANA_GPIO_Port, LED_CANA_Pin);
		}
		if (b_timer50ms_flag) {
			b_timer50ms_flag = 0;
			flag_pitch_control = 1;
			flag_mast_control = 1;
			// flag_can_tx_send = 1;
		}
		if (b_timer250ms_flag) {
			b_timer250ms_flag = 0;

			flag_can_tx_send = 1;
		}

		// Safety check, if we have a command from MARIO, make sure drive is enabled
		//if (motors.motors[DRIVE_PITCH].auto_command && !motors.motors[DRIVE_PITCH].enabled)
		//	motors.motors[DRIVE_PITCH].request_enable = 1;
		//motors.motors[DRIVE_PITCH].mode = MODE_AUTOMATIC;

		DoStateAssessPushButtons();
		DoStatePitchControl();
		DoStateMastControl();
		DoStateCAN();
	}

	return STATE_PITCH_CONTROL;
}

uint32_t DoStateEmergencyStop() {
	while (b_emergency_stop) {

	}

	return STATE_PITCH_CONTROL;
}

void DoStateError() {
	Error_Handler();
}

//uint16_t test_debug_log_can_message[200] = {0};
//uint8_t test_debug_log_can_message_counter = 0;
void SetMotorMode(DRIVE_MOTOR motor, uint32_t can_value) {
	can_value = (can_value & 0xFF); //SUPER IMPORTANT

	/*if (test_debug_log_can_message_counter > 200) {
	 test_debug_log_can_message_counter = 0;
	 } else {
	 test_debug_log_can_message_counter++;
	 }
	 test_debug_log_can_message[test_debug_log_can_message_counter] = can_value; */

	uint32_t motor_mode = MODE_MANUAL;
	if (can_value == MOTOR_MODE_MANUAL) {
		motor_mode = MODE_MANUAL;
	} else if (can_value == MOTOR_MODE_AUTOMATIC) {
		motor_mode = MODE_AUTOMATIC;
	} else {
		return; // Do not set motor mode if mode value from CAN is invalid
	}

	motorss.motors[motor].mode = motor_mode;
}

void SetMotorDirection(DRIVE_MOTOR motor, int32_t can_value) {
	can_value = (can_value & 0xFF); //SUPER IMPORTANT

	/*if (test_debug_log_can_message_counter > 200) {
	 test_debug_log_can_message_counter = 0;
	 } else {
	 test_debug_log_can_message_counter++;
	 }
	 test_debug_log_can_message[test_debug_log_can_message_counter] = can_value;*/

	uint32_t motor_direction = DIR_INVALID;
	if (can_value == MOTOR_DIRECTION_STOP)
		motor_direction = DIR_STOP;
	else if (can_value == MOTOR_DIRECTION_LEFT)
		motor_direction = DIR_LEFT;
	else if (can_value == MOTOR_DIRECTION_RIGHT)
		motor_direction = DIR_RIGHT;
	else
		return;

	motorss.motors[motor].direction = motor_direction;

	// Check change of direction
	CheckChangeDirectionMotor(motor);
}

void ProcessCanMessage() {
	if (HAL_GPIO_ReadPin(PB1_GPIO_Port, PB1_Pin) == GPIO_PIN_RESET) {
		return 0;
	}
	if (HAL_GPIO_ReadPin(PB2_GPIO_Port, PB2_Pin) == GPIO_PIN_RESET) {
		return 0;
	}
	//return 0;

	typedef union BytesToType_ {
		struct {
			uint8_t bytes[4];
		};
		int32_t int_val;
		uint32_t uint_val;
		float float_val;
	} BytesToType;
	static BytesToType bytesToType;

	// Technically CAN data can be 8 bytes but we only send 4-bytes data to the motor driver
	// uint32_t upper_can_data = rxData[4] | (rxData[5] << 8) | (rxData[6] << 16) | (rxData[7] << 24);
	uint32_t can_data = rxData[0] | (rxData[1] << 8) | (rxData[2] << 16)
			| (rxData[3] << 24);

	//
	// Motor Modes
	//
	// TODO: (Marc) Should one have precedence over the other ? What if steering wheel sets mode that is then overwritten by mario ?
	if (pRxHeader.StdId == CAN_ID_CMD_MARIO_PITCH_MODE) {
		SetMotorMode(DRIVE2, can_data);
	} else if (pRxHeader.StdId == CAN_ID_CMD_MARIO_MAST_MODE) {
		SetMotorMode(DRIVE1, can_data);
	}
	//
	// MARIO Manual motor commands
	//
	else if (pRxHeader.StdId == CAN_ID_CMD_MARIO_PITCH_DIRECTION) {
		SetMotorDirection(DRIVE1, can_data);


	} else if (pRxHeader.StdId == CAN_ID_CMD_MARIO_MAST_DIRECTION) {
		SetMotorDirection(DRIVE1, can_data);
	}
	//
	// MARIO Automatic motor commands
	//
	else if (pRxHeader.StdId == CAN_ID_CMD_MARIO_PITCH_SPEED) {
		can_data = (can_data & 0xFF); //SUPER IMPORTANT

		//speed_stepper_motor_pitch = 100;
		speed_stepper_motor_pitch = can_data;
	}
	/*
	 else if (pRxHeader.StdId == MARIO_MAST_CMD)
	 {
	 // Automatic Mode MArio mast command -> number of steps to turn
	 memcpy(bytesToType.bytes, rxData, 4);
	 int32_t cmd_val = bytesToType.int_val;

	 if (cmd_val == 0)
	 motors.mast_motor.auto_direction = DIR_STOP;
	 else if (cmd_val > 0)
	 motors.mast_motor.auto_direction = DIR_LEFT;
	 else
	 motors.mast_motor.auto_direction = DIR_RIGHT;

	 if (motors.mast_motor.mode == MODE_AUTOMATIC)
	 motors.mast_motor.request_enable = 1;
	 motors.mast_motor.auto_command = 1;  // Turn is activated
	 }
	 //
	 // ROPS + Error commands
	 //
	 else if (pRxHeader.StdId == MARIO_PITCH_EMERGENCY_STOP)
	 {
	 b_emergency_stop = 1;
	 motors.pitch_motor.request_disable = 1;
	 }
	 else if (pRxHeader.StdId == MARIO_MAST_EMERGENCY_STOP)
	 {
	 b_emergency_stop = 1;
	 motors.mast_motor.request_disable = 1;
	 }
	 else if (pRxHeader.StdId == MARIO_DRIVE_MOTOR_RESET)
	 {
	 // TODO: (Marc) Implement soft reset of MCU ?
	 motors.pitch_motor.request_disable = 1;
	 motors.mast_motor.request_disable = 1;

	 DoStateInit();
	 }*/
	else if (pRxHeader.StdId == CAN_ID_CMD_MARIO_ROPS) {
		uint8_t rops_data = (can_data & 0xFF);
		if (rops_data == ROPS_ENABLE)
			b_rops = 1;
		else if (rops_data == ROPS_DISABLE)
			b_rops = 0;
		else {
			// Unknown value for ROPS command, assume cmd was to activate ROPS
			b_rops = 1;
		}
	} else {
		// Unknown CAN ID
	}
}

void CAN_ReceiveFifoCallback(CAN_HandleTypeDef *hcan, uint32_t fifo) {

	uint32_t num_messages = HAL_CAN_GetRxFifoFillLevel(hcan, fifo);
	for (int i = 0; i < num_messages; ++i) {
		if (HAL_CAN_GetRxMessage(hcan, fifo, &pRxHeader, rxData) != HAL_OK) {
			Error_Handler();
		}

		ProcessCanMessage();
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	//HAL_GPIO_TogglePin(LED_CANB_GPIO_Port, LED_CANB_Pin);
	CAN_ReceiveFifoCallback(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	//HAL_GPIO_TogglePin(LED_CANB_GPIO_Port, LED_CANB_Pin);
	CAN_ReceiveFifoCallback(hcan, CAN_RX_FIFO1);
}

// CAN error callbacks
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan) {
	// TODO: (Marc) Error detection/handling
	//HAL_GPIO_TogglePin(LED_CANB_GPIO_Port, LED_CANB_Pin);
}

void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan) {
	// TODO: (Marc) Error detection/handling
	//HAL_GPIO_TogglePin(LED_CANB_GPIO_Port, LED_CANB_Pin);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
	// TODO: (Marc) Error detection/handling
	//HAL_GPIO_TogglePin(LED_CANB_GPIO_Port, LED_CANB_Pin);
}

HAL_StatusTypeDef TransmitCAN(uint32_t id, uint8_t *buf, uint8_t size,
		uint8_t with_priority) {
	// CAN_TxHeaderTypeDef msg;
	pTxHeader.StdId = id;
	pTxHeader.IDE = CAN_ID_STD;
	pTxHeader.RTR = CAN_RTR_DATA;
	pTxHeader.DLC = size; // Number of bytes to send
	pTxHeader.TransmitGlobalTime = DISABLE;

	uint8_t found_mailbox = 0;
	for (int i = 0; i < 10; ++i) {
		// Check that mailbox is available for tx
		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
			found_mailbox = 1;
			break;
		}
		// Otherwise wait until free mailbox
		// for (int j = 0; j < 500; ++j) {}
		//delay_us(50);
	}
	if (!found_mailbox) {
		// TODO: (Marc) Should really be the error led once it's been soldered
		//HAL_GPIO_WritePin(LED_CANB_GPIO_Port, LED_CANB_Pin, GPIO_PIN_SET);
	}

	if (with_priority) {
		// If message is important, make sure no other messages are queud to ensure it will be sent after any other
		// values that could override it.
		for (int i = 0; i < 10; ++i) {
			// Check that all 3 mailboxes are empty
			if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 3)
				break;
			// Otherwise wait until 3 free mailbox
			// for (int j = 0; j < 500; ++j) {}
			//delay_us(50);
		}
	}

	uint32_t mb;
	HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, buf, &mb);
	if (ret != HAL_OK) {
		return ret;
	}

	// Update the CAN led
	HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start_IT(&htim7);

	current_state = STATE_INIT;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		ExecuteStateMachine();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 128;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_3TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
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

	/*
	 CAN_FilterTypeDef filter_all;
	 // All common bits go into the ID register

	 filter_all.FilterIdHigh = 0x0000;
	 filter_all.FilterIdLow = 0x0000;

	 // Which bits to compare for filter
	 filter_all.FilterMaskIdHigh = 0x0000;
	 filter_all.FilterMaskIdLow = 0x0000;


	 filter_all.FilterIdHigh = 0x0000;
	 filter_all.FilterIdLow = 0x0070;

	 // Which bits to compare for filter
	 filter_all.FilterMaskIdHigh = 0x0000;
	 filter_all.FilterMaskIdLow = 0x07F0;

	 filter_all.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	 filter_all.FilterBank = 0; // Which filter to use from the assigned ones
	 filter_all.FilterMode = CAN_FILTERMODE_IDMASK;
	 filter_all.FilterScale = CAN_FILTERSCALE_32BIT;
	 filter_all.FilterActivation = CAN_FILTER_ENABLE;
	 filter_all.SlaveStartFilterBank = 14; // How many filters to assign to CAN1
	 if (HAL_CAN_ConfigFilter(&hcan1, &filter_all) != HAL_OK)
	 {
	 Error_Handler();
	 }
	 */

	CAN_FilterTypeDef sf_fifo0;
	// All common bits go into the ID register
	sf_fifo0.FilterIdHigh = DRIVEMOTOR_FIFO0_RX_FILTER_ID_HIGH;
	sf_fifo0.FilterIdLow = DRIVEMOTOR_FIFO0_RX_FILTER_ID_LOW;

	// Which bits to compare for filter
	sf_fifo0.FilterMaskIdHigh = 0x0000;
	sf_fifo0.FilterMaskIdLow = (FIFO0_RX_FILTER_MASK_LOW & 0x07FF);

	sf_fifo0.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sf_fifo0.FilterBank = 0; // Which filter to use from the assigned ones
	sf_fifo0.FilterMode = CAN_FILTERMODE_IDMASK;
	sf_fifo0.FilterScale = CAN_FILTERSCALE_32BIT;
	sf_fifo0.FilterActivation = CAN_FILTER_ENABLE;
	sf_fifo0.SlaveStartFilterBank = 14; // How many filters to assign to CAN1
	if (HAL_CAN_ConfigFilter(&hcan1, &sf_fifo0) != HAL_OK) {
		Error_Handler();
	}

	CAN_FilterTypeDef sf_fifo1;
	// All common bits go into the ID register
	//sf_fifo1.FilterIdHigh = 0x0000;
	//sf_fifo1.FilterIdLow = 0x0000;
	//sf_fifo1.FilterMaskIdHigh = 0x0000;
	//sf_fifo1.FilterMaskIdLow = 0x0000;
	sf_fifo1.FilterIdHigh = DRIVEMOTOR_FIFO1_RX_FILTER_ID_HIGH;
	sf_fifo1.FilterIdLow = DRIVEMOTOR_FIFO1_RX_FILTER_ID_LOW;

	// Which bits to compare for filter
	sf_fifo1.FilterMaskIdHigh = 0x0000;
	sf_fifo1.FilterMaskIdLow = (FIFO1_RX_FILTER_MASK_LOW & 0x7FF);

	sf_fifo1.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	sf_fifo1.FilterBank = 1; // Which filter to use from the assigned ones
	sf_fifo1.FilterMode = CAN_FILTERMODE_IDMASK;
	sf_fifo1.FilterScale = CAN_FILTERSCALE_32BIT;
	sf_fifo1.FilterActivation = CAN_FILTER_ENABLE;
	sf_fifo1.SlaveStartFilterBank = 14; // How many filters to assign to CAN1
	if (HAL_CAN_ConfigFilter(&hcan1, &sf_fifo1) != HAL_OK) {
		Error_Handler();
	}

	//if (HAL_CAN_RegisterCallback(&hcan1, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, can_irq))
	//{
	//	  Error_Handler();
	//}
	if (HAL_CAN_Start(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan1,
			CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();
	}
	/*
	 if (HAL_CAN_ActivateNotification(&hcan1,
	 (CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN |
	 CAN_IT_RX_FIFO1_FULL | CAN_IT_RX_FIFO1_OVERRUN)) != HAL_OK)
	 {
	 Error_Handler();
	 }
	 */

  /* USER CODE END CAN1_Init 2 */

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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

	// HAL_TIM_PWM_Start(&htim1, channel);
  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 48;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500;
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
  htim4.Init.Prescaler = 639;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4999;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 63;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFF;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

	HAL_TIM_Base_Start(&htim5);

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 63;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 480;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 63;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 2;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, FT_RESET_Pin|SPI_CS2_Pin|SPI_CS1_Pin|BIN2_1_Pin
                          |BIN1_1_Pin|DIR1_Pin|STEP1_Pin|RESET1_Pin
                          |nSLEEP1_Pin|STEP2_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : FT_RESET_Pin BIN2_1_Pin BIN1_1_Pin DIR1_Pin
                           STEP1_Pin RESET1_Pin nSLEEP1_Pin STEP2_Pin */
  GPIO_InitStruct.Pin = FT_RESET_Pin|BIN2_1_Pin|BIN1_1_Pin|DIR1_Pin
                          |STEP1_Pin|RESET1_Pin|nSLEEP1_Pin|STEP2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_CS2_Pin SPI_CS1_Pin */
  GPIO_InitStruct.Pin = SPI_CS2_Pin|SPI_CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
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

  /*Configure GPIO pins : PB2_Pin PB1_Pin */
  GPIO_InitStruct.Pin = PB2_Pin|PB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_WARNING_Pin LED_ERROR_Pin LED_CANB_Pin LED_CANA_Pin
                           LED1_Pin LED2_Pin LED4_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED_WARNING_Pin|LED_ERROR_Pin|LED_CANB_Pin|LED_CANA_Pin
                          |LED1_Pin|LED2_Pin|LED4_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// EXTI Line External Interrupt ISR Handler CallBack
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_9) // PushButton 1
	{
		//HAL_GPIO_TogglePin(LED_CANA_GPIO_Port, LED_CANA_Pin);
		pb1_value = 1;
		pb1_update = 1;
	}

	// DISABLED DISABLED DISABLED

	else if (GPIO_Pin == GPIO_PIN_8) // PushButton 2
	{
		//HAL_GPIO_TogglePin(LED_CANB_GPIO_Port, LED_CANB_Pin);
		pb2_value = 1;
		pb2_update = 1;
	}

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
	while (1) {
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
