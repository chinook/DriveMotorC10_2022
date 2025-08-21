/*
 * can.c
 *
 *  Created on: Aug 14, 2025
 *      Author: thoma
 */
#include "can.h"

#include "stm32f4xx_hal.h"

#include "chinook_can_ids.h"
#include "main.h"
#include "motor.h"
#include "state_machine.h"

CAN_HandleTypeDef *hcan = &hcan1;

uint8_t flag_can_direction = 0;
uint8_t flag_can_speed = 0;
uint32_t can_motor_pitch_speed = 0; //0-100%
uint32_t can_motor_pitch_direction = MOTOR_DIRECTION_STOP;

void ProcessCanMessage() {
	//if (HAL_GPIO_ReadPin(PB1_GPIO_Port, PB1_Pin) == GPIO_PIN_RESET) {
	//	return;
	//}
	//if (HAL_GPIO_ReadPin(PB2_GPIO_Port, PB2_Pin) == GPIO_PIN_RESET) {
	//	return;
	//}

	// Technically CAN data can be 8 bytes but we only send 4-bytes data to the motor driver
	// uint32_t upper_can_data = rxData[4] | (rxData[5] << 8) | (rxData[6] << 16) | (rxData[7] << 24);
	uint32_t can_data = rxData[0] | (rxData[1] << 8) | (rxData[2] << 16)
			| (rxData[3] << 24);

	if (pRxHeader.StdId == CAN_ID_CMD_MARIO_PITCH_DIRECTION) {
		flag_can_direction = 1;
		can_data = (can_data & 0xFF); //SUPER IMPORTANT

		can_motor_pitch_direction = can_data;
	} else if (pRxHeader.StdId == CAN_ID_CMD_MARIO_PITCH_SPEED) {
		flag_can_speed = 1;
		can_data = (can_data & 0xFF); //SUPER IMPORTANT

		//can_motor_pitch_speed = 100;
		can_motor_pitch_speed = can_data;
	} else {
		// Unknown CAN ID
	}
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

