/*
 * state_machine.c
 *
 *  Created on: Aug 14, 2025
 *      Author: thoma
 */
#include "state_machine.h"

#include "stm32f4xx_it.h"

#include "can.h"
#include "chinook_can_ids.h"
#include "main.h"
#include "motor.h"
#include "drv8711_lib.h"

typedef enum STATES {
	STATE_INIT = 0,
	STATE_ASSESS_PUSH_BUTTONS,
	STATE_PITCH_CONTROL,
	STATE_MAST_CONTROL,
	STATE_CAN,

	STATE_ROPS,
	STATE_EMERGENCY_STOP,

	STATE_ERROR = 0xFF
} STATES;

uint32_t current_state = STATE_INIT;

// Emergency flags
uint8_t b_rops = 0;
uint8_t b_emergency_stop = 0;

uint8_t flag_can_tx_send = 0;
uint8_t flag_pitch_control = 0;
uint8_t flag_mast_control = 0;
uint8_t flag_buttons = 0;

uint8_t flag_send_drive_pitch_config = 0;
uint8_t flag_send_drive_mast_config = 0;

uint32_t DoStateInit() {
	b_rops = 0;
	b_emergency_stop = 0;

	flag_can_tx_send = 0;

	flag_send_drive_pitch_config = 0;
	flag_send_drive_mast_config = 0;

	//memset(&can_tx_data, 0, sizeof(CAN_TX_Data));

	InitAndConfigDrive(DRIVE1);
	EnableMotor(DRIVE1);

	return STATE_ASSESS_PUSH_BUTTONS;
}

uint32_t DoStateAssessPushButtons() {
	if (flag_buttons) {
		flag_buttons = 0;

		if (motor_pitch_on == 1) {
			HAL_GPIO_WritePin(LED_CANB_GPIO_Port, LED_CANB_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(LED_CANB_GPIO_Port, LED_CANB_Pin, GPIO_PIN_RESET);
		}

		//CheckDriveStatusRegister(DRIVE1);
		if (pb1_value) {
			EnableMotor(DRIVE1);
			DirectionMotor(DRIVE1, MOTOR_DIRECTION_LEFT);

			//HAL_Delay(25);
			motor_pitch_on = 1;
			//HAL_TIM_Base_Start_IT(&htim4); // motor_pitch_on = 1 after Xms on TIM4

		} else if (pb2_value) {
			EnableMotor(DRIVE1);
			DirectionMotor(DRIVE1, MOTOR_DIRECTION_RIGHT);

			//HAL_Delay(25);
			motor_pitch_on = 1;
			//HAL_TIM_Base_Start_IT(&htim4); // motor_pitch_on = 1 after Xms on TIM4
		}
		/*
		 else if (direction_stepper_motor_pitch != MOTOR_DIRECTION_STOP) {

		 //HAL_Delay(25);
		 motor_pitch_on = 1;
		 //HAL_TIM_Base_Start_IT(&htim4); // motor_pitch_on = 1 after Xms on TIM4
		 //EnableMotor(DRIVE1);
		 if (direction_stepper_motor_pitch == MOTOR_DIRECTION_LEFT) {
		 DirectionMotor(DRIVE1, MOTOR_DIRECTION_LEFT);
		 } else if (direction_stepper_motor_pitch == MOTOR_DIRECTION_RIGHT) {
		 DirectionMotor(DRIVE1, MOTOR_DIRECTION_RIGHT);
		 }
		 }
		 */
		else {
			motor_pitch_on = 0;

			//HAL_GPIO_WritePin(LED_CANB_GPIO_Port, LED_CANB_Pin, GPIO_PIN_SET);
			//flag_drive_fault = 0;

			InitAndConfigDrive(DRIVE1);
		}
	}

	return STATE_PITCH_CONTROL;
}

uint32_t DoStatePitchControl() {

	return STATE_MAST_CONTROL;
}

uint32_t DoStateMastControl() {
	if (flag_mast_control == 1) {
		flag_mast_control = 0;

	}
	return STATE_CAN;
}

uint32_t DoStateCAN() {
	if (flag_can_tx_send) // Sent every 50ms
	{
		flag_can_tx_send = 0;

		//uint32_t pitch_mode = can_tx_data.pitch_motor_mode_feedback;
		//uint32_t pitch_mode_msg = (
		//		(pitch_mode == MODE_MANUAL) ? MOTOR_MODE_MANUAL : MOTOR_MODE_AUTOMATIC);
		//TransmitCAN(CAN_ID_STATE_DRIVEMOTOR_PITCH_MODE, (uint8_t*) &pitch_mode_msg, 4, 0);

		//uint32_t mast_mode = can_tx_data.mast_motor_mode_feedback;
		//uint32_t mast_mode_msg = (
		//		(mast_mode == MODE_MANUAL) ? MOTOR_MODE_MANUAL : MOTOR_MODE_AUTOMATIC);
		//TransmitCAN(CAN_ID_STATE_DRIVEMOTOR_MAST_MODE, (uint8_t*) &mast_mode_msg, 4, 0);

		static float test = 0;
		static float debug_log_4_value = 0;
		debug_log_4_value = debug_log_4_value + test;
		TransmitCAN(CAN_ID_MARIO_VAL_DEBUG_LOG_4, (uint8_t*) &debug_log_4_value, 4, 0);
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
		if (timer_50ms_flag) {
			timer_50ms_flag = 0;

			flag_pitch_control = 1;
			flag_mast_control = 1;
			// flag_can_tx_send = 1;
		}
		if (timer_100ms_flag) {
			timer_100ms_flag = 0;

		}

		if (timer_250ms_flag) {
			timer_250ms_flag = 0;

			flag_can_tx_send = 1;
		}
// Check timers
		if (timer_500ms_flag) {
			timer_500ms_flag = 0;

			HAL_GPIO_TogglePin(LED_CANA_GPIO_Port, LED_CANA_Pin);
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

uint8_t motor_direction_pitch = 0;
void ExecuteStateMachine() {
	DoStateInit();
	ResetDriveStatusRegister(DRIVE1);
	EnableMotor(DRIVE1);


	while (1) {

		// Check timers
		/*
		 if (step_flag) {
		 step_flag = 0;
		 if (motor_pitch_on) {
		 StepDrive(DRIVE1);
		 }
		 }
		 */

		if (timer_50ms_flag) {
			timer_50ms_flag = 0;

			flag_buttons = 1;
			flag_can_tx_send = 1;
			DoStateCAN();
			//flag_pitch_control = 1;
			//flag_mast_control = 1;

			//if (flag_drive_fault == 1) {
			//HAL_GPIO_TogglePin(LED_CANA_GPIO_Port, LED_CANA_Pin);
			//}
		}
		/*
		 if (timer_100ms_flag) {
		 timer_100ms_flag = 0;

		 }

		 if (timer_250ms_flag) {
		 timer_250ms_flag = 0;

		 }
		 */

		if (timer_500ms_flag) {
			timer_500ms_flag = 0;

			//if (flag_drive_fault == 0) {
			HAL_GPIO_TogglePin(LED_CANA_GPIO_Port, LED_CANA_Pin);
			//}
		}

		if (motor_pitch_on == 1) {
			HAL_GPIO_WritePin(LED_CANB_GPIO_Port, LED_CANB_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(LED_CANB_GPIO_Port, LED_CANB_Pin, GPIO_PIN_RESET);
		}

		if (drive_regs[DRIVE1].ctrl_reg.enbl == 1) {
			HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);
		}

		if (flag_buttons) {
			flag_buttons = 0;

			motor_direction_pitch = MOTOR_DIRECTION_STOP;
			motor_pitch_on = 0;

			if (flag_can_speed) {
				flag_can_speed = 0;

				if (can_motor_pitch_speed > 0) {
					motor_pitch_on = 1;
				}
			}
			if (flag_can_direction) {
				flag_can_direction = 0;

				if (can_motor_pitch_direction == MOTOR_DIRECTION_LEFT) {
					motor_direction_pitch = MOTOR_DIRECTION_LEFT;
				} else if (can_motor_pitch_direction == MOTOR_DIRECTION_RIGHT) {
					motor_direction_pitch = MOTOR_DIRECTION_RIGHT;
				}
			}

			if (pb1_value) {
				motor_direction_pitch = MOTOR_DIRECTION_LEFT;
				motor_pitch_on = 1;
			} else if (pb2_value) {
				motor_direction_pitch = MOTOR_DIRECTION_RIGHT;
				motor_pitch_on = 1;
			}

			if (motor_pitch_on == 1) {
				DirectionMotor(DRIVE1, motor_direction_pitch);
				EnableMotor(DRIVE1);

			} else {
				DisableMotor(DRIVE1);
			}

			if (CheckDriveStatusRegister(DRIVE1)) {
				ResetDriveStatusRegister(DRIVE1);
			}
		}
		/*
		 if (motor_pitch_on == 1) {
		 DirectionMotor(DRIVE1, motor_direction_pitch);
		 //ResetDriveStatusRegister(DRIVE1);
		 EnableMotor(DRIVE1);
		 } else {
		 DisableMotor(DRIVE1);
		 //if (drive_regs[DRIVE1].ctrl_reg.enbl == 1) {
		 //InitAndConfigDrive(DRIVE1);
		 //}
		 //ResetDriveStatusRegister(DRIVE1);
		 }*/

		/*
		 //CheckDriveStatusRegister(DRIVE1);
		 if (pb1_value) {
		 if (motor_direction_pitch != MOTOR_DIRECTION_LEFT) {
		 motor_direction_pitch = MOTOR_DIRECTION_LEFT;
		 DirectionMotor(DRIVE1, MOTOR_DIRECTION_LEFT);
		 }
		 ResetDriveStatusRegister(DRIVE1);
		 //if (drive_regs[DRIVE1].ctrl_reg.enbl != 1) {
		 EnableMotor(DRIVE1);
		 //}

		 motor_pitch_on = 1;
		 } else if (pb2_value) {
		 if (motor_direction_pitch != MOTOR_DIRECTION_RIGHT) {
		 motor_direction_pitch = MOTOR_DIRECTION_RIGHT;
		 DirectionMotor(DRIVE1, MOTOR_DIRECTION_RIGHT);
		 }
		 ResetDriveStatusRegister(DRIVE1);
		 //if (drive_regs[DRIVE1].ctrl_reg.enbl != 1) {
		 EnableMotor(DRIVE1);
		 //}

		 motor_pitch_on = 1;
		 }
		 /*
		 else if (flag_can_direction || flag_can_speed) {
		 if (flag_can_direction) {
		 flag_can_direction = 0;
		 if (direction_stepper_motor_pitch != MOTOR_DIRECTION_STOP) {

		 if (direction_stepper_motor_pitch == MOTOR_DIRECTION_LEFT) {
		 if (motor_direction_pitch != MOTOR_DIRECTION_LEFT) {
		 motor_direction_pitch = MOTOR_DIRECTION_LEFT;
		 DirectionMotor(DRIVE1, MOTOR_DIRECTION_LEFT);
		 }
		 EnableMotor(DRIVE1);
		 motor_pitch_on = 1;
		 } else if (direction_stepper_motor_pitch == MOTOR_DIRECTION_RIGHT) {
		 if (motor_direction_pitch != MOTOR_DIRECTION_RIGHT) {
		 motor_direction_pitch = MOTOR_DIRECTION_RIGHT;
		 DirectionMotor(DRIVE1, MOTOR_DIRECTION_RIGHT);
		 }
		 EnableMotor(DRIVE1);
		 motor_pitch_on = 1;
		 }
		 }
		 }
		 if (flag_can_speed) {
		 flag_can_speed = 0;
		 if (speed_stepper_motor_pitch > 0) {
		 ResetDriveStatusRegister(DRIVE1);
		 EnableMotor(DRIVE1);
		 motor_pitch_on = 1;
		 } else {
		 motor_pitch_on = 0;
		 InitAndConfigDrive(DRIVE1);
		 }
		 }
		 }
		 */
		/*
		 else {
		 motor_pitch_on = 0;
		 //HAL_Delay(25);

		 //HAL_GPIO_WritePin(LED_CANB_GPIO_Port, LED_CANB_Pin, GPIO_PIN_SET);
		 //flag_drive_fault = 0;

		 //if (drive_regs[DRIVE1].ctrl_reg.enbl == 1) {
		 InitAndConfigDrive(DRIVE1);
		 //DisableMotor(DRIVE1);
		 //}

		 //HAL_Delay(100);
		 }
		 }
		 */
		/*
		 // Check for ROPS or emergency stop flags
		 if (b_rops) {
		 current_state = STATE_ROPS;
		 }
		 if (b_emergency_stop) {
		 current_state = STATE_EMERGENCY_STOP;
		 }*/
		/*
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
		 };*/
	}
}
