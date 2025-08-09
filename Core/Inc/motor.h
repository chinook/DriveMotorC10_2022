/*
 * motor.h
 *
 *  Created on: 25 mai 2022
 *      Author: Marc
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_


#include "stm32f4xx_hal.h"

typedef enum
{
	DRIVE1,
	DRIVE2,

	DRIVE_MOTOR_NUM
} DRIVE_MOTOR;

typedef enum
{
	DIR_STOP = 0,
	DIR_LEFT,
	DIR_RIGHT,

	DIR_INVALID
} MOTOR_DIRECTION;

enum DRIVE_STATUS
{
	DRIVE_ASLEEP = 0,
	DRIVE_AWAKE
};


//extern SPI_HandleTypeDef* hspi;
//extern TIM_HandleTypeDef* pwm1_timer;
//extern TIM_HandleTypeDef* pwm2_timer;
//extern uint32_t pwm1_channel;
//extern uint32_t pwm2_channel;

//extern uint32_t speed_stepper_motor_pitch_int_converted;
//extern uint8_t gpio_pin_value;

extern uint8_t flag_drive_fault;

void InitDrives();

void EnableDrive(DRIVE_MOTOR drive_index);
void DisableDrive(DRIVE_MOTOR drive_index);

void ResetDrive(DRIVE_MOTOR drive_index);

void SendConfigRegisters(DRIVE_MOTOR drive_index);

void DriveSleep(DRIVE_MOTOR drive_index);
void DriveWakeUp(DRIVE_MOTOR drive_index);
uint32_t IsDriveAwake(DRIVE_MOTOR drive_index);

void SetDirection(DRIVE_MOTOR drive_index, uint32_t direction);

void EnableDriveIndexer(DRIVE_MOTOR drive_index);
void EnableDriveExternalPWM(DRIVE_MOTOR drive_index);

void better_step_function();

void SetDutyCycle(uint32_t pwm_index, uint16_t duty_cycle);
void DriveMastRight();
void DriveMastLeft();
void DriveMastStop();

void DEBUG_SPI(DRIVE_MOTOR drive_index);
void DEBUG_SPI_CHATGPT(DRIVE_MOTOR drive_index);



#endif /* INC_MOTOR_H_ */
