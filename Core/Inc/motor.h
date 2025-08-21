/*
 * motor.h
 *
 *  Created on: 25 mai 2022
 *      Author: Marc
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_


#include "stm32f4xx_hal.h"

#include "drv8711_lib.h"

typedef enum
{
	DIR_STOP = 0,
	DIR_LEFT,
	DIR_RIGHT,

	DIR_INVALID
} MOTOR_DIRECTION;

extern uint8_t flag_drive_fault;
extern uint8_t motor_pitch_on;

void InitAndConfigDrive(DRIVE_MOTOR drive_index);
void DisableDrive(DRIVE_MOTOR drive_index);

void EnableMotor(DRIVE_MOTOR drive_index);
void DisableMotor(DRIVE_MOTOR drive_index);

void DirectionMotor(DRIVE_MOTOR drive_index, uint8_t dir);

//auto run with interrupt at 1us
void StepFunction();

#endif /* INC_MOTOR_H_ */
