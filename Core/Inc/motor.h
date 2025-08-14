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

extern uint8_t flag_drive_fault;
extern uint8_t motor_pitch_on;

void InitDrives();

void EnableDrive(DRIVE_MOTOR drive_index);
void DisableDrive(DRIVE_MOTOR drive_index);

void DirectionDrive(DRIVE_MOTOR drive_index, uint8_t dir);
void StepFunction();

#endif /* INC_MOTOR_H_ */
