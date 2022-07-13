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

#define DRIVE_MAST  DRIVE1
#define DRIVE_PITCH DRIVE2

// DRV8711 Register map addresses
typedef enum
{
	DRV8711_CTRL_REG,		// 0x00
	DRV8711_TORQUE_REG,		// 0x01
	DRV8711_OFF_REG,		// 0x02
	DRV8711_BLANK_REG,		// 0x03
	DRV8711_DECAY_REG,		// 0x04
	DRV8711_STALL_REG,		// 0x05
	DRV8711_DRIVE_REG,		// 0x06
	DRV8711_STATUS_REG,		// 0x07

	NUM_DRIVE_REGS
} DRIVE_REG_ADDR;


// Values of the DRV8711 registers
typedef struct
{
	uint32_t enbl : 1;
	uint32_t rdir : 1;
	uint32_t rstep : 1;
	uint32_t mode : 4;
	uint32_t extstall : 1;
	uint32_t isgain : 2;
	uint32_t dtime : 2;
} CTRL_REG;

typedef struct
{
	uint32_t torque : 8;
	uint32_t smplth : 3;
	uint32_t reserved : 1;
} TORQUE_REG;

typedef struct
{
	uint32_t toff : 8;
	uint32_t pwmmode : 1;
	uint32_t reserved : 3;
} OFF_REG;

typedef struct
{
	uint32_t tblank : 8;
	uint32_t abt : 1;
	uint32_t reserved : 3;
} BLANK_REG;

typedef struct
{
	uint32_t tdecay : 8;
	uint32_t decmod : 3;
	uint32_t reserved : 1;
} DECAY_REG;

typedef struct
{
	uint32_t sdthr : 8;
	uint32_t sdcnt : 2;
	uint32_t vdiv : 2;
} STALL_REG;

typedef struct
{
	uint32_t ocpth : 2;
	uint32_t ocpdeg : 2;
	uint32_t tdriven : 2;
	uint32_t tdrivep : 2;
	uint32_t idriven : 2;
	uint32_t idrivep : 2;
} DRIVE_REG;

typedef struct
{
	uint32_t ots : 1;
	uint32_t aocp : 1;
	uint32_t bocp : 1;
	uint32_t apdf : 1;
	uint32_t bpdf : 1;
	uint32_t uvlo : 1;
	uint32_t std : 1;
	uint32_t stdlat : 1;
} STATUS_REG;

typedef struct
{
	CTRL_REG ctrl_reg;
	TORQUE_REG torque_reg;
	OFF_REG off_reg;
	BLANK_REG blank_reg;
	DECAY_REG decay_reg;
	STALL_REG stall_reg;
	DRIVE_REG drive_reg;
	STATUS_REG status_reg;
} DRV8711_REGS;

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


extern SPI_HandleTypeDef* hspi;
extern TIM_HandleTypeDef* pwm1_timer;
extern TIM_HandleTypeDef* pwm2_timer;
extern uint32_t pwm1_channel;
extern uint32_t pwm2_channel;

void InitDrives(SPI_HandleTypeDef* hspi, TIM_HandleTypeDef* htim_pwm1, uint32_t channel_pwm1,
										 TIM_HandleTypeDef* htim_pwm2, uint32_t channel_pwm2);


void EnableDrive(DRIVE_MOTOR drive_index);
void DisableDrive(DRIVE_MOTOR drive_index);

void ResetDrive(DRIVE_MOTOR drive_index);
void ResetStatusRegisters(DRIVE_MOTOR drive_index);

void SendConfigRegisters(DRIVE_MOTOR drive_index);

void DriveSleep(DRIVE_MOTOR drive_index);
void DriveWakeUp(DRIVE_MOTOR drive_index);
uint32_t IsDriveAwake(DRIVE_MOTOR drive_index);

void SetDirection(DRIVE_MOTOR drive_index, uint32_t direction);

void EnableDriveIndexer(DRIVE_MOTOR drive_index);
void EnableDriveExternalPWM(DRIVE_MOTOR drive_index);

void Step(DRIVE_MOTOR drive_index);

void SetDutyCycle(uint32_t pwm_index, uint16_t duty_cycle);
void DriveMastRight();
void DriveMastLeft();
void DriveMastStop();

void DEBUG_SPI(DRIVE_MOTOR drive_index);



#endif /* INC_MOTOR_H_ */
