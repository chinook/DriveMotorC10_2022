/*
 * motor_lib.h
 *
 *  Created on: Aug 13, 2025
 *      Author: thoma
 */

#ifndef INC_DRV8711_LIB_H_
#define INC_DRV8711_LIB_H_

#include "main.h"

typedef enum DRIVE_MOTOR {
	DRIVE1,
	DRIVE2,

	DRIVE_MOTOR_NUM
} DRIVE_MOTOR;

typedef enum DRIVE_PINS {
	DRIVE_RESET, DRIVE_SLEEP, DRIVE_CS, DRIVE_STEP, DRIVE_DIR,

	NUM_PINS
} DRIVE_PINS;

// Values of the DRV8711 registers
typedef union CTRL_REG {
	uint16_t reg;
	struct {
		uint16_t enbl :1;
		uint16_t rdir :1;
		uint16_t rstep :1;
		uint16_t mode :4;
		uint16_t extstall :1;
		uint16_t isgain :2;
		uint16_t dtime :2;
		uint16_t reserved :4;
	};
} CTRL_REG;

typedef union TORQUE_REG {
	uint16_t reg;
	struct {
		uint16_t torque :8;
		uint16_t smplth :3;
		uint16_t reserved :5;
	};
} TORQUE_REG;

typedef union OFF_REG {
	uint16_t reg;
	struct {
		uint16_t toff :8;
		uint16_t pwmmode :1;
		uint16_t reserved :7;
	};
} OFF_REG;

typedef union BLANK_REG {
	uint16_t reg;
	struct {
		uint16_t tblank :8;
		uint16_t abt :1;
		uint16_t reserved :7;
	};
} BLANK_REG;

typedef union DECAY_REG {
	uint16_t reg;
	struct {
		uint16_t tdecay :8;
		uint16_t decmod :3;
		uint16_t reserved :5;
	};
} DECAY_REG;

typedef union STALL_REG {
	uint16_t reg;
	struct {
		uint16_t sdthr :8;
		uint16_t sdcnt :2;
		uint16_t vdiv :2;
		uint16_t reserved :4;
	};
} STALL_REG;

typedef union DRIVE_REG {
	uint16_t reg;
	struct {
		uint16_t ocpth :2;
		uint16_t ocpdeg :2;
		uint16_t tdriven :2;
		uint16_t tdrivep :2;
		uint16_t idriven :2;
		uint16_t idrivep :2;
		uint16_t reserved :4;
	};
} DRIVE_REG;

typedef union STATUS_REG {
	uint16_t reg;
	struct {
		uint16_t ots :1;
		uint16_t aocp :1;
		uint16_t bocp :1;
		uint16_t apdf :1;
		uint16_t bpdf :1;
		uint16_t uvlo :1;
		uint16_t std :1;
		uint16_t stdlat :1;
		uint16_t reserved :8;
	};
} STATUS_REG;

typedef struct DRV8711_REGS {
	CTRL_REG ctrl_reg;
	TORQUE_REG torque_reg;
	OFF_REG off_reg;
	BLANK_REG blank_reg;
	DECAY_REG decay_reg;
	STALL_REG stall_reg;
	DRIVE_REG drive_reg;
	STATUS_REG status_reg;
} DRV8711_REGS;

// DRV8711 Register map addresses
typedef enum DRIVE_REG_ADDR {
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

extern const GPIO_TypeDef *drive_ports[DRIVE_MOTOR_NUM][NUM_PINS];
extern const uint16_t drive_pins[DRIVE_MOTOR_NUM][NUM_PINS];

extern DRV8711_REGS drive_regs[DRIVE_MOTOR_NUM];

void InitRegValuesStepper(DRIVE_MOTOR drive_index);
uint16_t ReadRegConfig(DRIVE_MOTOR drive_index, uint8_t reg);

uint16_t ReadSPI(DRIVE_MOTOR drive_index, uint8_t reg);
uint32_t WriteSPI(DRIVE_MOTOR drive_index, uint8_t reg, uint16_t reg_config);

void SendDriveRegisters(DRIVE_MOTOR drive_index);
void ReadAndVerifyDriveRegisters(DRIVE_MOTOR drive_index);
uint8_t CheckDriveStatusRegister(DRIVE_MOTOR drive_index);
void ResetDriveStatusRegister(DRIVE_MOTOR drive_index);

#endif /* INC_DRV8711_LIB_H_ */
