/*
 * motor.c
 *
 *  Created on: 25 mai 2022
 *      Author: Marc
 */

#include "motor.h"
#include "main.h"

#define PWM_PERIOD 500


enum DRIVE_PINS
{
	DRIVE_RESET,
	DRIVE_SLEEP,
	DRIVE_CS,
	DRIVE_STEP,
	DRIVE_DIR,

	NUM_PINS
};

GPIO_TypeDef* drive_ports[DRIVE_MOTOR_NUM][NUM_PINS] =
{
	{
		RESET1_GPIO_Port,
		nSLEEP1_GPIO_Port,
		SPI_CS1_GPIO_Port,
		STEP1_GPIO_Port,
		DIR1_GPIO_Port
	},
	{
		RESET2_GPIO_Port,
		nSLEEP2_GPIO_Port,
		SPI_CS2_GPIO_Port,
		STEP2_GPIO_Port,
		DIR2_GPIO_Port
	}
};
uint16_t drive_pins[DRIVE_MOTOR_NUM][NUM_PINS] =
{
	{
		RESET1_Pin,
		nSLEEP1_Pin,
		SPI_CS1_Pin,
		STEP1_Pin,
		DIR1_Pin
	},
	{
		RESET2_Pin,
		nSLEEP2_Pin,
		SPI_CS2_Pin,
		STEP2_Pin,
		DIR2_Pin
	}
};

SPI_HandleTypeDef* hspi;

TIM_HandleTypeDef* pwm1_timer;
TIM_HandleTypeDef* pwm2_timer;
uint32_t pwm1_channel;
uint32_t pwm2_channel;

DRV8711_REGS drive_regs[DRIVE_MOTOR_NUM];

uint32_t drive_status[DRIVE_MOTOR_NUM] = { DRIVE_AWAKE, DRIVE_AWAKE };

// Helper SPI functions
void TransmitMotorSPI(DRIVE_MOTOR drive_index, uint8_t reg);
uint16_t ReceiveMotorSPI(DRIVE_MOTOR drive_index, uint8_t reg);

void SendDriveRegisters(DRIVE_MOTOR drive_index);

void InitRegValues(DRIVE_MOTOR drive_index);
void InitRegValues2(DRIVE_MOTOR drive_index);
void InitDriveMotor(DRIVE_MOTOR drive_index);

void InitRegValuesPWM(DRIVE_MOTOR drive_index);

// PWM
TIM_HandleTypeDef* pwm_timers[2] = { 0, 0 };
uint32_t pwm_channels[2] = { 0, 0 };
enum PWMs
{
	PWM1 = 0,
	PWM2
};

void InitDrives(SPI_HandleTypeDef* hspi_, TIM_HandleTypeDef* htim_pwm1, uint32_t channel_pwm1,
		 	 	 	 	 	 	 	 	  TIM_HandleTypeDef* htim_pwm2, uint32_t channel_pwm2)
{
	hspi = hspi_;
	pwm1_timer = htim_pwm1;
	pwm2_timer = htim_pwm2;
	pwm1_channel = channel_pwm1;
	pwm2_channel = channel_pwm2;

	pwm_timers[0] = pwm1_timer;
	pwm_timers[1] = pwm2_timer;
	pwm_channels[0] = pwm1_channel;
	pwm_channels[1] = pwm2_channel;

	HAL_GPIO_WritePin(drive_ports[DRIVE_PITCH][DRIVE_CS],
							  drive_pins[DRIVE_PITCH][DRIVE_CS], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(drive_ports[DRIVE_MAST][DRIVE_CS],
							  drive_pins[DRIVE_MAST][DRIVE_CS], GPIO_PIN_RESET);

	InitDriveMotor(DRIVE_MAST);
	HAL_Delay(5);
	InitDriveMotor(DRIVE_PITCH);
	HAL_Delay(5);

	//InitDriveMotor(DRIVE_PITCH);
	//HAL_Delay(10);

	//InitDriveMotor(DRIVE_PITCH);
	//InitDriveMotor(DRIVE_MAST);
	/*
	// Reset drive
	HAL_GPIO_WritePin(drive_ports[DRIVE_MAST][DRIVE_RESET],
			drive_pins[DRIVE_MAST][DRIVE_RESET], GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(drive_ports[DRIVE_MAST][DRIVE_RESET],
			drive_pins[DRIVE_MAST][DRIVE_RESET], GPIO_PIN_RESET);

	// Disable sleeping
	HAL_GPIO_WritePin(drive_ports[DRIVE_MAST][DRIVE_SLEEP],
			drive_pins[DRIVE_MAST][DRIVE_SLEEP], GPIO_PIN_SET);

	// Set initial reg values
	InitRegValues(DRIVE_MAST);
	*/


}


void TransmitMotorSPI(DRIVE_MOTOR drive_index, uint8_t reg)
{
	if (reg >= 7)
		return;

	//HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
	//		  		    drive_pins[drive_index][DRIVE_CS], GPIO_PIN_RESET);

	for (int i = 0; i < 1000; ++i) {}
	// uint16_t data = *(uint16_t*)(&drive_regs[drive_index]) + reg;
	uint16_t data = *((uint16_t*)(&drive_regs[drive_index])) | (reg << 12);

	uint8_t tx_data[2] = {0};
	tx_data[0] = ((reg & 0x07) << 4) | ((data & 0x0F00) >> 8);
	tx_data[1] = (data & 0xFF);
	uint8_t ret = HAL_SPI_Transmit(hspi, tx_data, 2, HAL_MAX_DELAY);
	if (ret != HAL_OK)
	{
		HAL_GPIO_WritePin(LED_CANB_GPIO_Port, LED_CANB_Pin, GPIO_PIN_SET);
	}
	for (int i = 0; i < 1000; ++i) {}

	//HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
	//				  drive_pins[drive_index][DRIVE_CS], GPIO_PIN_SET);
}

void DEBUG_SPI(DRIVE_MOTOR drive_index)
{
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
					  drive_pins[drive_index][DRIVE_CS], GPIO_PIN_SET);

	//TransmitMotorSPI(drive_index, 0);

	uint8_t tx_data = 0x70 | 0x80;
	HAL_SPI_Transmit(hspi, &tx_data, 1, HAL_MAX_DELAY);
	uint8_t rx_data;
	HAL_SPI_Receive(hspi, &rx_data, 1, HAL_MAX_DELAY);


	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
					  drive_pins[drive_index][DRIVE_CS], GPIO_PIN_RESET);
}

uint16_t ReceiveMotorSPI(DRIVE_MOTOR drive_index, uint8_t reg)
{
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
					  drive_pins[drive_index][DRIVE_CS], GPIO_PIN_SET);

	uint8_t tx_data = reg | 0x80;
	HAL_SPI_Transmit(hspi, &tx_data, 1, HAL_MAX_DELAY);

	uint8_t rx_data[2];
	HAL_SPI_Receive(hspi, rx_data, 2, HAL_MAX_DELAY);
	// (uint16_t)(rx_data[0] & 0xFF) | (rx_data[1] >> 8)
	// TODO: (Marc) Comment recevoir 12bits SPI

	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
					  drive_pins[drive_index][DRIVE_CS], GPIO_PIN_RESET);

	return 0;
}

void SendDriveRegisters(DRIVE_MOTOR drive_index)
{
	// Send every register to the drive
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
					  drive_pins[drive_index][DRIVE_CS], GPIO_PIN_SET);

	// We send every register except for the status register (up to 7 register)
	for (uint8_t i = 0; i < (NUM_DRIVE_REGS - 1); ++i)
	{
		// uint16_t data = *((uint16_t*)(&drive_regs[drive_index]) + i);
		TransmitMotorSPI(drive_index, i);
		HAL_Delay(1);
	}

	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
					  drive_pins[drive_index][DRIVE_CS], GPIO_PIN_RESET);
}


void InitRegValues(DRIVE_MOTOR drive_index)
{
	// CTRL register
	drive_regs[drive_index].ctrl_reg.enbl = 1;	// Enable motor
	drive_regs[drive_index].ctrl_reg.rdir = 0;	// Direction set by DIR pin
	drive_regs[drive_index].ctrl_reg.rstep = 0;	// No automatic stepping
	drive_regs[drive_index].ctrl_reg.mode = 0b0010;	// 1/4 step
	//drive_regs[drive_index].ctrl_reg.mode = 0b0001;	// 1/2 step
	//drive_regs[drive_index].ctrl_reg.mode = 0b0000;	// Full step
	drive_regs[drive_index].ctrl_reg.extstall = 0;	// Internal stall detect
	drive_regs[drive_index].ctrl_reg.isgain = 0b00;	// Gain of 5
	drive_regs[drive_index].ctrl_reg.dtime = 0b11;	// Dead-time of 850ns

	// TORQUE register
	drive_regs[drive_index].torque_reg.torque = 100;	// Sets full torque for H-bridge
	drive_regs[drive_index].torque_reg.smplth = 0b001;	// 100us BEMF sample threshold

	// OFF register
	drive_regs[drive_index].off_reg.toff = 0x30;		// Sets fixed off time, in increments of 500ns
	drive_regs[drive_index].off_reg.pwmmode = 0;		// Use internal indexer

	// BLANK register
	drive_regs[drive_index].blank_reg.tblank = 0x80;	// Current trip blanking time, in increments of 20ns
	drive_regs[drive_index].blank_reg.abt = 0;		// Disable adaptive blanking time

	// DECAY register
	drive_regs[drive_index].decay_reg.tdecay = 0x10;		// Mixed decay transition time, in increments of 500ns
	drive_regs[drive_index].decay_reg.decmod = 0b001;	// Slow decay for increasing current

	// STALL register
	drive_regs[drive_index].stall_reg.sdthr = 0x40;	// Stall detect threshold
	drive_regs[drive_index].stall_reg.sdcnt = 0b00;	// STALLn asserted on first step with BEMF below SDTHR
	drive_regs[drive_index].stall_reg.vdiv = 0b00;	// BEMF divided by 32

	// DRIVE register
	drive_regs[drive_index].drive_reg.ocpth = 0b01;		// OCP threshold of 500mV
	drive_regs[drive_index].drive_reg.ocpdeg = 0b10;		// OCP deglitch time of 4us
	drive_regs[drive_index].drive_reg.tdriven = 0b01;	// Low-side gate drive time of 500ns
	drive_regs[drive_index].drive_reg.tdrivep = 0b01;	// High-side gate drive time of 500ns
	drive_regs[drive_index].drive_reg.idriven = 0b10;	// Low-side gate drive peak current of 300mA peak (sink)
	drive_regs[drive_index].drive_reg.idrivep = 0b10;	// High-side gate drive peak current of 150mA peak (sink)
}

void InitRegValuesPWM(DRIVE_MOTOR drive_index)
{
	// CTRL register
	drive_regs[drive_index].ctrl_reg.enbl = 1;	// Enable motor
	drive_regs[drive_index].ctrl_reg.rdir = 0;	// Direction set by DIR pin
	drive_regs[drive_index].ctrl_reg.rstep = 0;	// No automatic stepping
	//drive_regs[drive_index].ctrl_reg.mode = 0b0010;	// 1/4 step
	drive_regs[drive_index].ctrl_reg.mode = 0b0000;	// Full step
	drive_regs[drive_index].ctrl_reg.extstall = 0;	// Internal stall detect
	// drive_regs[drive_index].ctrl_reg.isgain = 0b00;	// Gain of 5
	drive_regs[drive_index].ctrl_reg.isgain = 0b01;	// Gain of 10
	drive_regs[drive_index].ctrl_reg.dtime = 0b00;	// Dead-time of 850ns

	// TORQUE register
	drive_regs[drive_index].torque_reg.torque = 24;// max 2.5A  //0xFF;	// Sets full torque for H-bridge
	drive_regs[drive_index].torque_reg.smplth = 0b111;	// 100us BEMF sample threshold

	// OFF register
	drive_regs[drive_index].off_reg.toff = 0x80;		// Sets fixed off time, in increments of 500ns
	// drive_regs[drive_index].off_reg.pwmmode = pwm_mode;	// Use internal indexer
	drive_regs[drive_index].off_reg.pwmmode = 1;

	// BLANK register
	drive_regs[drive_index].blank_reg.tblank = 0x80;	// Current trip blanking time, in increments of 20ns
	drive_regs[drive_index].blank_reg.abt = 0;		// Disable adaptive blanking time

	// DECAY register
	drive_regs[drive_index].decay_reg.tdecay = 0x80;		// Mixed decay transition time, in increments of 500ns
	drive_regs[drive_index].decay_reg.decmod = 0b000;	// Force slow decay at all times


	// STALL register
	drive_regs[drive_index].stall_reg.sdthr = 0xFF;	// Stall detect threshold
	drive_regs[drive_index].stall_reg.sdcnt = 0b11;	// STALLn asserted on first step with BEMF below SDTHR
	drive_regs[drive_index].stall_reg.vdiv = 0b00;	// BEMF divided by 32

	// DRIVE register
	drive_regs[drive_index].drive_reg.ocpth = 0b11;		// OCP threshold of 500mV
	drive_regs[drive_index].drive_reg.ocpdeg = 0b11;		// OCP deglitch time of 4us
	drive_regs[drive_index].drive_reg.tdriven = 0b11;	// Low-side gate drive time of 500ns
	drive_regs[drive_index].drive_reg.tdrivep = 0b11;	// High-side gate drive time of 500ns
	drive_regs[drive_index].drive_reg.idriven = 0b11;	// Low-side gate drive peak current of 300mA peak (sink)
	drive_regs[drive_index].drive_reg.idrivep = 0b11;	// High-side gate drive peak current of 150mA peak (sink)
}

void InitRegValues2(DRIVE_MOTOR drive_index)
{
	// CTRL register
	drive_regs[drive_index].ctrl_reg.enbl = 1;	// Enable motor
	drive_regs[drive_index].ctrl_reg.rdir = 0;	// Direction set by DIR pin
	drive_regs[drive_index].ctrl_reg.rstep = 0;	// No automatic stepping
	//drive_regs[drive_index].ctrl_reg.mode = 0b0010;	// 1/4 step
	if (drive_index == DRIVE_PITCH)
		drive_regs[drive_index].ctrl_reg.mode = 0b0001;	// 1/2 step
	else
		drive_regs[drive_index].ctrl_reg.mode = 0b0000;	// Full step
	drive_regs[drive_index].ctrl_reg.extstall = 0;	// Internal stall detect
	// drive_regs[drive_index].ctrl_reg.isgain = 0b00;	// Gain of 5
	drive_regs[drive_index].ctrl_reg.isgain = 0b01;	// Gain of 10
	drive_regs[drive_index].ctrl_reg.dtime = 0b00;	// Dead-time of 850ns

	// TORQUE register
	drive_regs[drive_index].torque_reg.torque = 24;// max 2.5A  //0xFF;	// Sets full torque for H-bridge
	drive_regs[drive_index].torque_reg.smplth = 0b111;	// 100us BEMF sample threshold

	// OFF register
	drive_regs[drive_index].off_reg.toff = 0x80;		// Sets fixed off time, in increments of 500ns
	// drive_regs[drive_index].off_reg.pwmmode = pwm_mode;	// Use internal indexer
	drive_regs[drive_index].off_reg.pwmmode = 1;

	// BLANK register
	drive_regs[drive_index].blank_reg.tblank = 0x80;	// Current trip blanking time, in increments of 20ns
	drive_regs[drive_index].blank_reg.abt = 0;		// Disable adaptive blanking time

	// DECAY register
	drive_regs[drive_index].decay_reg.tdecay = 0x80;		// Mixed decay transition time, in increments of 500ns
	if (drive_index == DRIVE_PITCH)
		drive_regs[drive_index].decay_reg.decmod = 0b010;	// Force fast decay at all times
	else
		drive_regs[drive_index].decay_reg.decmod = 0b000;	// Force slow decay at all times


	// STALL register
	drive_regs[drive_index].stall_reg.sdthr = 0xFF;	// Stall detect threshold
	drive_regs[drive_index].stall_reg.sdcnt = 0b11;	// STALLn asserted on first step with BEMF below SDTHR
	drive_regs[drive_index].stall_reg.vdiv = 0b00;	// BEMF divided by 32

	// DRIVE register
	drive_regs[drive_index].drive_reg.ocpth = 0b11;		// OCP threshold of 500mV
	drive_regs[drive_index].drive_reg.ocpdeg = 0b11;		// OCP deglitch time of 4us
	drive_regs[drive_index].drive_reg.tdriven = 0b11;	// Low-side gate drive time of 500ns
	drive_regs[drive_index].drive_reg.tdrivep = 0b11;	// High-side gate drive time of 500ns
	drive_regs[drive_index].drive_reg.idriven = 0b11;	// Low-side gate drive peak current of 300mA peak (sink)
	drive_regs[drive_index].drive_reg.idrivep = 0b11;	// High-side gate drive peak current of 150mA peak (sink)
}

void InitDriveMotor(DRIVE_MOTOR drive_index)
{
	// Reset drive
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_RESET],
			drive_pins[drive_index][DRIVE_RESET], GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_RESET],
			drive_pins[drive_index][DRIVE_RESET], GPIO_PIN_RESET);

	// Disable sleeping
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_SLEEP],
			drive_pins[drive_index][DRIVE_SLEEP], GPIO_PIN_SET);

	// Set initial reg values
	//uint8_t pwm_mode = 0; // Internal indexer enabled
	//if (drive_index == DRIVE_MAST)
	//	pwm_mode = 1; // External pwm enabled
	// InitRegValues2(drive_index, pwm_mode);
	if (drive_index == DRIVE_MAST)
		InitRegValuesPWM(drive_index);
	else
		InitRegValues2(drive_index);

	// Send regs over SPI
	SendDriveRegisters(drive_index);
}


void EnableDrive(DRIVE_MOTOR drive_index)
{
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
						  drive_pins[drive_index][DRIVE_CS], GPIO_PIN_SET);

	drive_regs[drive_index].ctrl_reg.enbl = 1;
	TransmitMotorSPI(drive_index, DRV8711_CTRL_REG);

	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
						  drive_pins[drive_index][DRIVE_CS], GPIO_PIN_RESET);
}

void DisableDrive(DRIVE_MOTOR drive_index)
{
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
						  drive_pins[drive_index][DRIVE_CS], GPIO_PIN_SET);

	drive_regs[drive_index].ctrl_reg.enbl = 0;
	TransmitMotorSPI(drive_index, DRV8711_CTRL_REG);

	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
						  drive_pins[drive_index][DRIVE_CS], GPIO_PIN_RESET);
}

void ResetDrive(DRIVE_MOTOR drive_index)
{
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_RESET],
					  drive_pins[drive_index][DRIVE_RESET], GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_RESET],
					  drive_pins[drive_index][DRIVE_RESET], GPIO_PIN_RESET);
}

void DriveSleep(DRIVE_MOTOR drive_index)
{
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_SLEEP],
					  drive_pins[drive_index][DRIVE_SLEEP], GPIO_PIN_RESET);
	drive_status[drive_index] = DRIVE_ASLEEP;
}

void DriveWakeUp(DRIVE_MOTOR drive_index)
{
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_SLEEP],
					  drive_pins[drive_index][DRIVE_SLEEP], GPIO_PIN_SET);
	drive_status[drive_index] = DRIVE_AWAKE;
}

uint32_t IsDriveAwake(DRIVE_MOTOR drive_index)
{
	return drive_status[drive_index];
}

void SetDirection(DRIVE_MOTOR drive_index, uint32_t direction)
{
	GPIO_PinState state = (direction == DIR_FORWARD) ? 1 : 0;
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_DIR],
					  drive_pins[drive_index][DRIVE_DIR], state);
}

void EnableDriveIndexer(DRIVE_MOTOR drive_index)
{
	drive_regs[drive_index].off_reg.pwmmode = 0;		// Use internal indexer

	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
					  drive_pins[drive_index][DRIVE_CS], GPIO_PIN_SET);
	TransmitMotorSPI(drive_index, DRV8711_OFF_REG);
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
					  drive_pins[drive_index][DRIVE_CS], GPIO_PIN_RESET);
}

void EnableDriveExternalPWM(DRIVE_MOTOR drive_index)
{
	drive_regs[drive_index].off_reg.pwmmode = 1;		// Use external PWM

	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
					  drive_pins[drive_index][DRIVE_CS], GPIO_PIN_SET);
	TransmitMotorSPI(drive_index, DRV8711_OFF_REG);
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
					  drive_pins[drive_index][DRIVE_CS], GPIO_PIN_RESET);
}

void Step(DRIVE_MOTOR drive_index)
{
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_STEP],
					  drive_pins[drive_index][DRIVE_STEP], GPIO_PIN_SET);
	//HAL_Delay(5);
	for (int i = 0; i < 45; ++i) {}
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_STEP],
					  drive_pins[drive_index][DRIVE_STEP], GPIO_PIN_RESET);
	//HAL_Delay(5);
	for (int i = 0; i < 45; ++i) {}
}


void SetDutyCycle(uint32_t pwm_index, uint16_t duty_cycle)
{
	HAL_TIM_PWM_Stop(pwm_timers[pwm_index], pwm_channels[pwm_index]); // stop generation of pwm

	TIM_OC_InitTypeDef sConfigOC;
	// (*htim).Init.Period = PWM_PERIOD; // set the period duration
	// HAL_TIM_PWM_Init(htim); // reinititialise with new period value

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	//sConfigOC.Pulse = (uint32_t)(duty_cycle * (float)PWM_PERIOD); // set the pulse duration
	sConfigOC.Pulse = duty_cycle;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(pwm_timers[pwm_index], &sConfigOC, pwm_channels[pwm_index]);

	// HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2); // start pwm generation
}

void DriveMastRight()
{
	SetDutyCycle(PWM1, 360);
	SetDutyCycle(PWM2, 480);
	HAL_TIM_PWM_Start(pwm1_timer, pwm1_channel);
	HAL_TIM_PWM_Start(pwm2_timer, pwm2_channel);
}

void DriveMastLeft()
{
	SetDutyCycle(PWM1, 480);
	SetDutyCycle(PWM2, 360);
	HAL_TIM_PWM_Start(pwm1_timer, pwm1_channel);
	HAL_TIM_PWM_Start(pwm2_timer, pwm2_channel);
}

void DriveMastStop()
{
	SetDutyCycle(PWM1, 0);
	SetDutyCycle(PWM2, 0);
	HAL_TIM_PWM_Stop(pwm1_timer, pwm1_channel);
	HAL_TIM_PWM_Stop(pwm2_timer, pwm2_channel);
}



