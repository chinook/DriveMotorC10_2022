/*
 * motor.c
 *
 *  Created on: 25 mai 2022
 *      Author: Marc
 *  Edited by : Thomas Maitre after June 2024
 */
#include "motor.h"

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "drv8711_lib.h"
#include "chinook_can_ids.h"

uint8_t motor_pitch_on = 0;

void InitAndConfigDrive(DRIVE_MOTOR drive_index) {
	// Reset drive
	DisableDrive(drive_index);

	// init register for a stepper motor
	InitRegValuesStepper(drive_index);

	//seulement pour debug SPI : ne pas utiliser pour tester un moteur
	//InitRegValuesStepperDefault(drive_index); //WARNING : TORQUE IS SET TO MAXIMUM

	// Send regs over SPI
	SendDriveRegisters(drive_index);

	// Verify if chip setup is good
	ReadAndVerifyDriveRegisters(drive_index);

	CheckDriveStatusRegister(drive_index);

	ResetDriveStatusRegister(drive_index);
}

void DisableDrive(DRIVE_MOTOR drive_index) {
	DisableMotor(drive_index);

	if (drive_regs[drive_index].ctrl_reg.enbl == 0) {
		HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_RESET],
				drive_pins[drive_index][DRIVE_RESET], GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_RESET],
				drive_pins[drive_index][DRIVE_RESET], GPIO_PIN_RESET);
		// Disable sleeping
		HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_SLEEP],
				drive_pins[drive_index][DRIVE_SLEEP], GPIO_PIN_SET);
		// CS à LOW
		HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
				drive_pins[drive_index][DRIVE_CS], GPIO_PIN_RESET);

		HAL_Delay(5); //wait for everything to setup : DRV8711 recommend 1ms
	}

}

void EnableMotor(DRIVE_MOTOR drive_index) {
	/*
	 if (CheckDriveStatusRegister(drive_index) != 0) {
	 return;
	 }
	 */
	uint16_t reg_config = ReadRegConfig(drive_index, 0);
	reg_config = reg_config | 0x0001;

	uint32_t err = WriteSPI(drive_index, DRV8711_CTRL_REG, reg_config);

	if (err == 0) {
		drive_regs[drive_index].ctrl_reg.enbl = 1;
	}

	//ReadAndVerifyDriveRegisters(drive_index);
	//CheckDriveStatusRegister(drive_index);
}

void DisableMotor(DRIVE_MOTOR drive_index) {
	/*
	 if (CheckDriveStatusRegister(drive_index) != 0) {
	 return;
	 }
	 */

	uint16_t reg_config = ReadRegConfig(drive_index, 0);
	reg_config = reg_config & 0xFFFD;

	uint32_t err = WriteSPI(drive_index, DRV8711_CTRL_REG, reg_config);

	if (err == 0) {
		drive_regs[drive_index].ctrl_reg.enbl = 0;
	}

	//ReadAndVerifyDriveRegisters(drive_index);
}

//uint8_t sub_step = 0;
void StepDrive(DRIVE_MOTOR drive_index) {

	//if (CheckDriveStatusRegister(drive_index) != 0) {
	//	return;
	//}

	uint16_t reg_config = ReadRegConfig(drive_index, 0);
	reg_config = reg_config | 0x0004;

	WriteSPI(drive_index, DRV8711_CTRL_REG, reg_config);

	/*
	 if (sub_step == 1) {
	 sub_step = 0;
	 HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_STEP],
	 drive_pins[drive_index][DRIVE_STEP], GPIO_PIN_SET);
	 } else {
	 sub_step = 1;
	 HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_STEP],
	 drive_pins[drive_index][DRIVE_STEP], GPIO_PIN_RESET);
	 }*/

}

void DirectionMotor(DRIVE_MOTOR drive_index, uint8_t dir) {
	/*
	 if (CheckDriveStatusRegister(drive_index) != 0) {
	 return;
	 }
	 uint16_t reg_config = ReadRegConfig(drive_index, 0);

	 if (drive_regs[drive_index].ctrl_reg.rdir != 0) {
	 drive_regs[drive_index].ctrl_reg.rdir = 0;
	 reg_config = ReadRegConfig(drive_index, 0);
	 WriteSPI(drive_index, DRV8711_CTRL_REG, reg_config);
	 ReadAndVerifyDriveRegisters(drive_index);
	 }
	 */
	if (dir == MOTOR_DIRECTION_LEFT) {
		HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_DIR],
				drive_pins[drive_index][DRIVE_DIR], GPIO_PIN_RESET);
	} else if (dir == MOTOR_DIRECTION_RIGHT) {
		HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_DIR],
				drive_pins[drive_index][DRIVE_DIR], GPIO_PIN_SET);
	} else {
		return;
	}

	//CheckDriveStatusRegister(drive_index);
}

uint32_t max_speed = (uint32_t) ((0xFF * 0.00000002) + (0xFF * 0.0000005)) * 1000000;
#define SPEED 500
uint32_t half_speed = SPEED >> 1;  // Divides by 2
uint32_t counter = 0;
uint32_t one_turn_counter = 0;
uint8_t step_on = 0;
void StepFunction() {
	/*
	 if (step_on == 1) { //0
	 step_on = 0;
	 if (motor_pitch_on == 1) {
	 HAL_GPIO_WritePin(drive_ports[DRIVE1][DRIVE_STEP],
	 drive_pins[DRIVE1][DRIVE_STEP], GPIO_PIN_SET);

	 //StepDrive(DRIVE1);
	 }
	 } else if (step_on == 0) {
	 step_on = 1;
	 HAL_GPIO_WritePin(drive_ports[DRIVE1][DRIVE_STEP], drive_pins[DRIVE1][DRIVE_STEP],
	 GPIO_PIN_RESET);
	 }
	 */

	if (counter >= SPEED) { //0
		counter = 0;
		if (motor_pitch_on == 1) {
			//if (one_turn_counter >= 200) {
			//	motor_pitch_on = 0;
			//	one_turn_counter = 0;
			//CheckDriveStatusRegister(DRIVE1);
			//ReadAndVerifyDriveRegisters(DRIVE1);
			//} else {
			//	one_turn_counter++;
			//}
			//HAL_GPIO_WritePin(LED_CANB_GPIO_Port, LED_CANB_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(drive_ports[DRIVE1][DRIVE_STEP],
					drive_pins[DRIVE1][DRIVE_STEP], GPIO_PIN_SET);
		}
	} else if (counter == half_speed) {
		counter++;
		//if (motor_pitch_on == 1) {
		HAL_GPIO_WritePin(drive_ports[DRIVE1][DRIVE_STEP], drive_pins[DRIVE1][DRIVE_STEP],
				GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(LED_CANB_GPIO_Port, LED_CANB_Pin, GPIO_PIN_RESET);
		//}
	} else {
		counter++;
	}

}

/*
 void InitRegValuesStepperDefault(DRIVE_MOTOR drive_index) {
 //
 // ───── CTRL REGISTER (0x00) ─────────────────────────────────────────────
 // Contrôle principal du moteur (activation, direction, step, mode de microstep)
 //
 drive_regs[drive_index].ctrl_reg.enbl = 0;
 // ENBL : Active la sortie du driver
 // 0 = désactivé (sorties en haute impédance), 1 = activé
 // Par défaut DRV8711 : 0

 drive_regs[drive_index].ctrl_reg.rdir = 0;
 // RDIR : Direction contrôlée par pin DIR si = 0, sinon inversée
 // Par défaut DRV8711 : 0

 drive_regs[drive_index].ctrl_reg.rstep = 0;
 // RSTEP : Step contrôlé par pin STEP si = 0, sinon interne
 // Par défaut DRV8711 : 0

 drive_regs[drive_index].ctrl_reg.mode = 0b0010;
 // MODE : Réglage du microstepping (0 = full step, jusqu’à 0b1111 = 1/256)
 // Par défaut DRV8711 : 0b0010 (1/4 step)

 drive_regs[drive_index].ctrl_reg.extstall = 0;
 // EXTSTALL : 0 = Stall détecté en interne, 1 = via pin externe
 // Par défaut DRV8711 : 0

 drive_regs[drive_index].ctrl_reg.isgain = 0b00;
 // ISGAIN : Gain du senseur de courant (00 = 5, 01 = 10, 10 = 20, 11 = 40)
 // Par défaut DRV8711 : 0b00 (gain de 5)

 drive_regs[drive_index].ctrl_reg.dtime = 0b11;
 // DTIME : Dead time entre les switches du pont H (00 = 850ns)
 // Par défaut DRV8711 : 0b11

 //
 // ───── TORQUE REGISTER (0x01) ───────────────────────────────────────────
 // Détermine le courant de phase appliqué
 //
 //drive_regs[drive_index].torque_reg.torque = 24; // 100% de puissance avec isgain à 10
 drive_regs[drive_index].torque_reg.torque = 0xFF;
 // TORQUE : Niveau de couple (0–255), proportionnel au courant de sortie
 // Par défaut DRV8711 : 0xFF (255)

 drive_regs[drive_index].torque_reg.smplth = 0b001;
 // SMPLTH : Durée du seuil BEMF pour la détection de blocage
 // Par défaut DRV8711 : 0b001

 //
 // ───── OFF REGISTER (0x02) ──────────────────────────────────────────────
 // Temps d’arrêt de PWM et mode PWM
 //
 drive_regs[drive_index].off_reg.toff = 0x30;
 // TOFF : Temps mort (0 = désactivé, >0 = en pas de 500ns)
 // Par défaut DRV8711 : 0x30 (24 = 12 µs)

 drive_regs[drive_index].off_reg.pwmmode = 0;
 // PWMMODE : 0 = interne (indexeur), 1 = externe (STEP/DIR)
 // Par défaut DRV8711 : 0

 //
 // ───── BLANK REGISTER (0x03) ────────────────────────────────────────────
 // Temps de masquage de détection de courant après une commutation
 //
 drive_regs[drive_index].blank_reg.tblank = 0x80;
 // TBLANK : Masque de courant (en pas de 20ns) — 0x80 = 2.56 µs
 // Par défaut DRV8711 : 0x80

 drive_regs[drive_index].blank_reg.abt = 0;
 // ABT : Adaptive Blanking Time (0 = désactivé, 1 = activé)
 // Par défaut DRV8711 : 0

 //
 // ───── DECAY REGISTER (0x04) ────────────────────────────────────────────
 // Contrôle le mode de "décroissance" du courant moteur
 //
 drive_regs[drive_index].decay_reg.tdecay = 0x10;
 // TDECAY : temps avant transition entre fast et slow decay (500ns steps)
 // Par défaut DRV8711 : 0x10 (8 µs)

 drive_regs[drive_index].decay_reg.decmod = 0b001;
 // DECMOD : 000 = slow decay forcé, 001 = fast, 010 = mixed decay
 // Par défaut DRV8711 : 0b001 (slow decay)

 //
 // ───── STALL REGISTER (0x05) ────────────────────────────────────────────
 // Détection de blocage moteur (optionnel)
 //
 drive_regs[drive_index].stall_reg.sdthr = 0x40;
 // SDTHR : Seuil de BEMF pour détecter un blocage
 // Par défaut DRV8711 : 0x40

 drive_regs[drive_index].stall_reg.sdcnt = 0b00;
 // SDCNT : nombre de pas requis avant détection (00 = 2, ..., 11 = 8 pas)
 // Par défaut DRV8711 : 0b00

 drive_regs[drive_index].stall_reg.vdiv = 0b00;
 // VDIV : Diviseur du signal BEMF pour détection (00 = /32)
 // Par défaut DRV8711 : 0b00

 //
 // ───── DRIVE REGISTER (0x06) ────────────────────────────────────────────
 // Réglage de la puissance des transistors MOSFET (gate drive)
 //
 drive_regs[drive_index].drive_reg.ocpth = 0b01;
 // OCPTH : Seuil de détection de surintensité (00 = 250mV, 11 = 2000mV)
 // Par défaut DRV8711 : 0b01 (500mV)

 drive_regs[drive_index].drive_reg.ocpdeg = 0b10;
 // OCPDEG : Temps de filtrage de la surintensité (00 = 1 µs, 11 = 8 µs)
 // Par défaut DRV8711 : 0b10

 drive_regs[drive_index].drive_reg.tdriven = 0b01;
 // TDRIVEN : Durée de l’impulsion LOW-SIDE (00 = 250ns, 11 = 500ns)
 // Par défaut DRV8711 : 0b01

 drive_regs[drive_index].drive_reg.tdrivep = 0b01;
 // TDRIVEP : Durée de l’impulsion HIGH-SIDE (00 = 250ns, 11 = 500ns)
 // Par défaut DRV8711 : 0b01

 drive_regs[drive_index].drive_reg.idriven = 0b10;
 // IDRIVEP : Courant de crête HIGH-SIDE (00 = 20mA, 11 = 150mA)
 // Par défaut DRV8711 : 0b10

 drive_regs[drive_index].drive_reg.idrivep = 0b10;
 // IDRIVEP : Courant de crête HIGH-SIDE (00 = 20mA, 11 = 150mA)
 // Par défaut DRV8711 : 0b10
 }
 */

/*
 uint32_t multiplicator_slowing_motor = 0;
 uint32_t speed_stepper_motor_pitch_int_converted = 40; //in multiple of 50us
 uint8_t gpio_pin_value = 0;
 //fonctionne avec un timer d'un multiple de la vitesse maximale
 //better_step_function() s'exécute tous les 2ms
 //vitesse max 500kHz, soit multiplicator_slowing_motor > stepper_motor_pitch où stepper_motor_pitch = 40 (50us*40=2ms)
 void better_step_function() {
 return 0;

 if (speed_stepper_motor_pitch >= 100) {
 speed_stepper_motor_pitch_int_converted = 40;
 } else {
 speed_stepper_motor_pitch_int_converted = (uint32_t) (1
 / (float) ((float) speed_stepper_motor_pitch / 100));
 }
 if (speed_stepper_motor_pitch_int_converted < 40)
 speed_stepper_motor_pitch_int_converted = 40; //sécurité sinon moteur bloque et besoin de HARD RESET toute la boite élé

 if (multiplicator_slowing_motor
 <= speed_stepper_motor_pitch_int_converted) {
 multiplicator_slowing_motor++;
 } else {
 multiplicator_slowing_motor = 0;
 if (motor_pitch_on == 1) {
 if (gpio_pin_value == 0) {
 gpio_pin_value = 1;
 HAL_GPIO_WritePin(drive_ports[DRIVE2][DRIVE_STEP],
 drive_pins[DRIVE2][DRIVE_STEP], GPIO_PIN_SET);
 } else if (gpio_pin_value == 1) {
 gpio_pin_value = 0;
 HAL_GPIO_WritePin(drive_ports[DRIVE2][DRIVE_STEP],
 drive_pins[DRIVE2][DRIVE_STEP], GPIO_PIN_RESET);
 }
 }
 }
 } */

/*

 //valeurs de registre du moteur DC compé C12 sur la DRIVE1 du haut
 void InitRegValuesPWM(DRIVE_MOTOR drive_index) {
 // CTRL register
 // drive_regs[drive_index].ctrl_reg.enbl = 1;	// Enable motor
 drive_regs[drive_index].ctrl_reg.enbl = 0;// Disable motor drive by default
 drive_regs[drive_index].ctrl_reg.rdir = 0;	// Direction set by DIR pin
 drive_regs[drive_index].ctrl_reg.rstep = 0;	// No automatic stepping
 //drive_regs[drive_index].ctrl_reg.mode = 0b0010;	// 1/4 step
 drive_regs[drive_index].ctrl_reg.mode = 0b0000;	// Full step
 drive_regs[drive_index].ctrl_reg.extstall = 0;	// Internal stall detect
 // drive_regs[drive_index].ctrl_reg.isgain = 0b00;	// Gain of 5
 drive_regs[drive_index].ctrl_reg.isgain = 0b01;	// Gain of 10
 drive_regs[drive_index].ctrl_reg.dtime = 0b00;	// Dead-time of 850ns

 // TORQUE register
 drive_regs[drive_index].torque_reg.torque = 24;	// max 2.5A  //0xFF;	// Sets full torque for H-bridge
 drive_regs[drive_index].torque_reg.smplth = 0b111;// 100us BEMF sample threshold

 // OFF register
 drive_regs[drive_index].off_reg.toff = 0x80;// Sets fixed off time, in increments of 500ns
 // drive_regs[drive_index].off_reg.pwmmode = pwm_mode;	// Use internal indexer
 drive_regs[drive_index].off_reg.pwmmode = 1;

 // BLANK register
 drive_regs[drive_index].blank_reg.tblank = 0x80;// Current trip blanking time, in increments of 20ns
 drive_regs[drive_index].blank_reg.abt = 0;// Disable adaptive blanking time

 // DECAY register
 drive_regs[drive_index].decay_reg.tdecay = 0x80;// Mixed decay transition time, in increments of 500ns
 drive_regs[drive_index].decay_reg.decmod = 0b000;// Force slow decay at all times

 // STALL register
 drive_regs[drive_index].stall_reg.sdthr = 0xFF;	// Stall detect threshold
 drive_regs[drive_index].stall_reg.sdcnt = 0b11;	// STALLn asserted on first step with BEMF below SDTHR
 drive_regs[drive_index].stall_reg.vdiv = 0b00;	// BEMF divided by 32

 // DRIVE register
 drive_regs[drive_index].drive_reg.ocpth = 0b11;	// OCP threshold of 500mV
 drive_regs[drive_index].drive_reg.ocpdeg = 0b11;// OCP deglitch time of 4us
 drive_regs[drive_index].drive_reg.tdriven = 0b11;// Low-side gate drive time of 500ns
 drive_regs[drive_index].drive_reg.tdrivep = 0b11;// High-side gate drive time of 500ns
 drive_regs[drive_index].drive_reg.idriven = 0b11;// Low-side gate drive peak current of 300mA peak (sink)
 drive_regs[drive_index].drive_reg.idrivep = 0b11;// High-side gate drive peak current of 150mA peak (sink)
 }

 //valeurs de registre du moteur stepper compé C12 sur la DRIVE2 du bas
 void InitRegValues2(DRIVE_MOTOR drive_index) {
 // CTRL register
 // drive_regs[drive_index].ctrl_reg.enbl = 1;	// Enable motor
 drive_regs[drive_index].ctrl_reg.enbl = 0;// Disable motor drive by default
 drive_regs[drive_index].ctrl_reg.rdir = 0;	// Direction set by DIR pin
 drive_regs[drive_index].ctrl_reg.rstep = 0;	// No automatic stepping
 //drive_regs[drive_index].ctrl_reg.mode = 0b0010;	// 1/4 step
 if (drive_index == DRIVE_PITCH) {
 //drive_regs[drive_index].ctrl_reg.mode = 0b0010;	// 1/4 step
 drive_regs[drive_index].ctrl_reg.mode = 0b0001;	// 1/2 step
 //drive_regs[drive_index].ctrl_reg.mode = 0b0000;	// Full step
 } else {
 drive_regs[drive_index].ctrl_reg.mode = 0b0000;	// Full step
 }
 drive_regs[drive_index].ctrl_reg.extstall = 0;	// Internal stall detect
 // drive_regs[drive_index].ctrl_reg.isgain = 0b00;	// Gain of 5
 drive_regs[drive_index].ctrl_reg.isgain = 0b01;	// Gain of 10
 drive_regs[drive_index].ctrl_reg.dtime = 0b00;	// Dead-time of 850ns

 // TORQUE register
 drive_regs[drive_index].torque_reg.torque = 24;	// max 2.5A  //0xFF;	// Sets full torque for H-bridge
 drive_regs[drive_index].torque_reg.smplth = 0b111;// 100us BEMF sample threshold

 // OFF register
 drive_regs[drive_index].off_reg.toff = 0x80;// Sets fixed off time, in increments of 500ns
 // drive_regs[drive_index].off_reg.pwmmode = pwm_mode;	// Use internal indexer
 drive_regs[drive_index].off_reg.pwmmode = 1;

 // BLANK register
 drive_regs[drive_index].blank_reg.tblank = 0x80;// Current trip blanking time, in increments of 20ns
 drive_regs[drive_index].blank_reg.abt = 0;// Disable adaptive blanking time

 // DECAY register
 drive_regs[drive_index].decay_reg.tdecay = 0x80;// Mixed decay transition time, in increments of 500ns
 if (drive_index == DRIVE_PITCH) {
 drive_regs[drive_index].decay_reg.decmod = 0b010;// Force fast decay at all times
 } else {
 drive_regs[drive_index].decay_reg.decmod = 0b000;// Force slow decay at all times
 }

 // STALL register
 drive_regs[drive_index].stall_reg.sdthr = 0xFF;	// Stall detect threshold
 drive_regs[drive_index].stall_reg.sdcnt = 0b11;	// STALLn asserted on first step with BEMF below SDTHR
 drive_regs[drive_index].stall_reg.vdiv = 0b00;	// BEMF divided by 32

 // DRIVE register
 drive_regs[drive_index].drive_reg.ocpth = 0b11;	// OCP threshold of 500mV
 drive_regs[drive_index].drive_reg.ocpdeg = 0b11;// OCP deglitch time of 4us
 drive_regs[drive_index].drive_reg.tdriven = 0b11;// Low-side gate drive time of 500ns
 drive_regs[drive_index].drive_reg.tdrivep = 0b11;// High-side gate drive time of 500ns
 drive_regs[drive_index].drive_reg.idriven = 0b11;// Low-side gate drive peak current of 300mA peak (sink)
 drive_regs[drive_index].drive_reg.idrivep = 0b11;// High-side gate drive peak current of 150mA peak (sink)
 }

 */
