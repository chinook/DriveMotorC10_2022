/*
 * motor_lib.c
 *
 *  Created on: Aug 13, 2025
 *      Author: thoma
 */
#include "drv8711_lib.h"

#include <stdio.h>
#include <string.h>

//Gate Charge Total (Qg)
#define Qg 18 //nC (nano-Coulombs)
//value for the CSD88537ND FET : 14nC -> 18nC worst case

//Desired gate-charge time (RT)
//the time to charge the FET’s gate (gate-voltage rise time)
#define RT 400 //ns
//pick a value near reg CTRL TDTIME

//motor configuration

//1000 for a nema17HS08 or nema17HS19-2004S1
#define STEP_ANGLE 1.8 //in degree
#define NBR_STEP_ONE_TURN 200 //(360/STEP_ANGLE)

#define MAX_RPM 1000 //in RPM
#define MAX_RPS 16 //MAX_RPM / 60 in RPS
#define MAX_HZ 3200 //MAX_RPS * NBR_STEP_ONE_TURN in Hz
#define MIN_DELAY 312 //1 / MAX_HZ in us


//TORQUE
#define Ichop

uint32_t spi_error = 0;
uint32_t hal_spi_transmit_error = 0;
uint32_t hal_spi_transmitreceive_error = 0;

SPI_HandleTypeDef *hspi = &hspi1;

const GPIO_TypeDef *drive_ports[DRIVE_MOTOR_NUM][NUM_PINS] = { {
RESET1_GPIO_Port,
nSLEEP1_GPIO_Port,
SPI_CS1_GPIO_Port,
STEP1_GPIO_Port,
DIR1_GPIO_Port }, {
RESET2_GPIO_Port,
nSLEEP2_GPIO_Port,
SPI_CS2_GPIO_Port,
STEP2_GPIO_Port,
DIR2_GPIO_Port } };

const uint16_t drive_pins[DRIVE_MOTOR_NUM][NUM_PINS] = { {
RESET1_Pin,
nSLEEP1_Pin,
SPI_CS1_Pin,
STEP1_Pin,
DIR1_Pin }, {
RESET2_Pin,
nSLEEP2_Pin,
SPI_CS2_Pin,
STEP2_Pin,
DIR2_Pin } };

DRV8711_REGS drive_regs[DRIVE_MOTOR_NUM];

void InitRegValuesStepper(DRIVE_MOTOR drive_index) {
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
// RSTEP : 1: Indexer will advance one step; automatically cleared after write
// Par défaut DRV8711 : 0

	drive_regs[drive_index].ctrl_reg.mode = 0b0000;
// MODE : Réglage du microstepping (0 = full step, jusqu’à 0b1000 = 1/256)
// Par défaut DRV8711 : 0b0000 (full step)

	drive_regs[drive_index].ctrl_reg.extstall = 0;
// EXTSTALL : 0 = Stall détecté en interne, 1 = via pin externe
// Par défaut DRV8711 : 0

	drive_regs[drive_index].ctrl_reg.isgain = 0b00;
// ISGAIN : Gain du senseur de courant (00 = 5, 01 = 10, 10 = 20, 11 = 40)
// Par défaut DRV8711 : 0b00 (gain de 5)

	drive_regs[drive_index].ctrl_reg.dtime = 0b00;
// DTIME : Dead time entre les switches du pont H (00 = 400ns, 01 = 450ns, 10 = 650ns, 11 = 850ns)
// Par défaut DRV8711 : 0b00

//
// ───── TORQUE REGISTER (0x01) ───────────────────────────────────────────
// Détermine le courant de phase appliqué
//
	drive_regs[drive_index].torque_reg.torque = 8; // 100% de puissance avec isgain à 20
// TORQUE : Niveau de couple (0–255), proportionnel au courant de sortie
// Par défaut DRV8711 : 0xFF (255)

	drive_regs[drive_index].torque_reg.smplth = 0b000;
// SMPLTH : Durée du seuil BEMF pour la détection de blocage
// Par défaut DRV8711 : 0b001

//
// ───── OFF REGISTER (0x02) ──────────────────────────────────────────────
// Temps d’arrêt de PWM et mode PWM
//
	drive_regs[drive_index].off_reg.toff = 0x00;
// TOFF : Temps mort (0 = désactivé, >0 = en pas de 500ns)
// Par défaut DRV8711 : 0x30 (24 = 12 µs)

	drive_regs[drive_index].off_reg.pwmmode = 0;
// PWMMODE : 0 = interne (indexeur), 1 = externe (STEP/DIR)
// Par défaut DRV8711 : 0

//// ───── BLANK REGISTER (0x03) ────────────────────────────────────────────
// Temps de masquage de détection de courant après une commutation
//
	drive_regs[drive_index].blank_reg.tblank = 0x00;
// TBLANK : Masque de courant (en pas de 20ns) - — 0x80 = 2.56 µs
// Par défaut DRV8711 : 0x80

	drive_regs[drive_index].blank_reg.abt = 1;
// ABT : Adaptive Blanking Time (0 = désactivé, 1 = activé)
// Par défaut DRV8711 : 0

//
// ───── DECAY REGISTER (0x04) ────────────────────────────────────────────
// Contrôle le mode de "décroissance" du courant moteur
//
	drive_regs[drive_index].decay_reg.tdecay = 0x10;
// TDECAY : temps avant transition entre fast et slow decay (500ns steps)
// Par défaut DRV8711 : 0x10 (8 µs)

	drive_regs[drive_index].decay_reg.decmod = 0b101;
// DECMOD : Use auto mixed decay at all times
// Par défaut DRV8711 : 0b001 Slow decay for increasing current, mixed decay for decreasing current (indexer mode only)

//
// ───── STALL REGISTER (0x05) ────────────────────────────────────────────
// Détection de blocage moteur (optionnel)
//
	drive_regs[drive_index].stall_reg.sdthr = 0x3F; //0xFF
// SDTHR : Seuil de BEMF pour détecter un blocage
// Par défaut DRV8711 : 0x40

	drive_regs[drive_index].stall_reg.sdcnt = 0b00; //0b11
// SDCNT : nombre de pas requis avant détection (00 = 2, ..., 11 = 8 pas)
// Par défaut DRV8711 : 0b00

	drive_regs[drive_index].stall_reg.vdiv = 0b10;
// VDIV : Diviseur du signal BEMF pour détection (00 = /32)
// Par défaut DRV8711 : 0b00

//
// ───── DRIVE REGISTER (0x06) ────────────────────────────────────────────
// Réglage de la puissance des transistors MOSFET (gate drive)
//
	drive_regs[drive_index].drive_reg.ocpth = 0b00; //00
// OCPTH : OCP threshold (00 = 250mV, 01 = 500mV, 10 = 750mV, 11 = 1000mV)
// Par défaut DRV8711 : 0b01

	drive_regs[drive_index].drive_reg.ocpdeg = 0b10; //10
// OCPDEG : OCP deglitch time (00 = 1 µs, 01 = 2 µs, 10 = 4 µs, 11 = 8 µs)
// Par défaut DRV8711 : 0b10

	drive_regs[drive_index].drive_reg.tdriven = 0b10; //10
// TDRIVEN : Low-side gate drive time (00 = 250ns, 01 = 500ns, 10 = 1us, 11 = 2us)
// Par défaut DRV8711 : 0b01

	drive_regs[drive_index].drive_reg.tdrivep = 0b10; //10
// TDRIVEP : High-side gate drive time (00 = 250ns, 01 = 500ns, 10 = 1us, 11 = 2us)
// Par défaut DRV8711 : 0b01

	drive_regs[drive_index].drive_reg.idriven = 0b00; //11
// IDRIVEP : Low-side gate drive peak current peak (sink) (00 = 100mA, 01 = 200mA, 10 = 300mA, 11 = 400mA)
// Par défaut DRV8711 : 0b10

	drive_regs[drive_index].drive_reg.idrivep = 0b00; //11
// IDRIVEP : High-side gate drive peak current peak (source) (00 = 50mA, 01 = 100mA, 10 = 150mA, 11 = 200mA)
// Par défaut DRV8711 : 0b10
}

uint16_t ReadRegConfig(DRIVE_MOTOR drive_index, uint8_t reg) {
	uint16_t reg_data = -1;

	switch (reg) {
	case 0:
		reg_data = drive_regs[drive_index].ctrl_reg.reg;
		break;
	case 1:
		reg_data = drive_regs[drive_index].torque_reg.reg;
		break;
	case 2:
		reg_data = drive_regs[drive_index].off_reg.reg;
		break;
	case 3:
		reg_data = drive_regs[drive_index].blank_reg.reg;
		break;
	case 4:
		reg_data = drive_regs[drive_index].decay_reg.reg;
		break;
	case 5:
		reg_data = drive_regs[drive_index].stall_reg.reg;
		break;
	case 6:
		reg_data = drive_regs[drive_index].drive_reg.reg;
		break;
	case 7:
		reg_data = drive_regs[drive_index].status_reg.reg;
		break;
	default:
		reg_data = 0;
		break;
	}
	return reg_data;
}

void SelectDriveCS(DRIVE_MOTOR drive_index) {
	for (uint8_t num = 0; num < DRIVE_MOTOR_NUM; num++) {
		if (num != drive_index) {
			// Désélectionner la puce (CS à LOW après la transmission)
			HAL_GPIO_WritePin(drive_ports[num][DRIVE_CS], drive_pins[num][DRIVE_CS],
					GPIO_PIN_RESET);
		} else if (num == drive_index) {
			// Sélectionner la puce (CS à HIGH avant la transmission)
			HAL_GPIO_WritePin(drive_ports[num][DRIVE_CS], drive_pins[num][DRIVE_CS],
					GPIO_PIN_SET);
		}
	}
}

void UnselectDriveCS() {
	for (uint8_t num = 0; num < DRIVE_MOTOR_NUM; num++) {
		// Désélectionner la puce (CS à LOW après la transmission)
		HAL_GPIO_WritePin(drive_ports[num][DRIVE_CS], drive_pins[num][DRIVE_CS],
				GPIO_PIN_RESET);
	}
}

uint16_t TransmitReceiveSPI(DRIVE_MOTOR drive_index, uint16_t data) {
	// Convertir en 2 octets MSB-first
	uint8_t tx_data[2] = { (data >> 8) & 0xFF, data & 0xFF };

	uint8_t rx_data[2] = { 0 };

	// Sélectionner la puce (CS à HIGH avant la transmission)
	SelectDriveCS(drive_index);

	// Transmettre le message
	uint8_t err = HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 2,
	HAL_MAX_DELAY);

	if (err != HAL_OK) {
		// SPI erreur → allumer LED de diagnostic
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		hal_spi_transmitreceive_error++;
	}

	// Désélectionner les puces (CS à LOW après la transmission)
	UnselectDriveCS();

	// Reconstruire la valeur lue
	uint16_t received_data = ((uint16_t) rx_data[0] << 8) | rx_data[1];

	/*//test vs expected data
	 received_data = received_data & 0x0FFF;
	 uint8_t reg_test = (data & 0x7000) >> 12;
	 uint16_t expected_data_test = ReadRegConfig(drive_index, reg_test);
	 if ((reg_test == 1) && (expected_data_test & 0x0300)) { // datasheet bit 10 on register 1 always return 0
	 expected_data_test = expected_data_test & 0xFBFF;
	 }
	 if ((reg_test == 0) && (expected_data_test & 0x0002)) { // datasheet bit 2 on register 0 clear after write
	 expected_data_test = expected_data_test & 0xFFFD;
	 }

	 if (received_data != expected_data_test) {
	 //HAL_GPIO_WritePin(LED_CANB_GPIO_Port, LED_CANB_Pin, GPIO_PIN_SET);
	 //flag_drive_fault = 1;
	 }*/

	// Extraire seulement les 12 bits de data
	return received_data & 0x0FFF;

}

uint16_t ReadSPI(DRIVE_MOTOR drive_index, uint8_t reg) {
	// Construire la commande de lecture : bit 15 = 1, reg sur bits 14–12
	uint16_t read_cmd = (1 << 15) | (reg << 12);

	uint16_t received_data = TransmitReceiveSPI(drive_index, read_cmd);

	if ((reg == DRV8711_TORQUE_REG)
			&& (ReadRegConfig(drive_index, reg) & 0b010000000000)) { // datasheet bit 10 on register 1 always return 0
		received_data = received_data | 0b010000000000;
	}
	if ((reg == DRV8711_CTRL_REG) && (received_data & 0b000000000100)) { // datasheet bit 2 on register 0 clear after write
		received_data = received_data & 0b111111111011;
	}

	return received_data;
}

void TransmitSPI(DRIVE_MOTOR drive_index, uint16_t data) {
	// Convertir en 2 octets MSB-first
	uint8_t tx_data[2] = { (data >> 8) & 0xFF, data & 0xFF };

	//On écrie sur la puce :
	tx_data[0] = tx_data[0] & 0x7F;

	// Sélectionner la puce (CS à HIGH avant la transmission)
	SelectDriveCS(drive_index);

	// Transmettre le message
	uint8_t err = HAL_SPI_Transmit(hspi, tx_data, 2, HAL_MAX_DELAY);
	if (err != HAL_OK) {
		// SPI erreur → allumer LED de diagnostic
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		hal_spi_transmit_error++;
	}

	// Désélectionner les puces (CS à LOW après la transmission)
	UnselectDriveCS();
}

uint32_t WriteSPI(DRIVE_MOTOR drive_index, uint8_t reg, uint16_t reg_config) {
	uint16_t data = (reg << 12) & 0x7000; //registre
	data = data | (reg_config & 0x0FFF); //config
	data = data & 0x7FFF; //écriture
	uint16_t expected_data = data & 0x0FFF;

	TransmitSPI(drive_index, data);
	uint16_t received_data = ReadSPI(drive_index, reg);

	if (received_data != expected_data) {
		TransmitSPI(drive_index, data);
		received_data = ReadSPI(drive_index, reg);
		if (received_data != expected_data) {
			//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
			spi_error++;
			return spi_error;
		}
	}
	return 0;
}

void SendDriveRegisters(DRIVE_MOTOR drive_index) {
	// Écrire tous les registres sauf STATUS (registre 0x07)
	for (uint8_t reg = 0; reg < NUM_DRIVE_REGS - 1; reg++) {
		uint16_t reg_config = ReadRegConfig(drive_index, reg);
		WriteSPI(drive_index, reg, reg_config);
	}
}

void ReadAndVerifyDriveRegisters(DRIVE_MOTOR drive_index) {
	uint8_t error_detected = 0;

	for (uint8_t reg = 0; reg < NUM_DRIVE_REGS - 1; reg++) { // Skip STATUS (0x07)
		uint16_t received_data = ReadSPI(drive_index, reg);
		uint16_t expected_data = ReadRegConfig(drive_index, reg);

		if (received_data != expected_data) {
			//HAL_GPIO_WritePin(LED_CANB_GPIO_Port, LED_CANB_Pin, GPIO_PIN_SET);
			error_detected += 1;
		}
	}
}

/*
 * DRV8711 STATUS REGISTER (0x07) - 16 bits (lecture seule)
 * ----------------------------------------------------------
 * Seuls les bits 0 à 7 sont utilisés pour refléter l'état de la puce.
 * Les bits 7 à 0 sont réservés ou non utilisés (généralement ignorés).
 *
 * Format des bits (MSB → LSB) :
 *
 * Bit 0 - OTS     (Overtemperature Shutdown)
 *           → Surchauffe interne détectée. Le pont H est désactivé pour protéger le circuit.
 *
 * Bit 1 - AOCP    (Channel A Overcurrent Protection)
 *           → Surintensité détectée sur le pont A. Risque de court-circuit ou de moteur défectueux.
 *
 * Bit 2 - BOCP    (Channel B Overcurrent Protection)
 *           → Surintensité détectée sur le pont B.
 *
 * Bit 3 - APDF    (Channel A Pre-Driver Fault)
 *           → Défaut dans le circuit de commande des MOSFETs du pont A.
 *
 * Bit 4 - BPDF    (Channel B Pre-Driver Fault)
 *           → Défaut dans le circuit de commande des MOSFETs du pont B.
 *
 * Bit 5 - UVLO    (Undervoltage Lockout)
 *           → Tension d’alimentation insuffisante. Les moteurs sont désactivés par sécurité.
 *
 * Bit 6 - STD     (Stall Detect)
 *           → Blocage moteur détecté par comparaison BEMF < seuil configuré.
 *
 * Bit 8 - STDLAT  (Stall Detect Latch)
 *           → Latch du blocage moteur ; reste à 1 tant que non réinitialisé.
 */
static uint16_t last_status = 0;
uint8_t flag_drive_fault = 0;
uint8_t CheckDriveStatusRegister(DRIVE_MOTOR drive_index) {
	uint16_t status = ReadSPI(drive_index, DRV8711_STATUS_REG);

	//ResetDriveStatusRegister(drive_index);

	if (status != 0x0000) {
		// Une ou plusieurs erreurs détectées → flag drive en faute
		HAL_GPIO_TogglePin(LED_WARNING_GPIO_Port, LED_WARNING_Pin);
		last_status = status;
		flag_drive_fault = 1;
		return 1;
	} else {
		// Pas d'erreur
		flag_drive_fault = 0;
		return 0;
	}

}

void ResetDriveStatusRegister(DRIVE_MOTOR drive_index) {
	WriteSPI(drive_index, DRV8711_STATUS_REG, 0);
}
