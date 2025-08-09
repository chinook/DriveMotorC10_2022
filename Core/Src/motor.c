/*
 * motor.c
 *
 *  Created on: 25 mai 2022
 *      Author: Marc
 *  Edited by : Thomas Maitre after June 2024
 */

#include "motor.h"
#include "main.h"

#include <stdio.h>
#include <string.h>

enum DRIVE_PINS {
	DRIVE_RESET, DRIVE_SLEEP, DRIVE_CS, DRIVE_STEP, DRIVE_DIR,

	NUM_PINS
};

GPIO_TypeDef *drive_ports[DRIVE_MOTOR_NUM][NUM_PINS] = { {
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

uint16_t drive_pins[DRIVE_MOTOR_NUM][NUM_PINS] = { {
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

// DRV8711 Register map addresses
typedef enum {
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
typedef union {
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

typedef union {
	uint16_t reg;
	struct {
		uint16_t torque :8;
		uint16_t smplth :3;
		uint16_t reserved :5;
	};
} TORQUE_REG;

typedef union {
	uint16_t reg;
	struct {
		uint16_t toff :8;
		uint16_t pwmmode :1;
		uint16_t reserved :7;
	};
} OFF_REG;

typedef union {
	uint16_t reg;
	struct {
		uint16_t tblank :8;
		uint16_t abt :1;
		uint16_t reserved :7;
	};
} BLANK_REG;

typedef union {
	uint16_t reg;
	struct {
		uint16_t tdecay :8;
		uint16_t decmod :3;
		uint16_t reserved :5;
	};
} DECAY_REG;

typedef union {
	uint16_t reg;
	struct {
		uint16_t sdthr :8;
		uint16_t sdcnt :2;
		uint16_t vdiv :2;
		uint16_t reserved :4;
	};
} STALL_REG;

typedef union {
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

typedef union {
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

typedef struct {
	CTRL_REG ctrl_reg;
	TORQUE_REG torque_reg;
	OFF_REG off_reg;
	BLANK_REG blank_reg;
	DECAY_REG decay_reg;
	STALL_REG stall_reg;
	DRIVE_REG drive_reg;
	STATUS_REG status_reg;
} DRV8711_REGS;

DRV8711_REGS drive_regs[DRIVE_MOTOR_NUM];

uint32_t drive_status[DRIVE_MOTOR_NUM] = { DRIVE_AWAKE, DRIVE_AWAKE };

void InitDriveMotor(DRIVE_MOTOR drive_index);
void InitRegValuesStepper(DRIVE_MOTOR drive_index);
void SendDriveRegisters(DRIVE_MOTOR drive_index);
void ReadAndVerifyDriveRegisters(DRIVE_MOTOR drive_index);
void CheckDriveStatusRegister(DRIVE_MOTOR drive_index);

void TransmitSPI(DRIVE_MOTOR drive_index, uint16_t data);
void WriteSPI(DRIVE_MOTOR drive_index, uint8_t reg);
uint16_t ReadSPI(DRIVE_MOTOR drive_index, uint8_t reg);

uint16_t TransmitReceiveSPI(DRIVE_MOTOR drive_index, uint16_t data);
void SelectDriveCS(DRIVE_MOTOR drive_index);
void UnselectDriveCS();
uint16_t ReadRegConfig(DRIVE_MOTOR drive_index, uint8_t reg);

void InitRegValuesStepperDefault(DRIVE_MOTOR drive_index);

SPI_HandleTypeDef *hspi = &hspi1;

void InitDrives() {
	/*
	 pwm1_timer = htim_pwm1;
	 pwm2_timer = htim_pwm2;
	 pwm1_channel = channel_pwm1;
	 pwm2_channel = channel_pwm2;

	 pwm_timers[0] = pwm1_timer;
	 pwm_timers[1] = pwm2_timer;
	 pwm_channels[0] = pwm1_channel;
	 pwm_channels[1] = pwm2_channel;
	 */
	UnselectDriveCS();
	//InitDriveMotor(DRIVE1);
	InitDriveMotor(DRIVE1);
}

void InitDriveMotor(DRIVE_MOTOR drive_index) {
	// Reset drive
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_RESET],
			drive_pins[drive_index][DRIVE_RESET], GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_RESET],
			drive_pins[drive_index][DRIVE_RESET], GPIO_PIN_RESET);

	// Disable sleeping
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_SLEEP],
			drive_pins[drive_index][DRIVE_SLEEP], GPIO_PIN_SET);

	// CS à LOW
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
			drive_pins[drive_index][DRIVE_CS], GPIO_PIN_RESET);

	// init register for a stepper motor
	InitRegValuesStepper(drive_index);
	//InitRegValuesStepperDefault(drive_index);

	// Send regs over SPI
	SendDriveRegisters(drive_index);

	// Verify if chip setup is good
	ReadAndVerifyDriveRegisters(drive_index);

}

// Send every register to the drive
void SendDriveRegisters(DRIVE_MOTOR drive_index) {
	// Écrire tous les registres sauf STATUS (registre 0x07)
	for (uint8_t reg = 0; reg < NUM_DRIVE_REGS - 1; reg++) {
		WriteSPI(drive_index, reg);
	}
}

void ReadAndVerifyDriveRegisters(DRIVE_MOTOR drive_index) {
	uint8_t error_detected = 0;

	for (uint8_t i = 0; i < 255; i++) {
		for (uint8_t reg = NUM_DRIVE_REGS - 2; reg < NUM_DRIVE_REGS - 1;
				reg--) { // Skip STATUS (0x07)
			uint16_t received_data = ReadSPI(drive_index, reg);

			uint16_t expected_data = ReadRegConfig(drive_index, reg);
			if ((reg == 1) && (expected_data & 0x0300)) { // datasheet bit 10 on register 1 always return 0
				expected_data = expected_data &0xFBFF;
			}

			if (received_data != expected_data) {
				HAL_GPIO_WritePin(LED_CANB_GPIO_Port, LED_CANB_Pin,
						GPIO_PIN_SET);
				flag_drive_fault = 1;
				error_detected += 1;
			}
		}
	}
}

void WriteSPI(DRIVE_MOTOR drive_index, uint8_t reg) {
	uint16_t reg_config = ReadRegConfig(drive_index, reg);

	uint16_t data = (reg << 12) & 0x7000; //registre
	data = data | (reg_config & 0x0FFF); //config
	data = data & 0x7FFF; //écriture

	TransmitSPI(drive_index, data);

	ReadSPI(drive_index, reg);
}

uint16_t ReadSPI(DRIVE_MOTOR drive_index, uint8_t reg) {
	// Construire la commande de lecture : bit 15 = 1, reg sur bits 14–12
	uint16_t read_cmd = (1 << 15) | (reg << 12);

	uint16_t received_data = TransmitReceiveSPI(drive_index, read_cmd);

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
	if (HAL_SPI_Transmit(hspi, tx_data, 2, HAL_MAX_DELAY) != HAL_OK) {
		// SPI erreur → allumer LED de diagnostic
		HAL_GPIO_WritePin(LED_CANB_GPIO_Port, LED_CANB_Pin, GPIO_PIN_SET);
	}

	// Désélectionner les puces (CS à LOW après la transmission)
	UnselectDriveCS();
}

uint16_t TransmitReceiveSPI(DRIVE_MOTOR drive_index, uint16_t data) {
	// Convertir en 2 octets MSB-first
	uint8_t tx_data[2] = { (data >> 8) & 0xFF, data & 0xFF };

	uint8_t rx_data[2] = { 0 };

	// Sélectionner la puce (CS à HIGH avant la transmission)
	SelectDriveCS(drive_index);

	// Transmettre le message
	if (HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 2, HAL_MAX_DELAY)) {
		// SPI erreur → allumer LED de diagnostic
		HAL_GPIO_WritePin(LED_CANB_GPIO_Port, LED_CANB_Pin, GPIO_PIN_SET);
	}

	// Désélectionner les puces (CS à LOW après la transmission)
	UnselectDriveCS();

	// Reconstruire la valeur lue
	uint16_t received_data = ((uint16_t) rx_data[0] << 8) | rx_data[1];

	//test vs expected data
	received_data = received_data & 0x0FFF;
	uint8_t reg_test = (data & 0x7000) >> 12;
	uint16_t expected_data_test = ReadRegConfig(drive_index, reg_test);
	if ((reg_test == 1) && (expected_data_test & 0x0300)) { // datasheet bit 10 on register 1 always return 0
		expected_data_test = expected_data_test &0xFBFF;
	}

	if (received_data != expected_data_test) {
		HAL_GPIO_WritePin(LED_CANB_GPIO_Port, LED_CANB_Pin, GPIO_PIN_SET);
		//flag_drive_fault = 1;
	}

	// Extraire seulement les 12 bits de data
	return received_data & 0x0FFF;

}

void SelectDriveCS(DRIVE_MOTOR drive_index) {
	for (uint8_t num = 0; num < DRIVE_MOTOR_NUM; num++) {
		if (num != drive_index) {
			// Désélectionner la puce (CS à LOW après la transmission)
			HAL_GPIO_WritePin(drive_ports[num][DRIVE_CS],
					drive_pins[num][DRIVE_CS], GPIO_PIN_RESET);
		} else if (num == drive_index) {
			// Sélectionner la puce (CS à HIGH avant la transmission)
			HAL_GPIO_WritePin(drive_ports[num][DRIVE_CS],
					drive_pins[num][DRIVE_CS], GPIO_PIN_SET);
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

/*
 * DRV8711 STATUS REGISTER (0x07) - 16 bits (lecture seule)
 * ----------------------------------------------------------
 * Seuls les bits 15 à 8 sont utilisés pour refléter l'état de la puce.
 * Les bits 7 à 0 sont réservés ou non utilisés (généralement ignorés).
 *
 * Format des bits (MSB → LSB) :
 *
 * Bit 15 - OTS     (Overtemperature Shutdown)
 *           → Surchauffe interne détectée. Le pont H est désactivé pour protéger le circuit.
 *
 * Bit 14 - AOCP    (Bridge A Overcurrent Protection)
 *           → Surintensité détectée sur le pont A. Risque de court-circuit ou de moteur défectueux.
 *
 * Bit 13 - BOCP    (Bridge B Overcurrent Protection)
 *           → Surintensité détectée sur le pont B.
 *
 * Bit 12 - APDF    (Bridge A Pre-Driver Fault)
 *           → Défaut dans le circuit de commande des MOSFETs du pont A.
 *
 * Bit 11 - BPDF    (Bridge B Pre-Driver Fault)
 *           → Défaut dans le circuit de commande des MOSFETs du pont B.
 *
 * Bit 10 - UVLO    (Undervoltage Lockout)
 *           → Tension d’alimentation insuffisante. Les moteurs sont désactivés par sécurité.
 *
 * Bit  9 - STD     (Stall Detect)
 *           → Blocage moteur détecté par comparaison BEMF < seuil configuré.
 *
 * Bit  8 - STDLAT  (Stall Detect Latch)
 *           → Latch du blocage moteur ; reste à 1 tant que non réinitialisé.
 *
 * Exemple :
 * Si rx[0] = 0x84 (donc status_reg = 0x8400) :
 *   → Bit 15 (OTS) = 1 → surchauffe
 *   → Bit 10 (UVLO) = 1 → tension trop faible
 *   → Tous les autres bits = 0 → pas de défaut détecté sur les ponts A/B
 */
uint8_t flag_drive_fault = 0;
void CheckDriveStatusRegister(DRIVE_MOTOR drive_index) {
	uint16_t status = ReadSPI(drive_index, 7);

// Récupérer les bits d’état (seuls les bits 15–8 comptent)
	uint8_t status_flags = (status >> 8) & 0xFF;

	if (status_flags != 0x00) {
		// Une ou plusieurs erreurs détectées → flag drive en faute
		flag_drive_fault = 1;
	} else {
		// Pas d'erreur
		flag_drive_fault = 0;
	}
}

void EnableDrive(DRIVE_MOTOR drive_index) {
// Préparer le registre
	drive_regs[drive_index].ctrl_reg.enbl = 1;

// CS à LOW avant la transmission
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
			drive_pins[drive_index][DRIVE_CS], GPIO_PIN_RESET);

// Envoyer le registre CTRL
//TransmitMotorSPI(drive_index, DRV8711_CTRL_REG);

// CS à HIGH après la transmission
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
			drive_pins[drive_index][DRIVE_CS], GPIO_PIN_SET);
}

void DisableDrive(DRIVE_MOTOR drive_index) {
// Désactive l'étage de puissance logiciellement
	drive_regs[drive_index].ctrl_reg.enbl = 0;

// Activer la communication SPI (CS = LOW)
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
			drive_pins[drive_index][DRIVE_CS], GPIO_PIN_RESET);

// Envoyer le registre CTRL avec ENBL = 0
//TransmitMotorSPI(drive_index, DRV8711_CTRL_REG);

// Terminer la communication SPI (CS = HIGH)
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
			drive_pins[drive_index][DRIVE_CS], GPIO_PIN_SET);
}

void ResetDrive(DRIVE_MOTOR drive_index) {
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_RESET],
			drive_pins[drive_index][DRIVE_RESET], GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_RESET],
			drive_pins[drive_index][DRIVE_RESET], GPIO_PIN_RESET);
}

void DriveSleep(DRIVE_MOTOR drive_index) {
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_SLEEP],
			drive_pins[drive_index][DRIVE_SLEEP], GPIO_PIN_RESET);
	drive_status[drive_index] = DRIVE_ASLEEP;
}

void DriveWakeUp(DRIVE_MOTOR drive_index) {
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_SLEEP],
			drive_pins[drive_index][DRIVE_SLEEP], GPIO_PIN_SET);
	drive_status[drive_index] = DRIVE_AWAKE;
}

uint32_t IsDriveAwake(DRIVE_MOTOR drive_index) {
	return drive_status[drive_index];
}

void SetDirection(DRIVE_MOTOR drive_index, uint32_t direction) {
	GPIO_PinState state = (direction == DIR_LEFT) ? 1 : 0;
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_DIR],
			drive_pins[drive_index][DRIVE_DIR], state);
}

void EnableDriveIndexer(DRIVE_MOTOR drive_index) {
	drive_regs[drive_index].off_reg.pwmmode = 0;		// Use internal indexer

	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
			drive_pins[drive_index][DRIVE_CS], GPIO_PIN_SET);
//TransmitMotorSPI(drive_index, DRV8711_OFF_REG);
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
			drive_pins[drive_index][DRIVE_CS], GPIO_PIN_RESET);
}

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
}

void EnableDriveExternalPWM(DRIVE_MOTOR drive_index) {
	drive_regs[drive_index].off_reg.pwmmode = 1;		// Use external PWM

	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
			drive_pins[drive_index][DRIVE_CS], GPIO_PIN_SET);
//TransmitMotorSPI(drive_index, DRV8711_OFF_REG);
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
			drive_pins[drive_index][DRIVE_CS], GPIO_PIN_RESET);
}

void DEBUG_SPI(DRIVE_MOTOR drive_index) {
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

void DEBUG_SPI_CHATGPT(DRIVE_MOTOR drive_index) {
// Sélectionner le DRV8711 (CS = LOW)
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
			drive_pins[drive_index][DRIVE_CS], GPIO_PIN_RESET);

// Étape 1 : envoyer la commande de lecture du registre 0x07 (STATUS)
	uint8_t tx_cmd[2] = { 0xF0, 0x00 };  // 0xF000 = lecture du registre 0x07
	uint8_t rx_dummy[2] = { 0 };

	HAL_SPI_TransmitReceive(hspi, tx_cmd, rx_dummy, 2, HAL_MAX_DELAY);

// Désélectionner momentanément (important selon le timing du DRV8711)
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
			drive_pins[drive_index][DRIVE_CS], GPIO_PIN_SET);
	HAL_Delay(1); // Petit délai si nécessaire

// Réactiver CS pour lire la vraie donnée
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
			drive_pins[drive_index][DRIVE_CS], GPIO_PIN_RESET);

	uint8_t tx_dummy[2] = { 0x00, 0x00 };
	uint8_t rx_data[2] = { 0 };

	HAL_SPI_TransmitReceive(hspi, tx_dummy, rx_data, 2, HAL_MAX_DELAY);

// Désélection finale du périphérique
	HAL_GPIO_WritePin(drive_ports[drive_index][DRIVE_CS],
			drive_pins[drive_index][DRIVE_CS], GPIO_PIN_SET);

// Recomposer les 16 bits reçus
	uint16_t result = (rx_data[0] << 8) | rx_data[1];

// Affichage debug via UART ou autre
	printf("DRV8711[%d] STATUS = 0x%04X\r\n", drive_index, result);

	if (result & (1 << 0))
		printf("  OCP: Overcurrent\n");
	if (result & (1 << 1))
		printf("  UVLO: Undervoltage\n");
	if (result & (1 << 2))
		printf("  OTS: Overtemp\n");
	if (result & (1 << 3))
		printf("  PDF: Power Fault\n");
	if (result & (1 << 4))
		printf("  STDLAT: Stall Latch\n");
}

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
// RSTEP : Step contrôlé par pin STEP si = 0, sinon interne
// Par défaut DRV8711 : 0

	drive_regs[drive_index].ctrl_reg.mode = 0b0000;
// MODE : Réglage du microstepping (0 = full step, jusqu’à 0b1111 = 1/256)
// Par défaut DRV8711 : 0b0000 (full step)

	drive_regs[drive_index].ctrl_reg.extstall = 0;
// EXTSTALL : 0 = Stall détecté en interne, 1 = via pin externe
// Par défaut DRV8711 : 0

	drive_regs[drive_index].ctrl_reg.isgain = 0b01;
// ISGAIN : Gain du senseur de courant (00 = 5, 01 = 10, 10 = 20, 11 = 40)
// Par défaut DRV8711 : 0b00 (gain de 5)

	drive_regs[drive_index].ctrl_reg.dtime = 0b00;
// DTIME : Dead time entre les switches du pont H (00 = 850ns)
// Par défaut DRV8711 : 0b00

//
// ───── TORQUE REGISTER (0x01) ───────────────────────────────────────────
// Détermine le courant de phase appliqué
//
//drive_regs[drive_index].torque_reg.torque = 24; // 100% de puissance avec isgain à 10
	drive_regs[drive_index].torque_reg.torque = 12; // 50% de puissance avec isgain à 10
// TORQUE : Niveau de couple (0–255), proportionnel au courant de sortie
// Par défaut DRV8711 : 0xFF (255)

	drive_regs[drive_index].torque_reg.smplth = 0b111;
// SMPLTH : Durée du seuil BEMF pour la détection de blocage
// Par défaut DRV8711 : 0b000

//
// ───── OFF REGISTER (0x02) ──────────────────────────────────────────────
// Temps d’arrêt de PWM et mode PWM
//
	drive_regs[drive_index].off_reg.toff = 0x80;
// TOFF : Temps mort (0 = désactivé, >0 = en pas de 500ns)
// Par défaut DRV8711 : 0x30 (24 = 12 µs)

	drive_regs[drive_index].off_reg.pwmmode = 1;
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
	drive_regs[drive_index].decay_reg.tdecay = 0x80;
// TDECAY : temps avant transition entre fast et slow decay (500ns steps)
// Par défaut DRV8711 : 0x10 (8 µs)

	drive_regs[drive_index].decay_reg.decmod = 0b000;
// DECMOD : 000 = slow decay forcé, 001 = fast, 010 = mixed decay
// Par défaut DRV8711 : 0b010 (mixed decay)

//
// ───── STALL REGISTER (0x05) ────────────────────────────────────────────
// Détection de blocage moteur (optionnel)
//
	drive_regs[drive_index].stall_reg.sdthr = 0xFF;
// SDTHR : Seuil de BEMF pour détecter un blocage
// Par défaut DRV8711 : 0x40

	drive_regs[drive_index].stall_reg.sdcnt = 0b11;
// SDCNT : nombre de pas requis avant détection (00 = 2, ..., 11 = 8 pas)
// Par défaut DRV8711 : 0b00

	drive_regs[drive_index].stall_reg.vdiv = 0b00;
// VDIV : Diviseur du signal BEMF pour détection (00 = /32)
// Par défaut DRV8711 : 0b00

//
// ───── DRIVE REGISTER (0x06) ────────────────────────────────────────────
// Réglage de la puissance des transistors MOSFET (gate drive)
//
	drive_regs[drive_index].drive_reg.ocpth = 0b11;
// OCPTH : Seuil de détection de surintensité (00 = 250mV, 11 = 2000mV)
// Par défaut DRV8711 : 0b01 (500mV)

	drive_regs[drive_index].drive_reg.ocpdeg = 0b11;
// OCPDEG : Temps de filtrage de la surintensité (00 = 1 µs, 11 = 8 µs)
// Par défaut DRV8711 : 0b00

	drive_regs[drive_index].drive_reg.tdriven = 0b11;
// TDRIVEN : Durée de l’impulsion LOW-SIDE (00 = 250ns, 11 = 500ns)
// Par défaut DRV8711 : 0b00

	drive_regs[drive_index].drive_reg.tdrivep = 0b11;
// TDRIVEP : Durée de l’impulsion HIGH-SIDE (00 = 250ns, 11 = 500ns)
// Par défaut DRV8711 : 0b00

	drive_regs[drive_index].drive_reg.idriven = 0b11;
// IDRIVEP : Courant de crête HIGH-SIDE (00 = 20mA, 11 = 150mA)
// Par défaut DRV8711 : 0b00
}

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

/*

 // PWM
 TIM_HandleTypeDef *pwm_timers[2] = { 0, 0 };
 uint32_t pwm_channels[2] = { 0, 0 };
 enum PWMs {
 PWM1 = 0, PWM2
 };

 TIM_HandleTypeDef *pwm1_timer;
 TIM_HandleTypeDef *pwm2_timer;
 uint32_t pwm1_channel;
 uint32_t pwm2_channel;

 void SetDutyCycle(uint32_t pwm_index, uint16_t duty_cycle) {
 HAL_TIM_PWM_Stop(pwm_timers[pwm_index], pwm_channels[pwm_index]); // stop generation of pwm

 TIM_OC_InitTypeDef sConfigOC;
 // (*htim).Init.Period = PWM_PERIOD; // set the period duration
 // HAL_TIM_PWM_Init(htim); // reinititialise with new period value

 sConfigOC.OCMode = TIM_OCMODE_PWM1;
 //sConfigOC.Pulse = (uint32_t)(duty_cycle * (float)PWM_PERIOD); // set the pulse duration
 sConfigOC.Pulse = duty_cycle;
 sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
 sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
 HAL_TIM_PWM_ConfigChannel(pwm_timers[pwm_index], &sConfigOC,
 pwm_channels[pwm_index]);

 //HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2); // start pwm generation
 }

 void DriveMastRight() {
 SetDutyCycle(PWM1, 360);
 SetDutyCycle(PWM2, 480);
 HAL_TIM_PWM_Start(pwm1_timer, pwm1_channel);
 HAL_TIM_PWM_Start(pwm2_timer, pwm2_channel);
 }

 void DriveMastLeft() {
 SetDutyCycle(PWM1, 480);
 SetDutyCycle(PWM2, 360);
 HAL_TIM_PWM_Start(pwm1_timer, pwm1_channel);
 HAL_TIM_PWM_Start(pwm2_timer, pwm2_channel);
 }

 void DriveMastStop() {
 SetDutyCycle(PWM1, 0);
 SetDutyCycle(PWM2, 0);
 HAL_TIM_PWM_Stop(pwm1_timer, pwm1_channel);
 HAL_TIM_PWM_Stop(pwm2_timer, pwm2_channel);
 }

 */
