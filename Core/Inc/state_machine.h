/*
 * state_machine.h
 *
 *  Created on: Aug 14, 2025
 *      Author: thoma
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_

#include "stm32f4xx_hal.h"

extern uint8_t b_rops;
extern uint8_t b_emergency_stop;

void ExecuteStateMachine();

#endif /* INC_STATE_MACHINE_H_ */
