/*
 * can.h
 *
 *  Created on: Aug 14, 2025
 *      Author: thoma
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "stm32f4xx_hal.h"

void CanInit();

extern uint8_t flag_can_direction;
extern uint8_t flag_can_speed;
extern uint32_t can_motor_pitch_speed; //0-100%
extern uint32_t can_motor_pitch_direction;

HAL_StatusTypeDef TransmitCAN(uint32_t id, uint8_t* buf, uint8_t size, uint8_t with_priority);

#endif /* INC_CAN_H_ */
