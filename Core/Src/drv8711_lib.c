/*
 * motor_lib.c
 *
 *  Created on: Aug 13, 2025
 *      Author: thoma
 */
#include <stdio.h>
#include <string.h>

//Gate Charge Total (Qg)
#define Qg 14 //nC (nano-Coulombs)
//value for the CSD88537ND FET : 14nC

//Desired gate-charge time (RT)
//the time to charge the FET’s gate (gate-voltage rise time)
#define RT 100 //ns
//pick a value


//motor configuration

//1000 for a nema17HS08 or nema17HS19-2004S1
#define STEP_ANGLE 1.8 //in degree
#define NBR_STEP_ONE_TURN 200 //(360/STEP_ANGLE)

#define MAX_RPM 1000 //in RPM
#define MAX_RPS 16 //MAX_RPM / 60 in RPS
#define MAX_HZ 3200 //MAX_RPS * NBR_STEP_ONE_TURN in Hz
#define MIN_DELAY 312 //1 / MAX_HZ in us
