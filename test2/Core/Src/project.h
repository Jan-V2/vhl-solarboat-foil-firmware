/*
 * project.h
 *
 *  Created on: Jun 16, 2021
 *      Author: john
 *  TODO fix current read
 *
 */

#ifndef SRC_PROJECT_H_
#define SRC_PROJECT_H_

 #ifdef __cplusplus
 #define EXTERNC extern "C"
 #else
 #define EXTERNC
 #endif

#ifdef __cplusplus
extern "C" {
#endif
	#include "main.h"
	#include "canlib/mcp2515.h"
	#include "DualVNH5019MotorShield/DualVNH5019MotorShield.h"
	#include "motor_config.h"
#ifdef __cplusplus
}
#endif

uint8_t uart2_rcv_buff[32];
uint8_t uart2_snd_buff[32];

//EXTERNC void compute_pid(Motor m);
//EXTERNC int sample_motor_current(Motor m);
EXTERNC void home_actuators();
EXTERNC void user_while(void);
EXTERNC void user_init(void);
EXTERNC void encoder1_ISR(void);
EXTERNC void encoder2_ISR(void);


#endif /* SRC_PROJECT_H_ */
