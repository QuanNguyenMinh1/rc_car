/*
 * motor.h
 *
 *  Created on: Jul 24, 2024
 *      Author: Quan
 */

#ifndef USERCODE_MOTOR_MOTOR_H_
#define USERCODE_MOTOR_MOTOR_H_

#include <stdint.h>
#include <stdlib.h>
#include "../user_define.h"
#include "../UserCode/PID/pid.h"

//#include "main.h"

typedef struct
{
	uint8_t iid;
	uint32_t ipulse_per_round;
	uint32_t ipulse_per_second;
	int16_t ipulse_per_sampling;
	int16_t icounter;
	int16_t ipre_counter;

//	uint16_t ipulse_per_sampling;
//	uint16_t icounter;
//	uint16_t ipre_counter;

//	uint32_t ipulse_per_sampling;
//	uint32_t icounter;
//	uint32_t ipre_counter;
//	int16_t inumber_of_pulse; //x4
	float fround_per_minute;
	float fvelocity;
	float freference_velocity;
	float fposition;
	float freference_position;
	float fts;
} Motor_t;

void motor_reset(Motor_t *tmotor);
void motor_init(Motor_t *tmotor, uint8_t iid);
void motor_deinit(Motor_t *tmotor, uint8_t iid);
void motor_set_duty(Motor_t* tmotor, int32_t iduty);
void motor_read_encoder(Motor_t *tmotor, TIM_HandleTypeDef *htim);
void motor_set_velocity(Motor_t *tmotor, PID_CONTROL_t *tpid_ctrl, float fvelocity);
void motor_set_position(Motor_t *tmotor, PID_CONTROL_t *tpid_ctrl, float fposition);

#endif /* USERCODE_MOTOR_MOTOR_H_ */
