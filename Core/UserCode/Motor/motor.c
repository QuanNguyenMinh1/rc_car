/*
 * motor.c
 *
 *  Created on: Jul 24, 2024
 *      Author: Quan
 */

#include "motor.h"
//extern PROCESS_t tprocess;

void motor_reset(Motor_t *tmotor)
{
    if (tmotor == NULL) {
        // Handle null pointer error
        return;
    }
	tmotor->iid = 0;
	tmotor->ipulse_per_round = 0;
	tmotor->ipulse_per_sampling = 0;
	tmotor->ipulse_per_second = 0;
	tmotor->icounter = 0;
	tmotor->ipre_counter = 0;
//	tmotor->inumber_of_pulse = 0;
	tmotor->fposition = 0;
	tmotor->fvelocity = 0;
	tmotor->fround_per_minute = 0;
	tmotor->fts = 0;
}

void motor_init(Motor_t *tmotor, uint8_t iid)
{
    if (tmotor == NULL) {
        // Handle null pointer error
        return;
    }
	motor_reset(tmotor); // truyen dia chi tmotor vo roi thi bay gio no ban than no da la dia chi roi nen ko can "&"
	tmotor->iid = iid;
    HAL_TIM_Base_Start_IT(&INTERRUPT_TIMER);
    switch (iid) {
		case 1:
		    HAL_TIM_Encoder_Start(&ENCODER_TIMER1, TIM_CHANNEL_1);
		    HAL_TIM_Encoder_Start(&ENCODER_TIMER1, TIM_CHANNEL_2);
	    	HAL_TIM_PWM_Start(&PWM_TIMER, TIM_CHANNEL_1);
	    	HAL_TIM_PWM_Start(&PWM_TIMER, TIM_CHANNEL_2);
//	    	HAL_TIM_PWM_Start(&PWM_TIMER, TIM_CHANNEL_3);
//	    	HAL_TIM_PWM_Start(&PWM_TIMER, TIM_CHANNEL_4);
			break;
		case 2:
	        HAL_TIM_Encoder_Start(&ENCODER_TIMER2, TIM_CHANNEL_1);
	        HAL_TIM_Encoder_Start(&ENCODER_TIMER2, TIM_CHANNEL_2);
	    	HAL_TIM_PWM_Start(&PWM_TIMER, TIM_CHANNEL_3);
	    	HAL_TIM_PWM_Start(&PWM_TIMER, TIM_CHANNEL_4);
//	    	HAL_TIM_PWM_Start(&PWM_TIMER, TIM_CHANNEL_1);
//	    	HAL_TIM_PWM_Start(&PWM_TIMER, TIM_CHANNEL_2);
			break;
    }
}

void motor_deinit(Motor_t *tmotor, uint8_t iid)
{
    if (tmotor == NULL) {
        // Handle null pointer error
        return;
    }
//    HAL_TIM_Base_Stop_IT(&INTERRUPT_TIMER);
    HAL_TIM_Base_Stop_IT(&PWM_TIMER);
    switch (iid) {
		case 1:
			HAL_TIM_PWM_Stop(&PWM_TIMER, TIM_CHANNEL_1);
			HAL_TIM_PWM_Stop(&PWM_TIMER, TIM_CHANNEL_2);
			MOTOR1_FORWARD_DUTY_CYCLE_REGISTER = 0;
			MOTOR1_BACKWARD_DUTY_CYCLE_REGISTER = 0;
			break;
		case 2:
			HAL_TIM_PWM_Stop(&PWM_TIMER, TIM_CHANNEL_3);
		    HAL_TIM_PWM_Stop(&PWM_TIMER, TIM_CHANNEL_4);
		    MOTOR2_FORWARD_DUTY_CYCLE_REGISTER = 0;
		    MOTOR2_BACKWARD_DUTY_CYCLE_REGISTER = 0;
			break;
	}
}

void motor_read_encoder(Motor_t *tmotor, TIM_HandleTypeDef *htim)
{
    if (tmotor == NULL || htim == NULL) {
        // Handle null pointer error
        return;
    }
	tmotor->icounter = htim->Instance->CNT;
	tmotor->fvelocity = 0.0f;
	if (tmotor->icounter - tmotor->ipre_counter > 0)
    {
    	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)) //1b
    	{
    		tmotor->ipulse_per_sampling = (tmotor->icounter - htim->Instance->ARR) - tmotor->ipre_counter;
			tmotor->fround_per_minute = ((float)tmotor->ipulse_per_sampling / SAMPLING_TIME) / 4.0f / (float)PPR * 60; // xungtrengiay / 4 / 330
			tmotor->fposition += (tmotor->ipulse_per_sampling / 4.0f) / (float)PPR * A_CIRCLE_IN_DEGREES; // doc pos chi can 1 xung de xac dinh thoi
//			tmotor->fvelocity = (tmotor->ipulse_per_sampling / 4.0f) / (float)PPR * A_CIRCLE_IN_DEGREES / SAMPLING_TIME;
//			tmotor->fts += SAMPLING_TIME;
//			tmotor->fts += SAMPLING_TIME * 4.0f;

//			tmotor->fvelocity = tmotor->fposition / tmotor->fts / 4.0f;
			tmotor->fvelocity = tmotor->fposition / SAMPLING_TIME;
//			tmotor->fts += SAMPLING_TIME;

    	}
    	else //1a
    	{
    		tmotor->ipulse_per_sampling = tmotor->icounter - tmotor->ipre_counter;
    		tmotor->fround_per_minute = ((float)tmotor->ipulse_per_sampling / SAMPLING_TIME) / 4.0f / (float)PPR * 60; // xungtrengiay / 4 / 330
			tmotor->fposition += (tmotor->ipulse_per_sampling / 4.0f) / (float)PPR * A_CIRCLE_IN_DEGREES; // doc pos chi can 1 xung de xac dinh thoi
//			tmotor->fvelocity = (tmotor->ipulse_per_sampling / 4.0f) / (float)PPR * A_CIRCLE_IN_DEGREES / SAMPLING_TIME;
//			tmotor->fts += SAMPLING_TIME;
//			tmotor->fts += SAMPLING_TIME * 4.0f;

//			tmotor->fvelocity = tmotor->fposition / tmotor->fts / 4.0f;

			tmotor->fvelocity = tmotor->fposition / SAMPLING_TIME;
//			tmotor->fts += SAMPLING_TIME;

    	}
    }
    else if (tmotor->icounter - tmotor->ipre_counter < 0) //2a
    {
    	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
    	{
    		tmotor->ipulse_per_sampling = tmotor->icounter - tmotor->ipre_counter;
    		tmotor->fround_per_minute = ((float)tmotor->ipulse_per_sampling / SAMPLING_TIME) / 4.0f / (float)PPR * 60; // xungtrengiay / 4 / 330
			tmotor->fposition += (tmotor->ipulse_per_sampling / 4.0f) / (float)PPR * A_CIRCLE_IN_DEGREES; // doc pos chi can 1 xung de xac dinh thoi
//			tmotor->fvelocity = (tmotor->ipulse_per_sampling / 4.0f) / (float)PPR * A_CIRCLE_IN_DEGREES / SAMPLING_TIME;
//			tmotor->fts += SAMPLING_TIME;
//			tmotor->fts += SAMPLING_TIME * 4.0f;

//			tmotor->fvelocity = tmotor->fposition / tmotor->fts / 4.0f;
			tmotor->fvelocity = tmotor->fposition / SAMPLING_TIME;
//			tmotor->fts += SAMPLING_TIME;

    	}
    	else //2b
    	{
    		tmotor->ipulse_per_sampling = (tmotor->icounter + htim->Instance->ARR) - tmotor->ipre_counter;
    		tmotor->fround_per_minute = ((float)tmotor->ipulse_per_sampling / SAMPLING_TIME) / 4.0f / (float)PPR * 60; // xungtrengiay / 4 / 330
			tmotor->fposition += (tmotor->ipulse_per_sampling / 4.0f) / (float)PPR * A_CIRCLE_IN_DEGREES; // doc pos chi can 1 xung de xac dinh thoi
//			tmotor->fvelocity = (tmotor->ipulse_per_sampling / 4.0f) / (float)PPR * A_CIRCLE_IN_DEGREES / SAMPLING_TIME;
//			tmotor->fts += SAMPLING_TIME;
//			tmotor->fts += SAMPLING_TIME * 4.0f;

//			tmotor->fvelocity = tmotor->fposition / tmotor->fts / 4.0f;

			tmotor->fvelocity = tmotor->fposition / SAMPLING_TIME;
//			tmotor->fts += SAMPLING_TIME;

    	}
    }
    else //3
    {
    	tmotor->ipulse_per_sampling = 0;
    }

//	tmotor->fts += SAMPLING_TIME;

	tmotor->ipre_counter = tmotor->icounter;
}

void motor_set_duty(Motor_t *tmotor, int32_t iduty)
{
	if (iduty >=0)
	{
		if (tmotor->iid == MOTOR1)
		{
			MOTOR1_FORWARD_DUTY_CYCLE_REGISTER = iduty;
			MOTOR1_BACKWARD_DUTY_CYCLE_REGISTER = 0;
		}
		else
		{
			MOTOR2_FORWARD_DUTY_CYCLE_REGISTER = iduty;
			MOTOR2_BACKWARD_DUTY_CYCLE_REGISTER = 0;
		}
	}
	else
	{
		if (tmotor->iid == MOTOR1)
		{
			MOTOR1_FORWARD_DUTY_CYCLE_REGISTER = 0;
			MOTOR1_BACKWARD_DUTY_CYCLE_REGISTER = (-iduty);
		}
		else
		{
			MOTOR2_FORWARD_DUTY_CYCLE_REGISTER = 0;
			MOTOR2_BACKWARD_DUTY_CYCLE_REGISTER = (-iduty);
		}
	}
}

void motor_set_velocity(Motor_t *tmotor, PID_CONTROL_t *tpid_ctrl, float fvelocity)
{
	if (tmotor == NULL || tpid_ctrl == NULL)
	{
		return;
	}

	tmotor->freference_velocity = fvelocity;
	motor_set_duty(tmotor, (int)pid_compute(tpid_ctrl, tmotor->freference_velocity, tmotor->fvelocity));
}

void motor_set_position(Motor_t *tmotor, PID_CONTROL_t *tpid_ctrl, float fposition)
{
    if (tmotor == NULL || tpid_ctrl == NULL)
    {
        // Handle null pointer error
        return;
    }

    tmotor->freference_position = fposition;
    motor_set_duty(tmotor, (int)pid_compute(tpid_ctrl, tmotor->freference_position, tmotor->fposition));
}
