/*
 * user_define.h
 *
 *  Created on: Jul 24, 2024
 *      Author: Quan
 */
/*
 * user_define.h
 *
 *  Created on: Jul 24, 2024
 *      Author: Quan
 */
#include "tim.h"

#ifndef USERCODE_USER_DEFINE_H_
#define USERCODE_USER_DEFINE_H_

#define SAMPLING_TIME 0.01f //suffix f to indicate the number is float, else compiler will think it is a double variable
#define A_CIRCLE_IN_DEGREES 360.0f

//#define PPR 330 // xung X1 doc duoc cua JGB 520 333 RPM
#define PPR 210 // 1 PHA CHO 7 X 30 = 210 XUNG (CẢ LOW VÀ HIGH) => X1 = 105 (cua GA12 1:30)

#define ENCODER_TIMER1 htim4
#define ENCODER_TIMER2 htim1
#define INTERRUPT_TIMER htim3
#define PWM_TIMER htim2

#define PID_CONTROLLER_LIMIT_MAX htim2.Init.Period
#define PID_CONTROLLER_LIMIT_MIN -(float)(htim2.Init.Period)

#define MOTOR1_FORWARD_DUTY_CYCLE_REGISTER htim2.Instance->CCR1
#define MOTOR1_BACKWARD_DUTY_CYCLE_REGISTER htim2.Instance->CCR2
#define MOTOR2_FORWARD_DUTY_CYCLE_REGISTER htim2.Instance->CCR3
#define MOTOR2_BACKWARD_DUTY_CYCLE_REGISTER htim2.Instance->CCR4
#define MOTOR1 1
#define MOTOR2 2

#define MAX_LEN 100

#define UART_COM huart3
#define UART_COM_INSTANCE huart3.Instance

#define FIRHALF 0
#define SECHALF 1
#define PENDING 2
#define DONE 4
#define READ 5

#define NO 0
#define YES 1

#define FORWARD 1
#define BACKWARD 0

#endif /* USERCODE_USER_DEFINE_H_ */

//#include "tim.h"
//
//#ifndef USERCODE_USER_DEFINE_H_
//#define USERCODE_USER_DEFINE_H_
//
//#define SAMPLING_TIME 0.01f //suffix f to indicate the number is float, else compiler will think it is a double variable
//#define A_CIRCLE_IN_DEGREES 360.0f
//
////#define PPR 330 // xung X1 doc duoc cua JGB 520 333 RPM
//#define PPR 210 // 1 PHA CHO 7 X 30 = 210 XUNG (CẢ LOW VÀ HIGH) => X1 = 105 (cua GA12 1:30)
//
//#define ENCODER_TIMER1 htim4
//#define ENCODER_TIMER2 htim1
//#define INTERRUPT_TIMER htim3
//#define PWM_TIMER htim2
//
//#define PID_CONTROLLER_LIMIT_MAX htim2.Init.Period
//#define PID_CONTROLLER_LIMIT_MIN -(float)(htim2.Init.Period)
//
//#define MOTOR1_FORWARD_DUTY_CYCLE_REGISTER htim2.Instance->CCR1
//#define MOTOR1_BACKWARD_DUTY_CYCLE_REGISTER htim2.Instance->CCR2
//#define MOTOR2_FORWARD_DUTY_CYCLE_REGISTER htim2.Instance->CCR3
//#define MOTOR2_BACKWARD_DUTY_CYCLE_REGISTER htim2.Instance->CCR4
//#define MOTOR1 1
//#define MOTOR2 2
//
//#define MAX_LEN 100
//
//#define UART_COM huart3
//#define UART_COM_INSTANCE huart3.Instance
//
//#define FIRHALF 0
//#define SECHALF 1
//#define PENDING 2
//#define DONE 4
//#define READ 5
//
//#define NO 0
//#define YES 1
//
//#define FORWARD 1
//#define BACKWARD 0
//
//#endif /* USERCODE_USER_DEFINE_H_ */
