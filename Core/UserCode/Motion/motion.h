/*
 * motion.h
 *
 *  Created on: Jul 25, 2024
 *      Author: Quan
 */

#ifndef USERCODE_MOTION_MOTION_H_
#define USERCODE_MOTION_MOTION_H_

#include "stdint.h"
#include <stdlib.h>
#include "../PID/pid.h"
#include "../Motor/motor.h"
#include "../Serial/serial.h"

// trapezoidal profile structure
typedef struct
{
	float nTime;
	float dMidStep1;
	float dMidStep2;
	float dMidStep3;
	float dAccelMax;
	float dVelMax;
	float dPosMax;
	float dA1;
	float dA2;
	float dA3;
	float dB2;
	float dB3;
	float dC3;

	float dt;
    float dt1;
    float dt2;
    float dt3;
    float dt4;
    float dtf; // t final
    int iN;
    float dj;
    uint8_t iid;
} PROFILE_t;

void profile_init(PROFILE_t *tProfile, uint8_t iid, float dAccelMax, float dVelMax, float dPosMax);
void compute_trapezoidal(PROFILE_t *tProfile, PID_CONTROL_t *tpid_ctrl1, PID_CONTROL_t *tpid_ctrl2);
void compute_scurve(PROFILE_t *tProfile, PID_CONTROL_t *tpid_ctrl1, PID_CONTROL_t *tpid_ctrl2);


#endif /* USERCODE_MOTION_MOTION_H_ */
