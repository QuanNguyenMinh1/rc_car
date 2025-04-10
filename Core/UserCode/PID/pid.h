/*
 * pid.h
 *
 *  Created on: Jul 25, 2024
 *      Author: Quan
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "stdint.h"
#include <stdlib.h>

typedef struct
{
    // Controller gains
    float fkp;
    float fki;
    float fkd;

    // Output limits
    float flim_min;
    float flim_max;

    // Intergral limits
    float flim_max_int;
    float flim_min_int;

    // Sampling time (in seconds)
    float fts;

    // Controller memory
    float ferror;
    float fpre_error;

    // P part, I part, D part
    float fproportional;
    float fintergral;
    float fderivative;

    // Controller output
    float fresult;

} PID_CONTROL_t;

void pid_reset(PID_CONTROL_t *tpid_ctrl);
void pid_init(PID_CONTROL_t *tpid_ctrl, float fkp, float fki, float fkd, float flim_max, float flim_min, float fts);
void pid_tunning_set(PID_CONTROL_t *tpid_ctrl, float fkp, float fki, float fkd);
float pid_compute(PID_CONTROL_t *tpid_ctrl, float fcmd_value, float fact_value);

#endif /* INC_PID_H_ */
