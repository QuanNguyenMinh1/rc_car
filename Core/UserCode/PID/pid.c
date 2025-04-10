/*
 * pid.c
 *
 *  Created on: Jul 25, 2024
 *      Author: Quan
 */

#include "pid.h"

void pid_reset(PID_CONTROL_t *tpid_ctrl)
{
	if (tpid_ctrl == NULL) {
		return;
	}
	tpid_ctrl->fkp = 0.0f;
	tpid_ctrl->fki = 0.0f;
	tpid_ctrl->fkd = 0.0f;

	tpid_ctrl->flim_max_int = 0.0f;
	tpid_ctrl->flim_min_int = 0.0f;

	tpid_ctrl->ferror = 0.0f;
	tpid_ctrl->fpre_error = 0.0f;

	tpid_ctrl->fproportional = 0.0f;
	tpid_ctrl->fintergral = 0.0f;
	tpid_ctrl->fderivative = 0.0f;

	tpid_ctrl->fresult = 0.0f;
}

void pid_init(PID_CONTROL_t *tpid_ctrl, float fkp, float fki, float fkd, float flim_max, float flim_min, float fts)
{
	if (tpid_ctrl == NULL || fkp < 0.0f || fki < 0.0f || fkd < 0.0f || flim_max < flim_min || fts < 0.0f)
	{
		return;
	}
	pid_reset(tpid_ctrl);
	tpid_ctrl->fkp = fkp;
	tpid_ctrl->fki = fki;
	tpid_ctrl->fkd = fkd;
	tpid_ctrl->flim_max = flim_max;
	tpid_ctrl->flim_min = flim_min;
	tpid_ctrl->fts = fts;
}

void pid_deinit(PID_CONTROL_t *tpid_ctrl)
{
	tpid_ctrl->fkp = 0;
	tpid_ctrl->fki = 0;
	tpid_ctrl->fkd = 0;
}

void pid_tunning_set(PID_CONTROL_t *tpid_ctrl, float fkp, float fki, float fkd)
{
	 if (tpid_ctrl == NULL || fkp < 0.0f || fki < 0.0f || fkd < 0.0f) {
	        // Handle invalid parameters or null pointer error
	        return;
	}

    tpid_ctrl->fkp = fkp;
    tpid_ctrl->fki = fki;
    tpid_ctrl->fkd = fkd;
}

float pid_compute(PID_CONTROL_t *tpid_ctrl, float fcmd_value, float fact_value)
{
   if (tpid_ctrl == NULL) {
		// Handle null pointer error
		return 0.0f; // or any default value indicating an error
	}

    // Calculate error value
    tpid_ctrl->ferror = fcmd_value - fact_value;

    // P part
    tpid_ctrl->fproportional = tpid_ctrl->fkp * tpid_ctrl->ferror;

    // I part
    tpid_ctrl->fintergral += 0.5f * tpid_ctrl->fki * tpid_ctrl->fts * (tpid_ctrl->ferror + tpid_ctrl->fpre_error);

    // Integrator Anti-windup

    // Update integral Limits
    if (tpid_ctrl->flim_max > tpid_ctrl->fproportional)
    {
        tpid_ctrl->flim_max_int = tpid_ctrl->flim_max - tpid_ctrl->fproportional;
    }
    else
    {
        tpid_ctrl->flim_max_int = 0.0f;
    }
    if (tpid_ctrl->flim_min < tpid_ctrl->fproportional)
    {
        tpid_ctrl->flim_min_int = tpid_ctrl->flim_min - tpid_ctrl->fproportional;
    }
    else
    {
        tpid_ctrl->flim_min_int = 0.0f;
    }
    // Apply integral limits
    if (tpid_ctrl->fintergral > tpid_ctrl->flim_max_int)
    {
        tpid_ctrl->fintergral = tpid_ctrl->flim_max_int;
    }
    else if (tpid_ctrl->fintergral < tpid_ctrl->flim_min_int)
    {
        tpid_ctrl->fintergral = tpid_ctrl->flim_min_int;
    }
    // D part
    tpid_ctrl->fderivative = 2.0f * tpid_ctrl->fkd / tpid_ctrl->fts * (tpid_ctrl->ferror - tpid_ctrl->fpre_error) - (tpid_ctrl->fderivative);

    // Compute output and apply limits
    tpid_ctrl->fresult = tpid_ctrl->fproportional + tpid_ctrl->fintergral + tpid_ctrl->fderivative;

    if (tpid_ctrl->fresult > tpid_ctrl->flim_max)
    {
        tpid_ctrl->fresult = tpid_ctrl->flim_max;
    }
    else if (tpid_ctrl->fresult < tpid_ctrl->flim_min)
    {
        tpid_ctrl->fresult = tpid_ctrl->flim_min;
    }

    // Update pre-error
    tpid_ctrl->fpre_error = tpid_ctrl->ferror;

    return tpid_ctrl->fresult;
}
