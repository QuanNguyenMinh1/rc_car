/*
 * motion.c
 *
 *  Created on: Jul 25, 2024
 *      Author: Quan
 */


#include "motion.h"

float dPosTemp = 0;
float g_dCmdVel = 0;
float dAccelTemp = 0;
uint8_t is_running = 0;
extern Motor_t tmotor1;
extern Motor_t tmotor2;
extern PROCESS_t tprocess;
extern uint8_t pid_init_flag;
extern uint8_t move_backward;
extern uint8_t state;
extern uint8_t flag_turn;
extern uint8_t state_ready;

// Profile Trapezoidal Speed
void profile_init(PROFILE_t *tProfile, uint8_t iid, float dAccelMax, float dVelMax, float dPosMax)
{
	tProfile->iid = iid;

	tProfile->dAccelMax = dAccelMax;
	tProfile->dVelMax = dVelMax;
	tProfile->dPosMax = dPosMax;
	tProfile->dA1 = 0.5f * tProfile->dAccelMax;
	tProfile->dA2 = tProfile->dVelMax;
	tProfile->dB2 = -0.5f * tProfile->dVelMax * tProfile->dVelMax / tProfile->dAccelMax;
	tProfile->dA3 = -0.5f * tProfile->dAccelMax;
	tProfile->dB3 = tProfile->dPosMax * tProfile->dAccelMax / tProfile->dVelMax + tProfile->dVelMax;
	tProfile->dC3 = -0.5f * tProfile->dPosMax * tProfile->dPosMax * tProfile->dAccelMax / (tProfile->dVelMax * tProfile->dVelMax) - 0.5f * tProfile->dVelMax * tProfile->dVelMax / tProfile->dAccelMax;
	tProfile->dMidStep1 = tProfile->dVelMax / tProfile->dAccelMax;
	tProfile->dMidStep2 = tProfile->dPosMax / tProfile->dVelMax;
	tProfile->dMidStep3 = tProfile->dMidStep1 + tProfile->dMidStep2;

	tProfile->nTime = 0;

    tProfile->dt1 = tProfile->dVelMax / tProfile->dAccelMax;
    tProfile->dt2 = 2 * tProfile->dt1;
    tProfile->dt3 = tProfile->dPosMax / tProfile->dVelMax;
    tProfile->dt4 = tProfile->dt3 + tProfile->dt1;
    tProfile->dtf = tProfile->dt3 + tProfile->dt2;
    tProfile->dj = tProfile->dAccelMax / tProfile->dt1;
    tProfile->iN = 100;

    tProfile->dt = 0;
}

void compute_trapezoidal(PROFILE_t *tProfile, PID_CONTROL_t *tpid_ctrl1, PID_CONTROL_t *tpid_ctrl2)
{
	dPosTemp = 0;
	g_dCmdVel = 0;

	if (tProfile->nTime <= tProfile->dMidStep1)
	{
		dPosTemp = (int32_t)(tProfile->dA1 * tProfile->nTime * tProfile->nTime);
		g_dCmdVel = 2 * tProfile->dA1 * tProfile->nTime;
	}
	else if (tProfile->nTime <= tProfile->dMidStep2)
	{
		dPosTemp = (int32_t)(tProfile->dA2 * tProfile->nTime + tProfile->dB2);
		g_dCmdVel = tProfile->dA2;
	}
	else if(tProfile->nTime <= tProfile->dMidStep3)
	{
		dPosTemp = (int32_t)(tProfile->dA3 * tProfile->nTime * tProfile->nTime + tProfile->dB3 * tProfile->nTime + tProfile->dC3);
		g_dCmdVel = 2 * tProfile->dA3 * tProfile->nTime + tProfile->dB3;
	}
	else
	{
		dPosTemp = tProfile->dPosMax;
	}
	/* CONTROL PID */
	if (g_dCmdVel != 0)
	{
		is_running = 1;
	}
	if (g_dCmdVel == 0 && is_running == 1)
	{
		is_running = 0;
		pid_init_flag = 0;
		tprocess = STOP;
	}

	 motor_set_velocity(&tmotor1, tpid_ctrl1, g_dCmdVel);
	 motor_set_velocity(&tmotor2, tpid_ctrl2, g_dCmdVel);


	tProfile->nTime += SAMPLING_TIME;
}

void compute_scurve(PROFILE_t *tProfile, PID_CONTROL_t *tpid_ctrl1, PID_CONTROL_t *tpid_ctrl2) // thêm 2 tham số PID_CONTROL_t *tpid_ctrl1, PID_CONTROL_t *tpid_ctrl2 để tun vận tốc)
{
	g_dCmdVel = 0;
	dPosTemp = 0;
	dAccelTemp = 0;
    if (tProfile->dVelMax <= sqrt(tProfile->dPosMax * tProfile->dAccelMax / 2))
    {

        // create_vector(tProfile->dt, tProfile->dtf, tProfile->iN);
//        for (int i = 0; i < tProfile->iN; i++)
//        {
            if (tProfile->dt <= tProfile->dt1)
            {
                dPosTemp = tProfile->dj * tProfile->dt * tProfile->dt * tProfile->dt / 6;
                g_dCmdVel  = tProfile->dj * tProfile->dt * tProfile->dt / 2;
                dAccelTemp = tProfile->dj * tProfile->dt;
            }
            else if (tProfile->dt <= tProfile->dt2)
            {
                dPosTemp = tProfile->dj * tProfile->dt1 * tProfile->dt1 * tProfile->dt1 / 6 + tProfile->dj * tProfile->dt1 * tProfile->dt1 / 2 * (tProfile->dt-tProfile->dt1) + tProfile->dAccelMax * (tProfile->dt - tProfile->dt1) * (tProfile->dt - tProfile->dt1) / 2 - tProfile->dj * (tProfile->dt - tProfile->dt1) * (tProfile->dt - tProfile->dt1) * (tProfile->dt - tProfile->dt1) / 6;
                g_dCmdVel = tProfile->dj * tProfile->dt1 * tProfile->dt1 / 2 + tProfile->dAccelMax * (tProfile->dt - tProfile->dt1) - tProfile->dj * (tProfile->dt - tProfile->dt1) * (tProfile->dt - tProfile->dt1) / 2;
                dAccelTemp = tProfile->dAccelMax - tProfile->dj * (tProfile->dt - tProfile->dt1);
            }
            else if (tProfile->dt <= tProfile->dt3)
            {
            	dPosTemp = tProfile->dAccelMax * tProfile->dt1 * tProfile->dt1 + tProfile->dVelMax * (tProfile->dt - tProfile->dt2);
            	g_dCmdVel = tProfile->dVelMax;
            	dAccelTemp = 0;
            }
            else if (tProfile->dt <= tProfile->dt4)
            {
            	dPosTemp = tProfile->dAccelMax * tProfile->dt1 * tProfile->dt1 + tProfile->dVelMax * (tProfile->dt3 - tProfile->dt2) + tProfile->dVelMax * (tProfile->dt - tProfile->dt3) - tProfile->dj * (tProfile->dt - tProfile->dt3) * (tProfile->dt - tProfile->dt3) * (tProfile->dt - tProfile->dt3) / 6;
            	g_dCmdVel = tProfile->dVelMax - tProfile->dj * (tProfile->dt - tProfile->dt3) * (tProfile->dt - tProfile->dt3) / 2;
            	dAccelTemp = -tProfile->dj * (tProfile->dt - tProfile->dt3);
            }
            else if (tProfile->dt <= tProfile->dtf)
            {
            	dPosTemp = tProfile->dPosMax - tProfile->dj * (tProfile->dtf - tProfile->dt) * (tProfile->dtf - tProfile->dt) * (tProfile->dtf - tProfile->dt) / 6;
            	g_dCmdVel  = tProfile->dVelMax - tProfile->dj * (tProfile->dt4 - tProfile->dt3) * (tProfile->dt4 - tProfile->dt3) / 2 - tProfile->dAccelMax * (tProfile->dt - tProfile->dt4) + tProfile->dj * (tProfile->dt - tProfile->dt4) * (tProfile->dt - tProfile->dt4) / 2;
            	dAccelTemp = -tProfile->dAccelMax + tProfile->dj * (tProfile->dt - tProfile->dt4);
            }
//        }
    }
	/* CONTROL PID */
    if (tProfile->iid == 1)
    {
    	motor_set_velocity(&tmotor1, tpid_ctrl1, g_dCmdVel);
    	tProfile->dt += SAMPLING_TIME; // same as tProfile->nTime
    }
    else if (tProfile->iid == 2)
    {
    	motor_set_velocity(&tmotor2, tpid_ctrl2, g_dCmdVel);
    	tProfile->dt += SAMPLING_TIME; // same as tProfile->nTime
    }
}
