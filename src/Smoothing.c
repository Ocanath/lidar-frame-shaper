#include "Smoothing.h"
#include <math.h>

/**/
float abs_f(float input)
{
	if(input < 0)
	{
		return -input;
	}
	else
	{
		return input;
	}
}

int init_smoothing_mem(smooth_mem_t * sm)
{
	if(sm == NULL)
	{
		return 1;
	}
	sm->freq = 0.001f;
	sm->offset = 0.f;
	sm->prev_fp = 100.f;
	sm->qd_start = 0.f;
	sm->qd_end_prev = -1000.f;
	sm->jump_trig = 0.01;
	return 0;
}

/*
 * INPUTS:
 *
 * USER INPUTS:
 * qd_end: 		new setpoint, to approach in a smooth fashion
 * period: 		the time in seconds it takes for the finger to track its new position
 *
 * INPUT PARAMETERS
 * q:	 		finger position
 *
 *PASS BY POINTER/REFERENCE/HELPER VARIABLES
 * start_ts, qd_start, qd_end_prev, freq
 *
 *OUTPUTS
 *qd, true setpoint for position control
 *
 */
void smooth_qd(float qd_end, float period,
		float q,
		smooth_mem_t * sm,
		float * qd, uint32_t tick)
{
	float abs_diff = abs_f(qd_end - (sm->qd_end_prev));
	if(abs_diff > sm->jump_trig || (sm->prev_fp > 500.f && period < 500.f) )	//if the jump in setpoint value is large, or the previous period is > 500 and the current period is < 500
	{
		sm->start_ts = tick;	//reset the tracking time
		sm->offset = 0.0f;
		sm->qd_start = q;			//set the start point to our current position, (in setpoint coordinates!!!)
		sm->freq = PI/period;	//precalculate so we don't have to constantly divide
	}

	if(abs_f(period - sm->prev_fp) > 0.001f)	//if the previous period is not equal to the current period
	{
		sm->offset = ((float)(tick - sm->start_ts)*.001f)*(PI/(sm->prev_fp)) + sm->offset;
		sm->start_ts = tick;
		sm->freq = PI/period;	//if we get a new period, calculate the frequency for that period
	}

	float ft_st = ((float)(tick - sm->start_ts)*.001f)*sm->freq + sm->offset;
	float t = ft_st*period*ONE_BY_PI;

	if(t <= period)
		*qd = wrap_2pi(qd_end-sm->qd_start)*(.5f*(sinf(ft_st-HALF_PI)+1.0f)) + sm->qd_start;
	else
		*qd = qd_end;

	sm->qd_end_prev = qd_end;		//block entry into this routine again/update the previous value
	sm->prev_fp = period;
}

