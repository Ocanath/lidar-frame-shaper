#ifndef SMOOTHING_H
#define SMOOTHING_H

#include "sin-math.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ONE_BY_PI	0.318309886f

/*Wrapper for all variables used for motor smoothing*/
typedef struct smooth_mem_t
{
	uint32_t start_ts;
	float qd_start;
	float qd_end_prev;
	float freq;
	float offset;
	float prev_fp; //previous period or frequency term.
	float jump_trig;
}smooth_mem_t;

void smooth_qd(float qd_end, float period,
		float q,
		smooth_mem_t * sm,
		float * qd,
		uint32_t tick);

int init_smoothing_mem(smooth_mem_t * sm);

#ifdef __cplusplus
}
#endif

#endif
