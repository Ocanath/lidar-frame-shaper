/*
 * dartt_params.h
 *
 *  Created on: Sep 13, 2025
 *      Author: Ocanath Robotman
 */

#ifndef INC_DARTT_MCTL_PARAMS_H_
#define INC_DARTT_MCTL_PARAMS_H_
#include "profiles.h"
#include "pctl.h"
#include "trig_fixed.h"

typedef enum {FOC_MODE, SINUSOIDAL_MODE, PCTL_IQ, PCTL_VQ, OPEN_LOOP_MODE} control_mode_t;	//foc with velocity?

typedef struct dartt_mctl_params_t
{

	int32_t command_word;

	//new - feedback terms, in order of likely importance to the control
	int32_t theta_rem_m;	//position 32bit in 14bit ticks
	int32_t iq;	//q axis current
	int32_t dtheta_fixedpoint_rad_p_sec;	//velocity in fixedpoint rad/sec
	int32_t id;	//d axis current
	int32_t ia;	//phase current a
	int32_t ib;	//phase current b
	int32_t ic; //phase current c

	pctl_params_t mctl_iq;
	pctl_params_t mctl_vq;


	int32_t open_loop_vq;
	int32_t open_loop_vd;

	uint8_t unused_1;
	uint8_t use_uart_encoder;
	uint8_t control_mode;
	uint8_t led_state;

	fds_motor_params_t fds_mp;
	int32_t autocalibration_voltage;

	//flag section. consider using only one 32bit word and bit positions for this - current setup is quite wasteful
	uint32_t load_action;	//when in main loop, set to trigger a scan of all dartt flags. Wrapper to save compute in commutation for excess flag handling
	uint32_t action_flag;	//commute motor

	unwrap_state_t unwrap_state;
	int32_t theta_offset;	//global offset applied to the rotor position

	uint32_t tick;	//millisecond tick
}dartt_mctl_params_t;

extern dartt_mctl_params_t gl_dp;

#endif /* INC_DARTT_MCTL_PARAMS_H_ */
