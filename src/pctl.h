/*
 * pctl.h
 *
 *  Created on: Sep 13, 2025
 *      Author: Ocanath Robotman
 */

#ifndef INC_PCTL_H_
#define INC_PCTL_H_

typedef struct i32_t
{
	int32_t i32;
	int32_t radix;	//'decimal' point. true value is i32/2^radix
}i32_t;

//position control structure
typedef struct fixed_PI_2_params_t
{
	i32_t kp;
	i32_t ki;
	int32_t x_integral_div;
	int32_t x;
	int32_t x_sat;
	uint8_t out_rshift;
}fixed_PI_2_params_t;

//full position control structure
typedef struct pctl_params_t
{
	fixed_PI_2_params_t kpki;
	i32_t kd;
	int32_t out_sat;
}pctl_params_t;


#endif /* INC_PCTL_H_ */
