#include "motor.h"


Motor::Motor(unsigned char addr, Serial * ser)
{

	//iniialize the motor
	ds.address = addr;	//must be mapped
	
	/*TODO: consider allocating heap? */
	ds.ctl_base.buf = (unsigned char *)(&dp_ctl);	//must be assigned
	ds.ctl_base.size = sizeof(dartt_mctl_params_t);
	ds.periph_base.buf = (unsigned char *)(&dp_periph);	//must be assigned
	ds.periph_base.size = sizeof(dartt_mctl_params_t);
	for(int i = 0; i < sizeof(dartt_mctl_params_t); i++)
	{
		ds.ctl_base.buf[i] = 0;
		ds.periph_base.buf[i] = 0;
	}

	ds.base_offset = 0;
	ds.msg_type = TYPE_SERIAL_MESSAGE;
	ds.tx_buf.buf = tx_buf_mem;
	ds.tx_buf.size = sizeof(tx_buf_mem) - NUM_BYTES_COBS_OVERHEAD;		//DO NOT CHANGE. This is for a good reason. See above note
	ds.tx_buf.len = 0;
	ds.rx_buf.buf = rx_buf_mem;
	ds.rx_buf.size = sizeof(rx_buf_mem) - NUM_BYTES_COBS_OVERHEAD;	//DO NOT CHANGE. This is for a good reason. See above note
	ds.rx_buf.len = 0;
	ds.blocking_tx_callback = &tx_blocking;
	ds.user_context_tx = (void*)(ser);
	ds.blocking_rx_callback = &rx_blocking;
	ds.user_context_rx = (void*)(ser);
	ds.timeout_ms = 10;


		//TODO: make this member function of the motor class
	read_slice = 
	{
		.buf = (unsigned char *)(&dp_ctl.theta_rem_m),
		.size = sizeof(uint32_t) * 3,
		.len = sizeof(uint32_t) * 3
	};
	write_slice = {
		.buf = (unsigned char *)(&dp_ctl.command_word),
		.size = sizeof(dp_ctl.command_word),
		.len = sizeof(dp_ctl.command_word)
	};
}

int Motor::read_data(void)
{
	int rc = dartt_read_multi(&read_slice, &ds);
	if(rc != DARTT_PROTOCOL_SUCCESS)
	{
		return rc;
	}
	//unit convert stuff
	q = ((float)dp_periph.theta_rem_m)/((float)(1<<14));
	qdot = ((float)dp_periph.dtheta_fixedpoint_rad_p_sec)/16.f;
	iq = (float)dp_periph.iq;
	return rc;
}

int Motor::write_data(void)
{
	int32_t radians_digital = (int32_t)(qd * ((float)(1<<14)));
	dp_ctl.command_word = wrap_2pi_14b(radians_digital);	//redundant cus controller does this but why not
	//unit convert stuff
	return dartt_write_multi(&write_slice, &ds);
}


int Motor::rezero(void)
{	
	dartt_buffer_t unwrappedangle = {
		.buf = (unsigned char *)(&dp_ctl.unwrap_state.unwrapped_angle),
		.size = sizeof(dp_ctl.unwrap_state.unwrapped_angle),
		.len = sizeof(dp_ctl.unwrap_state.unwrapped_angle)
	};
	int64_t avg_angle = 0;
	for(int i = 0; i < 10; i++)
	{
		int rc = dartt_read_multi(&unwrappedangle, &ds);	//get unwrapped angle value
		if(rc != DARTT_PROTOCOL_SUCCESS)
		{
			return rc;
		}
		avg_angle += dp_periph.unwrap_state.unwrapped_angle;
	}
	avg_angle /= 10;
	dp_ctl.theta_offset = avg_angle;
	dartt_buffer_t thetaoffset = {
		.buf = (unsigned char *)(&dp_ctl.theta_offset),
		.size = sizeof(dp_ctl.theta_offset),
		.len = sizeof(dp_ctl.theta_offset)
	};
	return dartt_write_multi(&thetaoffset, &ds);
}
