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

	ds.msg_type = TYPE_SERIAL_MESSAGE;

	ds.tx_buf.buf = tx_buf_mem;
	ds.tx_buf.size = SERIAL_BUFFER_SIZE - NUM_BYTES_COBS_OVERHEAD;		//DO NOT CHANGE. This is for a good reason. See above note
	ds.tx_buf.len = 0;
	ds.rx_buf.buf = rx_buf_mem;
	ds.rx_buf.size = SERIAL_BUFFER_SIZE - NUM_BYTES_COBS_OVERHEAD;	//DO NOT CHANGE. This is for a good reason. See above note
	ds.rx_buf.len = 0;
	ds.blocking_tx_callback = &tx_blocking;	//todo - figure something out here, cus we can't use the same socket...
	ds.user_context_tx = (void*)(ser);
	ds.blocking_rx_callback = &rx_blocking;
	ds.user_context_rx = (void*)(ser);
	ds.timeout_ms = 10;
}

