#include "motor.h"


Motor::Motor(unsigned char addr)
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

	ds.tx_buf.buf = new unsigned char[SERIAL_BUFFER_SIZE];
	ds.tx_buf.size = SERIAL_BUFFER_SIZE - NUM_BYTES_COBS_OVERHEAD;		//DO NOT CHANGE. This is for a good reason. See above note
	ds.tx_buf.len = 0;
	ds.rx_buf.buf = new unsigned char[SERIAL_BUFFER_SIZE];
	ds.rx_buf.size = SERIAL_BUFFER_SIZE - NUM_BYTES_COBS_OVERHEAD;	//DO NOT CHANGE. This is for a good reason. See above note
	ds.rx_buf.len = 0;
	ds.blocking_tx_callback = &tx_blocking;	//todo - figure something out here, cus we can't use the same socket...
	ds.user_context_tx = (void*)(&socket);
	ds.blocking_rx_callback = &rx_blocking;
	ds.user_context_rx = (void*)(&socket);
	ds.timeout_ms = 10;
}


Motor::~Motor()
{
	delete[] ds.tx_buf.buf;
	delete[] ds.rx_buf.buf;
	udp_disconnect(&socket);
}

Motor::Motor(Motor&& other) noexcept
    : dp_ctl(other.dp_ctl), dp_periph(other.dp_periph),
      ds(other.ds), socket(other.socket)
{
    ds.ctl_base.buf    = (unsigned char*)(&dp_ctl);
    ds.periph_base.buf = (unsigned char*)(&dp_periph);
    ds.user_context_tx = (void*)(&socket);
    ds.user_context_rx = (void*)(&socket);
    other.ds.tx_buf.buf = nullptr;
    other.ds.rx_buf.buf = nullptr;
    other.socket.socket    = TCS_SOCKET_INVALID;
    other.socket.connected = false;
}




Motor& Motor::operator=(Motor&& other) noexcept
{
    if (this == &other) return *this;
    delete[] ds.tx_buf.buf;
    delete[] ds.rx_buf.buf;
    udp_disconnect(&socket);
    dp_ctl = other.dp_ctl; dp_periph = other.dp_periph;
    ds = other.ds; socket = other.socket;
    ds.ctl_base.buf    = (unsigned char*)(&dp_ctl);
    ds.periph_base.buf = (unsigned char*)(&dp_periph);
    ds.user_context_tx = (void*)(&socket);
    ds.user_context_rx = (void*)(&socket);
    other.ds.tx_buf.buf = nullptr; other.ds.rx_buf.buf = nullptr;
    other.socket.socket = TCS_SOCKET_INVALID; other.socket.connected = false;
    return *this;
}

bool Motor::write_zero_offset(void)
{
	bool pass = true;

	dartt_buffer_t word = {
		.buf = (unsigned char *)(&dp_ctl.unwrap_state.unwrapped_angle),
		.size = sizeof(uint32_t)*4,
		.len = sizeof(int32_t)*4
	};
		
	int rc = dartt_read_multi(&word, &ds);
	if(rc != DARTT_PROTOCOL_SUCCESS)
	{
		pass = false;
	}
	else
	{
		dp_ctl.unwrap_state.unwrapped_angle = 0;	//clear any windup
		dp_ctl.theta_offset = wrap_2pi_fixed(dp_periph.unwrap_state.unwrapped_angle, TWO_PI_14B);	//
		rc = dartt_write_multi(&word, &ds);
		if(rc != DARTT_PROTOCOL_SUCCESS)
		{
			pass = false;
		}
	}
	return pass;
}