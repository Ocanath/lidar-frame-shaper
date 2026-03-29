#ifndef MOTOR_H
#define MOTOR_H

#include "dartt_mctl_params.h"
#include <vector>
#include "dartt_sync.h"
#include "serial_callbacks.h"
#include "tinycsocket.h"
#include "serial.h"

class Motor
{
public:
	dartt_mctl_params_t dp_ctl;
	dartt_mctl_params_t dp_periph;
	dartt_sync_t ds;
	unsigned char tx_buf_mem[SERIAL_BUFFER_SIZE];
	unsigned char rx_buf_mem[SERIAL_BUFFER_SIZE];

	Motor(unsigned char addr, Serial * ser);

	Motor(const Motor&) = delete;
	Motor& operator=(const Motor&) = delete;


	int read_data(void);
	int write_data(void);

private:
	dartt_buffer_t read_slice;
	dartt_buffer_t write_slice;

};

#endif