#ifndef MOTOR_H
#define MOTOR_H

#include "dartt_mctl_params.h"
#include <vector>
#include "dartt_sync.h"
#include "tinycsocket.h"
#include "dartt_init.h"

class Motor
{
public:
	dartt_mctl_params_t dp_ctl;
	dartt_mctl_params_t dp_periph;
	dartt_sync_t ds;
	UdpState socket;

	Motor(unsigned char addr);
	~Motor();

	Motor(Motor&& other) noexcept;
	Motor& operator=(Motor&& other) noexcept;
	Motor(const Motor&) = delete;
	Motor& operator=(const Motor&) = delete;

	bool write_zero_offset(void);
};

#endif