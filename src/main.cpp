#include <cstdio>

#define TINYCSOCKET_IMPLEMENTATION

// Platform headers (must come before GL on Windows)
#ifdef _WIN32
#define NOMINMAX
#include <winsock2.h>
#include <windows.h>
#endif

// tinycsocket (must come before SDL - SDL redefines main to SDL_main)
#include "tinycsocket.h"

// byte-stuffing
#include "cobs.h"

// dartt-protocol
#include "dartt.h"
#include "dartt_sync.h"
#include "checksum.h"

#include "dartt_init.h"

#include <Eigen/Dense>

#include <algorithm>
#include <string>
#include "dartt_mctl_params.h"
#include "motor.h"
#include "serial.h"
#include "lidar.h"

int main(int argc, char* argv[])
{
	(void)argc;
	(void)argv;

	Serial serial;
	bool connected = serial.autoconnect(921600);
	if(connected == false)
	{
		printf("Error: failed to connect to a serial port\n");
	}

	LidarSystem lidar; 
	bool lidar_connected = lidar.connect(2381);
	if(lidar_connected)
	{
		printf("Bind success to lidar port - ready for data\n");
	}

	Motor m(0, &serial); 	//dartt addr = 0

	bool running = true;
	while (running)
	{


		//TODO: make this member function of the motor class
		dartt_buffer_t r = {
			.buf = m.ds.ctl_base.buf,
			.size = sizeof(uint32_t) * 4,
			.len = sizeof(uint32_t) * 4
		};
		int dartt_rc = dartt_read_multi(&r, &m.ds);
		if(dartt_rc != DARTT_PROTOCOL_SUCCESS)
		{
			// printf("dartt read error %d\n", dartt_rc);
		}
		else
		{
			printf("%f\n", (float)m.dp_periph.theta_rem_m *180.f / ((float)(1<<14)) );
			//write
		}

		if (lidar.pollNewFrame())
		{
			printf("lidar timestamp: %u us\n", lidar.latestTimestamp());
		}
		
	}
	
	return 0;
}
