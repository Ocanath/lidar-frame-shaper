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
	lidar.connect(2381);

	Motor m(0); 	//dartt addr = 0

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
			//write
		}
		
	}
	
	return 0;
}
