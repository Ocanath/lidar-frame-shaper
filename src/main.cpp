#include <cstdio>
#define DR_WAV_IMPLEMENTATION
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

#include "serial_callbacks.h"

#include <Eigen/Dense>

#include <algorithm>
#include <string>
#include "dartt_mctl_params.h"
#include "motor.h"
#include "serial.h"
#include "lidar.h"
#include "AudioWriter.h"
#include "tick.h"
#include "angle_buffer.h"
#include "udp_forwarder.h"
#include "Smoothing.h"


#define GOLDEN_ANGLE_RADIANS	2.39996322973f 

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

	AngleBuffer angleBuffer;

	UdpForwarder forwarder;
	forwarder.setDestination("127.0.0.1", 9000);
	forwarder.startWebServer(1050);

	LidarSystem lidar;
	lidar.angleBuffer   = &angleBuffer;
	lidar.onPacketReady = [&forwarder](const uint8_t* data, size_t len) {
		forwarder.send(data, len);
	};
	bool lidar_connected = lidar.connect(2381);
	if(lidar_connected)
	{
		printf("Bind success to lidar port - ready for data\n");
	}


	Motor m(0, &serial); 	//dartt addr = 0
	int writer_idx = index_of_field(&m.dp_ctl.audio, m.ds.ctl_base.buf, m.ds.ctl_base.size);
	if(writer_idx < 0)
	{
		printf("Catastrophic error - no writer present in dartt control structure\n");
		return writer_idx;
	}
	AudioWriter audio_writer(0, writer_idx, serial);
	if(audio_writer.play("assets/startupsound_processed.wav") != 0)
	{
		printf("No device present - exiting\n");
		return 1;
	}
	if(m.rezero() != DARTT_PROTOCOL_SUCCESS)
	{
		printf("Failed to rezero, exiting\n");
		return 1;
	}
	m.read_data();
	m.dp_ctl.command_word = 0;	
	m.write_data();

	m.dp_ctl.mctl_vq = {
		.kpki = {
			.kp = {
				.i32 = 400,
				.radix = 8
			},
			.ki = {
				.i32 = 3,
				.radix = 10
			},
			.x_integral_div = 10,
			.x_sat = 1000
		},
		.kd = {
			.i32 = 40,
			.radix = 5
		},
		.out_sat = 3546
	};
	dartt_buffer_t mctlvq = {
		.buf = (unsigned char *)(&m.dp_ctl.mctl_vq),
		.size = sizeof(m.dp_ctl.mctl_vq),
		.len = sizeof(m.dp_ctl.mctl_vq)
	};
	if(dartt_sync(&mctlvq, &m.ds) != DARTT_PROTOCOL_SUCCESS)
	{
		printf("Failed to update motor control settings");
		return 1;
	}

	smooth_mem_t sm = {};
	init_smoothing_mem(&sm);

	bool running = true;
	m.qdset = 0.f;
	uint32_t prevtick = get_tick32()-1000;	//even if it underflows, it'll be correct. yay unsigned integer overflow
	while (running)
	{
		uint32_t tick = get_tick32();

		if((tick - prevtick) > 2000)
		{
			prevtick = tick;
			m.qdset += GOLDEN_ANGLE_RADIANS;
		}

		smooth_qd(m.qdset, 1.f, m.q, &sm, &m.qd, tick);

		int dartt_rc = m.read_data();
		if(dartt_rc != DARTT_PROTOCOL_SUCCESS)
		{
			printf("critical: dartt read error %d\n", dartt_rc);
		}
		else
		{
			angleBuffer.push(m.dp_periph.theta_rem_m, get_microsecond64());
			// m.qd += 0.0001;
			m.write_data();
			float qd_deg_wrapped = wrap_2pi(m.qd)*180.f/M_PI;
			float q_deg_wrapped = wrap_2pi(m.q)*180.f/M_PI;
			// printf("%f, %f\n", qd_deg_wrapped, q_deg_wrapped);
		}
		if (lidar.pollNewFrame())
		{
			printf("lidar timestamp: %u us\n", lidar.latestTimestamp());
		}
		
	}
	
	return 0;
}
