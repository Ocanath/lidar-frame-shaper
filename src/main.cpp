#include <cstdio>
#define DR_WAV_IMPLEMENTATION
#define TINYCSOCKET_IMPLEMENTATION

// Platform headers (must come before GL on Windows)
#ifdef _WIN32
#define NOMINMAX
#include <winsock2.h>
#include <windows.h>
#else
#include <arpa/inet.h>
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
#include <atomic>
#include <cstring>
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

// ── Zero-phase motor angle filter (filtfilt) ──────────────────────────────────
//
// Applies a first-order IIR forward then backward over a ring buffer of recent
// motor angle samples.  Forward+backward cancels the phase shift so the
// stamped angle has zero lag while still smoothing quantization noise.
//
// FILTFILT_N:    history length in samples (~100 Hz loop → 160 ms at N=16)
// FILTFILT_ALPHA: IIR coefficient — 0=no filter, ~0.5=moderate, ~0.8=heavy

static constexpr int   FILTFILT_N     = 16;
static constexpr float FILTFILT_ALPHA = 0.15f;

static float filtfiltBuf[FILTFILT_N] = {};
static int   filtfiltHead  = 0;
static int   filtfiltCount = 0;

static void filtfiltPush(float q)
{
    filtfiltBuf[filtfiltHead] = q;
    filtfiltHead = (filtfiltHead + 1) % FILTFILT_N;
    if (filtfiltCount < FILTFILT_N) ++filtfiltCount;
}

static float filtfiltRead(float alpha)
{
    const int N = filtfiltCount;
    if (N == 0) return 0.f;
    if (N == 1) return filtfiltBuf[0];

    const int start = (filtfiltHead - N + FILTFILT_N) % FILTFILT_N;

    // Forward pass
    float y[FILTFILT_N];
    y[0] = filtfiltBuf[start];
    for (int i = 1; i < N; ++i)
        y[i] = alpha * y[i-1] + (1.f - alpha) * filtfiltBuf[(start + i) % FILTFILT_N];

    // Backward pass — z[N-1] is the zero-phase filtered value at the newest sample
    float z[FILTFILT_N];
    z[N-1] = y[N-1];
    for (int i = N-2; i >= 0; --i)
        z[i] = alpha * z[i+1] + (1.f - alpha) * y[i];

    return z[N-1];
}

static const pctl_params_t DEFAULT_MCTL_VQ = {
	.kpki = {
		.kp             = {.i32 = 400, .radix = 8},
		.ki             = {.i32 = 3,   .radix = 10},
		.x_integral_div = 10,
		.x              = 0,
		.x_sat          = 1000,
		.out_rshift     = 0
	},
	.kd      = {.i32 = 40, .radix = 5},
	.out_sat = 3546
};

int main(int argc, char* argv[])
{
	(void)argc;
	(void)argv;

	// ── Serial / motor setup ───────────────────────────────────────────────────
	Serial serial;
	bool connected = serial.autoconnect(921600);
	if(connected == false)
	{
		printf("Error: failed to connect to a serial port\n");
	}

	AngleBuffer angleBuffer;

	UdpForwarder forwarder;

	// ── LiDAR setup ───────────────────────────────────────────────────────────
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

	// ── Motor init ────────────────────────────────────────────────────────────
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
	m.dp_ctl.mctl_vq = DEFAULT_MCTL_VQ;
	m.write_pctl_data();
	smooth_mem_t sm = {};
	init_smoothing_mem(&sm);

	// ── Main loop ─────────────────────────────────────────────────────────────
	bool running = true;
	m.qdset = 0.f;
	uint32_t start_time = get_tick32();	//even if it underflows, it'll be correct. yay unsigned integer overflow
	uint32_t prev_tick = 0;

	//operating mode
	enum {GOLDEN_SNAP, CONSTANT_VELOCITY};

	float gear_ratio = 1.47435294f;
	float rpm = 1.f;
	uint8_t mode = CONSTANT_VELOCITY;

	{
		MotorWebConfig initCfg;
		initCfg.mode      = mode;
		initCfg.rpm       = rpm;
		initCfg.gear_ratio = gear_ratio;
		initCfg.mctl_vq   = m.dp_ctl.mctl_vq;
		forwarder.initMotorConfig(initCfg);
	}
	forwarder.startWebServer(1050);

	while (running)
	{
		uint32_t tick = get_tick32() - start_time;
		float t_sec = (float)tick / 1000.f;

		if(mode == GOLDEN_SNAP)
		{
			if((tick - prev_tick) > 2000)
			{
				prev_tick = tick;
				m.qdset += (GOLDEN_ANGLE_RADIANS)*gear_ratio;	//belt ratio
			}
			smooth_qd(m.qdset, 1.f, m.q, &sm, &m.qd, tick);
		}
		else if(mode == CONSTANT_VELOCITY)
		{
			m.qd = wrap_2pi(t_sec*M_PI*2/60.f*rpm);
		}

		int dartt_rc = m.read_data();
		if(dartt_rc != DARTT_PROTOCOL_SUCCESS)
		{
			printf("critical: dartt read error %d\n", dartt_rc);
		}
		else
		{
			// Apply zero-phase filtfilt to encoder angle before pushing into
			// the angle buffer — removes quantization noise without phase lag.
			float raw_anglef = ((float)(-m.dp_periph.theta_rem_m)) / gear_ratio;
			filtfiltPush(raw_anglef);
			float filtered_anglef = filtfiltRead(FILTFILT_ALPHA);
			int32_t lidar_angle = wrap_2pi_14b((int32_t)(filtered_anglef));
			angleBuffer.push(lidar_angle, get_microsecond64());

			m.write_data();
		}

		if (lidar.pollNewFrame())
		{
			// printf("lidar timestamp: %u us\n", lidar.latestTimestamp());
		}

		MotorWebConfig newCfg;
		if (forwarder.pollMotorConfig(newCfg))
		{
			mode       = newCfg.mode;
			rpm        = newCfg.rpm;
			gear_ratio = newCfg.gear_ratio;
			m.dp_ctl.mctl_vq = newCfg.mctl_vq;
			m.write_pctl_data();
		}
	}

	// ── Cleanup ───────────────────────────────────────────────────────────────
	lidar.disconnect();

	return 0;
}
