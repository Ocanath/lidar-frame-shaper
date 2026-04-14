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
#include <atomic>
#include <cstring>
#include "dartt_mctl_params.h"
#include "motor.h"
#include "serial.h"
#include "lidar.h"
#include "AudioWriter.h"
#include "milliseconds.h"
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
static constexpr float FILTFILT_ALPHA = 0.5f;

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

// ── Packet injection ──────────────────────────────────────────────────────────
//
// Takes a standard 1206-byte VLP-16 packet and builds a 1242-byte modified
// packet by inserting 3 bytes of motor angle into each of the 12 blocks.
//
// Original block layout (100 bytes):
//   [0xFF][0xEE][az_lo][az_hi][ch0 dist_lo][ch0 dist_hi][ch0 intensity] ... (32 ch × 3B)
//
// Modified block layout (103 bytes):
//   [0xFF][0xEE][az_lo][az_hi][ma_lo][ma_mid][ma_hi][ch0 ...] ... (32 ch × 3B)
//
// motor_angle_fixed is q converted to 14-bit radix fixed-point:
//   int32_t fixed = (int32_t)(q_radians * (float)(1 << 14))
//
static void buildModifiedPacket(const uint8_t* raw1206,
                                 int32_t        motor_angle_fixed,
                                 uint8_t*       out1242)
{
    // Extract the 3 bytes we'll stamp into every block.
    // Store as little-endian 24-bit signed (matches the viewer's sign-extension logic).
    uint8_t ma0 =  (uint8_t)( motor_angle_fixed        & 0xFF);
    uint8_t ma1 =  (uint8_t)((motor_angle_fixed >>  8) & 0xFF);
    uint8_t ma2 =  (uint8_t)((motor_angle_fixed >> 16) & 0xFF);

    for (int b = 0; b < 12; ++b)
    {
        const uint8_t* src = raw1206 + b * 100;   // original block start
        uint8_t*       dst = out1242 + b * 103;   // modified block start

        // Sync + azimuth (4 bytes) — copy as-is
        dst[0] = src[0];   // 0xFF
        dst[1] = src[1];   // 0xEE
        dst[2] = src[2];   // azimuth low byte
        dst[3] = src[3];   // azimuth high byte

        // Motor angle (3 new bytes) — this is q, the actual measured position
        dst[4] = ma0;
        dst[5] = ma1;
        dst[6] = ma2;

        // 32 channels × 3 bytes each (96 bytes) — copy as-is
        memcpy(dst + 7, src + 4, 96);
    }

    // Footer: 6 bytes (timestamp 4B + return_mode 1B + model_id 1B)
    // Original footer starts at byte 1200; modified footer starts at byte 1236
    memcpy(out1242 + 1236, raw1206 + 1200, 6);
}


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

	// ── LiDAR setup ───────────────────────────────────────────────────────────
	LidarSystem lidar;
	bool lidar_connected = lidar.connect(2381);
	if(lidar_connected)
	{
		printf("Bind success to lidar port - ready for data\n");
	}

	// ── Output UDP socket (sends modified packets to the viewer) ──────────────
	//
	// The viewer (opengl_boilerplate) listens on port 9000 for 1242-byte packets.
	// We set up a plain UDP socket here and send to 127.0.0.1:9000 (localhost).
	// If the viewer is on a different machine, change the address below.

	tcs_lib_init();
	TcsSocket outSocket = TCS_SOCKET_INVALID;
	if (tcs_socket_preset(&outSocket, TCS_PRESET_UDP_IP4) != TCS_SUCCESS)
	{
		printf("Warning: failed to create output UDP socket — viewer will not receive data\n");
	}

	struct TcsAddress viewerAddr = TCS_ADDRESS_NONE;
	viewerAddr.family                = TCS_AF_IP4;
	viewerAddr.data.ip4.address      = inet_addr("100.168.0.23"); // viewer machine
	viewerAddr.data.ip4.port         = 9000;

	// ── Thread-safe motor angle ────────────────────────────────────────────────
	//
	// The onRawPacket callback fires on the LiDAR receive thread, but m.q is
	// written on the main thread.  We bridge this with an atomic int32_t that
	// stores q in 14-bit radix fixed-point — the same format the viewer expects.
	//
	// Why fixed-point instead of float?
	//   std::atomic<float> exists but has limited compiler support.
	//   int32_t is universally atomic on x86/ARM and matches the wire format
	//   so we don't need to convert twice.

	std::atomic<int32_t> motorAngleFixed{0};

	// ── Register the raw-packet callback ──────────────────────────────────────
	//
	// This lambda runs on the LiDAR receive thread each time a 1206-byte packet
	// arrives.  It reads the latest motor angle (atomically), builds the 1242-
	// byte modified packet, and sends it to the viewer.

	uint8_t outBuf[1242];   // reused each call — safe because callback is single-threaded

	lidar.onRawPacket = [&](const uint8_t* raw, size_t len)
	{
		if (len < 1206) return;
		if (outSocket == TCS_SOCKET_INVALID) return;

		// Grab the latest q — atomic load, no lock needed
		int32_t motorFixed = motorAngleFixed.load(std::memory_order_relaxed);

		// Build the modified 1242-byte packet
		buildModifiedPacket(raw, motorFixed, outBuf);

		// Send to viewer
		size_t sent = 0;
		TcsResult rc = tcs_send_to(outSocket, outBuf, 1242,
		                           TCS_FLAG_NONE, &viewerAddr, &sent);
		if (rc != TCS_SUCCESS)
		{
			printf("Warning: failed to send modified packet to viewer\n");
		}
	};

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

	// ── Main loop ─────────────────────────────────────────────────────────────
	bool running = true;
	m.qdset = 0.f;
	uint32_t prevtick = get_tick32()-1000;
	while (running)
	{
		uint32_t tick = get_tick32();

		// Every 2 seconds, advance the target angle by the golden angle (~137.5°)
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
			m.write_data();

			// Push raw encoder reading into filtfilt buffer and store the
			// zero-phase filtered result for the packet injection callback.
			filtfiltPush(m.q);
			motorAngleFixed.store(
				(int32_t)(filtfiltRead(FILTFILT_ALPHA) * (float)(1 << 14)),
				std::memory_order_relaxed
			);
		}

		if (lidar.pollNewFrame())
		{
			printf("lidar timestamp: %u us\n", lidar.latestTimestamp());
		}
	}

	// ── Cleanup ───────────────────────────────────────────────────────────────
	lidar.disconnect();
	if (outSocket != TCS_SOCKET_INVALID)
		tcs_close(&outSocket);
	tcs_lib_free();

	return 0;
}
