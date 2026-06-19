# 00 — System Overview & Hardware Data Flow

## What this system does

This is a **3-D LiDAR scanner** made from two off-the-shelf parts plus a custom motor controller:

| Part | Role |
|------|------|
| Velodyne VLP-16 ("Puck") | Spins its own 16-laser head at ~600 RPM and fires UDP packets over Ethernet |
| Secondary motor + belt drive | Slowly nods/rotates the entire Puck at ~0.5–1 RPM to sweep a full sphere |
| Motor controller (DARTT) | Runs field-oriented-control firmware; reports encoder position over serial |
| Raspberry Pi | Bridges everything — reads encoder, fuses angles into LiDAR packets, forwards to viewer |
| Windows PC (OpenGL viewer) | Renders the 3-D point cloud in real time |

The VLP-16 alone gives you a 30-degree vertical slice (~300,000 pts/sec). The secondary motor turns that slice into a full sphere over ~1–2 minutes.

---

## Data flow diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          Raspberry Pi                                       │
│                                                                             │
│  ┌──────────────┐      UART/COBS/DARTT        ┌────────────────────────┐  │
│  │ Motor ctrl   │ ◄──────────────────────────► │  Serial + Motor obj   │  │
│  │ (DARTT MCU)  │   theta_rem_m every ~10ms    │  m.read_data()        │  │
│  └──────────────┘                              └──────────┬─────────────┘  │
│                                                           │ raw angle       │
│                                                    filtfilt (IIR, N=16)    │
│                                                           │ smoothed angle  │
│                                                    ┌──────▼──────────────┐  │
│                                                    │  AngleBuffer        │  │
│                                                    │  (ring buf, 256)    │  │
│                                                    └──────▲──────────────┘  │
│                                                           │ interpolate()   │
│  ┌──────────────┐     UDP :2381 (1206 B)       ┌─────────┴──────────────┐  │
│  │  VLP-16      │ ──────────────────────────── │  LidarSystem           │  │
│  │  (Puck)      │  raw VLP packets             │  recvLoop()            │  │
│  └──────────────┘                              │  decodePacket()        │  │
│                                                │  ↓ inject motor angle  │  │
│                                                │  modified 1242-B pkt   │  │
│                                                └──────────┬─────────────┘  │
│                                                           │ onPacketReady  │
│                                                    ┌──────▼──────────────┐  │
│                                                    │  UdpForwarder       │  │
│                                                    │  send() → :9000     │  │
│                                                    │  web cfg → :1050    │  │
│                                                    └─────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
                                  │ UDP :9000 (1242 B, modified packets)
                                  ▼
                         ┌──────────────────┐
                         │  Windows Viewer  │
                         │  receivePacket() │
                         │  OpenGL render   │
                         └──────────────────┘
```

---

## The core timing problem

The VLP-16 and the secondary motor have **two completely independent clocks**:

- **VLP-16 clock**: µs since the top of the current hour, reset every hour
- **Pi CLOCK_BOOTTIME**: µs since the Pi last booted

They drift apart over time. The Pi needs to know *exactly* what its own clock read when each LiDAR block was fired, so it can look up the motor angle at that instant.

The solution is an **epoch offset** — a running estimate of `(Pi_clock - VLP_clock)` maintained with an EMA (exponentially weighted moving average). See `doc/06_lidar_thread.md` for the details.

---

## Why we inject the angle into the packet

Instead of sending angle data on a separate channel, we modify each 1206-byte VLP-16 packet to a **1242-byte modified packet** by inserting 3 bytes of motor angle into every block header. This keeps the data synchronized: the Windows viewer receives one self-contained stream where every point already knows what motor angle it corresponds to. No extra protocol needed.

---

## Modified packet format

**Raw VLP-16 packet (1206 bytes):**
```
[Block 0: 100 B][Block 1: 100 B] … [Block 11: 100 B][Footer: 6 B]
                                                       └─ timestamp, return mode, model ID
Each block: [0xFF 0xEE][azimuth LE 2B][ch0: dist 2B + intensity 1B] … [ch31]
```

**Modified packet (1242 bytes):**
```
[Block 0: 103 B][Block 1: 103 B] … [Block 11: 103 B][Footer: 6 B]
Each block: [0xFF 0xEE][azimuth LE 2B][motor_angle LE 3B][ch0–ch31 unchanged]
                                        ↑
                              3 new bytes inserted here
```

The +3 bytes per block × 12 blocks = +36 bytes total → 1206 + 36 = 1242.

---

## Files at a glance

| File | Lives on | Purpose |
|------|----------|---------|
| `src/main.cpp` | Pi | Orchestration: serial, motor, LiDAR, angle stamping |
| `src/lidar.cpp` | Pi | UDP receive, clock sync, packet injection |
| `src/lidar.h` | Pi | LidarSystem class definition |
| `src/angle_buffer.h` | Pi | Thread-safe ring buffer with interpolation |
| `src/motor.h/cpp` | Pi | DARTT motor abstraction |
| `src/serial.h/cpp` | Pi | UART serial port |
| `src/udp_forwarder.h/cpp` | Pi | Sends modified packets; serves web config |
| `src/tick.h` | Pi | `get_tick32()` / `get_microsecond64()` |
| `src/trig_fixed.h` | Pi | `wrap_2pi_14b()` fixed-point wrap |
| `external/tinycsocket.h` | Pi | Header-only cross-platform UDP |
| `external/cobs.h` | Pi | COBS byte-stuffing for DARTT serial |

---

## Next: `01_includes.md` — Includes & platform abstraction
