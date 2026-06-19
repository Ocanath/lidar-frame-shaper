# 08 — Embedded Systems Concepts: Reference Table

This document summarizes every major embedded systems concept used in this project, with a brief definition and where to find it in the code.

---

## Data structures

| Concept | Definition | Where used |
|---------|-----------|-----------|
| **Ring buffer (circular buffer)** | Fixed-size array with a head pointer that wraps around. O(1) push, no allocation. Oldest data is automatically overwritten when full. | `AngleBuffer` (256 angle samples), `filtfiltBuf` (16 filter samples), `frames_` (deque of point cloud frames) |
| **Double-ended queue (deque)** | Like a vector but O(1) at both ends. Used when you need to push to back and pop from front. | `frames_` in `LidarSystem` |
| **Fixed-point integer** | A real number stored as an integer scaled by 2^N. Avoids floating-point hardware. | `theta_rem_m` (Q14: value = radians × 16384) |

---

## Signal processing

| Concept | Definition | Where used |
|---------|-----------|-----------|
| **IIR filter (first-order low-pass)** | `y[i] = α·y[i−1] + (1−α)·x[i]`. Smooths noisy signals. Single multiply+add per sample. | `filtfiltBuf` in main loop (α=0.15, N=16) |
| **EMA (Exponential Moving Average)** | Same as IIR low-pass, used for tracking a slowly drifting value. | Clock sync `epochOffsetUs_` (α=1/100) |
| **Forward-backward filtering (filtfilt)** | Filter forward in time, then backward, to cancel phase lag. Returns zero-phase output. | `filtfiltRead()` (note: has a bug — see doc 02) |
| **Linear interpolation** | Estimate a value between two known points: `a + t * (b - a)`, t ∈ [0,1]. | `AngleBuffer::interpolate()` |
| **Angle wrap / unwrapping** | When an angle crosses ±π, the raw difference is ≈ ±2π but the motion is small. Detect and correct the jump. | `interpolate()` diff unwrap, `wrap_2pi_14b()` |

---

## Concurrency and synchronization

| Concept | Definition | Where used |
|---------|-----------|-----------|
| **Mutex (mutual exclusion lock)** | Prevents two threads from executing a critical section simultaneously. | `AngleBuffer::mtx_`, `LidarSystem::framesMutex_` |
| **RAII lock guard** | Acquires a mutex in its constructor, releases in its destructor. Lock is always released, even if an exception is thrown. | `std::lock_guard<std::mutex>` everywhere |
| **`mutable` mutex** | Allows a mutex to be locked inside a `const` method. The mutex is an implementation detail, not part of the object's logical state. | `AngleBuffer::mtx_` declared `mutable` |
| **Atomic flag** | A single variable that can be read/written from any thread without a mutex (hardware-guaranteed single-instruction operations). | `newFrameFlag_` in `LidarSystem` |
| **`exchange(false)`** | Atomically reads a value and sets it to false in one operation. Prevents losing an event between "check" and "clear". | `pollNewFrame()` |
| **Thread join** | Block the calling thread until another thread finishes. Ensures the thread's resources are cleaned up before the objects it used are destroyed. | `recvThread_.join()` in `disconnect()` |
| **Dependency injection** | Don't create dependencies inside an object — pass them in from outside. Makes code testable and loosely coupled. | `lidar.angleBuffer = &angleBuffer` |

---

## Communication protocols

| Concept | Definition | Where used |
|---------|-----------|-----------|
| **UART / serial** | Asynchronous serial bus. Two wires (TX, RX). No clock line. Both sides must agree on baud rate. Common in embedded systems for short-distance MCU communication. | Motor controller ↔ Pi (921600 baud) |
| **Baud rate** | Bits per second on a serial line. 921600 baud ≈ 92,160 bytes/second (at 10 bits per byte with start/stop). | Serial to motor controller |
| **UDP (User Datagram Protocol)** | Connectionless network protocol. No handshake, no guaranteed delivery, no ordering. Fast and simple — used when you control both ends and can tolerate lost packets. | VLP-16 → Pi (port 2381), Pi → Viewer (port 9000) |
| **COBS (Consistent Overhead Byte Stuffing)** | Framing protocol for serial. Ensures the delimiter byte (0x00) never appears in the payload. Overhead: at most 1 byte per 254 bytes. | DARTT protocol framing |
| **DARTT protocol** | Custom addressable request/response protocol over UART/COBS. Allows multiple devices on one serial bus. | Motor controller communication |
| **Little-endian** | Multi-byte numbers stored with least-significant byte first. `[A0, A1, A2, A3]` = `A0 | (A1<<8) | (A2<<16) | (A3<<24)`. Common on x86, VLP-16. | VLP-16 packet parsing throughout |

---

## Timing and clocks

| Concept | Definition | Where used |
|---------|-----------|-----------|
| **Unsigned integer timer arithmetic** | `uint32_t` subtraction correctly handles overflow: `(0x10 - 0xFFFFFFF0) = 0x20 = 32`. Works for any wrap-around timer without special handling. | `tick = get_tick32() - start_time` |
| **Two-clock synchronization** | Mapping between two independent clocks (VLP-16 internal vs. Pi CLOCK_BOOTTIME) using a running estimate of their offset. | `epochOffsetUs_` EMA in `decodePacket()` |
| **Top-of-hour wrap detection** | VLP-16 timestamp resets every 3600 seconds. A sudden jump of ≈3.6 billion microseconds in the offset indicates a wrap, not a real change. | `if (delta < -1800000000LL)` in clock sync |
| **Per-block timestamping** | Each of the 12 blocks in a VLP-16 packet is fired at a different time. Assigning each block its own inferred timestamp enables precise angle lookup even at high motor speeds. | `block_prog_us = lidar_ts + b * BLOCK_INTERVAL_US + epochOffsetUs_` |
| **Non-blocking poll** | Check a condition without waiting for it. Returns immediately with true/false. Used to check for events in a busy loop. | `pollNewFrame()`, `pollMotorConfig()` |

---

## Motor control

| Concept | Definition | Where used |
|---------|-----------|-----------|
| **PID controller** | P=proportional, I=integral, D=derivative. Continuously corrects error between setpoint and measurement. The fundamental building block of motor control. | `DEFAULT_MCTL_VQ` PID gains on MCU |
| **Fixed-point PID gains** | PID gains stored as `{integer, radix}` pairs. Real value = integer / 2^radix. Avoids floating-point on MCU. | `kp={400,8}→1.5625`, `ki={3,10}→0.00293` |
| **Output saturation** | Maximum output the PID can produce. Prevents integrator windup and protects hardware. | `out_sat = 3546` |
| **Velocity loop** | PID that controls motor speed (not position). `m.qd` = velocity setpoint in velocity control mode. | CONSTANT_VELOCITY mode |
| **Trajectory generator** | Smooth function that generates a position or velocity profile to reach a target without jerky motion. | `smooth_qd()` in GOLDEN_SNAP mode |
| **Gear ratio** | Mechanical transmission between motor shaft and output (LiDAR plate). `plate_angle = motor_angle / ratio`. | `gear_ratio = 1.47435294` |
| **Rezero / homing** | Reset the encoder's zero reference to the current position. Defines the coordinate system for all subsequent angle readings. | `m.rezero()` at startup |

---

## Software patterns

| Concept | Definition | Where used |
|---------|-----------|-----------|
| **Single-header library** | A library distributed as one `.h` file. `#define LIB_IMPLEMENTATION` in one `.cpp` to include the definitions. | `tinycsocket.h`, `dr_wav.h` |
| **Platform guard** | `#ifdef _WIN32 ... #else ... #endif` to handle OS-specific API differences. | Socket headers in `main.cpp` |
| **Lambda callback** | Anonymous function that captures local variables. Used to wire together objects without global state. | `lidar.onPacketReady = [&forwarder](...) {...}` |
| **RAII (Resource Acquisition Is Initialization)** | Acquire a resource in a constructor, release it in the destructor. Guarantees cleanup even if the code throws. | `LidarSystem::~LidarSystem()` calls `disconnect()` |
| **Canary / presence check** | Test that a critical hardware dependency is present at startup. Fail loudly if it's not. | `audio_writer.play()` checks MCU is alive |
| **`(void)var`** | Explicitly suppress "unused variable" compiler warnings without changing behavior. Common in embedded where `-Werror` is standard. | `(void)argc; (void)argv;` in `main()` |
| **`= {}` zero-initialization** | Initialize a struct or array to all zeros at declaration. Prevents undefined behavior from uninitialized state. | `smooth_mem_t sm = {}` |

---

## Point cloud geometry

| Concept | Definition | Where used |
|---------|-----------|-----------|
| **Spherical to Cartesian** | Convert (distance, azimuth, elevation) → (X, Y, Z) using trig. X = d·cos(el)·sin(az), Y = d·cos(el)·cos(az), Z = d·sin(el). | LiDAR point decode in `decodePacket()` |
| **VLP-16 interleaved lasers** | 16 beams at angles {−15°,+1°,−13°,+3°,...,+15°}. Interleaved to maximize firing rate and minimize crosstalk. | `VERT_ANGLES[16]` array |
| **Azimuth in centidegrees** | VLP-16 stores azimuth as uint16 in units of 0.01°. Gives 0.01° resolution over 360°. | `az_raw / 100.f` conversions |
| **Frame boundary detection** | Detect when the LiDAR completes a 360° scan by watching for the azimuth wrapping from near-360° back to near-0°. | `lastAzimuth_ > 18000.f && azimuth_block < lastAzimuth_` |
| **Magic bytes / sync word** | `0xFF 0xEE` at the start of every VLP-16 data block. Used to validate that we're parsing the right offset. | Block header check in packet decode |

---

## Further reading

- **VLP-16 packet format**: `doc/VLP-16 User Manual and Programming Guide 63-9243 Rev A.pdf`
- **Ring buffers**: Any embedded systems textbook; also "Making Embedded Systems" by Elecia White
- **Fixed-point arithmetic**: "Fixed-Point Arithmetic" — ARM application note AN 100
- **PID control**: "PID Without a PhD" by Tim Wescott (free online)
- **COBS**: "Consistent Overhead Byte Stuffing" by Stuart Cheshire and Mary Baker (1999)
