# 04 — Startup: Serial, Objects, Motor Init, and the Startup Sound

Source: `src/main.cpp` lines 105–160

---

## The code

```cpp
int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    // ── Serial / motor setup ──────────────────────────────────────────────
    Serial serial;
    bool connected = serial.autoconnect(921600);
    if(connected == false)
    {
        printf("Error: failed to connect to a serial port\n");
    }

    AngleBuffer angleBuffer;
    UdpForwarder forwarder;

    // ── LiDAR setup ───────────────────────────────────────────────────────
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

    // ── Motor init ────────────────────────────────────────────────────────
    Motor m(0, &serial);    // DARTT address = 0
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
```

---

## `(void)argc; (void)argv;`

This suppresses compiler warnings about unused parameters. `main` is required by the C standard to accept `argc` and `argv`, but we don't use command-line arguments. Writing `(void)var` is the idiomatic embedded way to say "I know this exists, I'm intentionally not using it."

**Lesson:** Embedded projects typically compile with `-Wall -Wextra -Werror` (treat all warnings as errors). Unused parameter warnings must be explicitly silenced, not ignored. Some teams use `__attribute__((unused))` instead, but `(void)var` is portable across all compilers.

---

## `Serial serial; serial.autoconnect(921600);`

The serial port is the UART link to the motor controller. `autoconnect` scans available serial ports and connects to the first one that responds to a DARTT handshake at 921600 baud.

**Why 921600 baud?** This is one of the highest standard baud rates. UART is slow compared to SPI or CAN bus — 921600 baud gives about 92,160 bytes/second (10 bits per byte including start/stop bits). The DARTT protocol round-trip (request + response) needs to complete in well under 10ms to keep the control loop running at ~100Hz. At 921600 baud, a ~50-byte DARTT packet takes ~0.5ms, which fits comfortably.

**Lesson:** Baud rate selection is a trade-off between speed and reliability. Higher baud rates are more sensitive to clock accuracy and cable capacitance. 921600 works reliably on short cables between the Pi and a nearby MCU.

---

## `AngleBuffer angleBuffer;`

Default-constructed — starts empty. It's a 256-slot ring buffer that will hold `{int32_t theta_rem_m, uint64_t timestamp_us}` pairs. The main loop pushes into it; the LiDAR receive thread reads from it.

**Lesson:** Creating the shared buffer before the threads that use it ensures there's no race condition at startup. Both threads see the same initialized object.

---

## `UdpForwarder forwarder;`

Handles all outbound UDP:
- Port 9000 → Windows viewer (1242-byte modified packets)
- Port 1050 → Web server for runtime config (gear ratio, RPM, PID gains)

---

## LiDAR setup: dependency injection via pointers and callbacks

```cpp
lidar.angleBuffer   = &angleBuffer;
lidar.onPacketReady = [&forwarder](const uint8_t* data, size_t len) {
    forwarder.send(data, len);
};
bool lidar_connected = lidar.connect(2381);
```

`LidarSystem` has two dependencies:
1. **`angleBuffer`** — where to look up motor angles when decoding packets
2. **`onPacketReady`** — what to do with a finished modified packet

Instead of hard-coding these, `lidar` exposes public pointers/callbacks so `main` can wire them together. This is called **dependency injection** — the object doesn't create its own dependencies, they're handed to it.

**`onPacketReady` is a lambda (anonymous function):** The `[&forwarder]` captures `forwarder` by reference. When the LiDAR thread calls `onPacketReady(data, len)`, it's calling `forwarder.send(data, len)`.

**Lesson:** Lambdas as callbacks are the modern C++ way to avoid global variables or function pointers. The `[&forwarder]` capture means "this lambda has a reference to forwarder — it doesn't copy it." This is safe here because `forwarder` outlives the `lidar` object (both are on the stack in `main`).

**Why `lidar.connect(2381)` last?** `connect` spawns the receive thread. If we called it before setting `angleBuffer` and `onPacketReady`, the thread could run a packet through before those are wired up, and we'd crash dereferencing a null pointer.

**Lesson:** In embedded/concurrent code, always set up all dependencies before starting any thread that uses them. Construction order matters.

---

## `Motor m(0, &serial);`

Creates a motor object for DARTT address 0. DARTT is an addressable bus — you could have multiple motor controllers on one UART by giving them different addresses (0, 1, 2...). Here there's only one.

---

## `index_of_field(&m.dp_ctl.audio, ...)`

This looks up the index (offset) of the `audio` field within the DARTT control packet's binary layout. The AudioWriter needs to know exactly which bytes in the packet to overwrite with audio data.

**Lesson:** Protocols that mix multiple fields into one binary packet need a way to locate fields. Instead of hard-coding offsets (fragile — breaks if struct layout changes), this function computes the offset at runtime by comparing pointer addresses. This is a defensive coding pattern.

```cpp
int writer_idx = index_of_field(&m.dp_ctl.audio, m.ds.ctl_base.buf, m.ds.ctl_base.size);
if(writer_idx < 0)
{
    printf("Catastrophic error - no writer present in dartt control structure\n");
    return writer_idx;
}
```

If `writer_idx < 0`, the audio field wasn't found — the struct layout is wrong or the MCU firmware doesn't match. This is a "catastrophic" error because it means the Pi and MCU are running incompatible software versions. **The program must exit.** Running with mismatched protocol definitions could send garbage commands to the motor.

---

## The startup sound as a hardware presence check

```cpp
AudioWriter audio_writer(0, writer_idx, serial);
if(audio_writer.play("assets/startupsound_processed.wav") != 0)
{
    printf("No device present - exiting\n");
    return 1;
}
```

Playing a sound through the motor controller is a clever **dual-purpose operation**:
1. It plays an audible startup chime (useful for knowing the system is alive)
2. It **confirms the MCU is present and responding** — if `play()` fails (no ACK from the MCU), we know something is wrong and exit immediately

**Lesson: Test your most critical dependency first, and make that test observable.** A silent startup makes debugging blind. If the system comes up and immediately plays a sound, you know: Pi is running, serial is connected, MCU is powered and responding. If there's no sound, you know exactly where the failure is. This is called a **canary check** — borrowing the mining metaphor of a canary in a coal mine.

---

## `m.rezero()`

Sets the encoder's current position as zero. The motor controller's encoder counts from its power-on position. `rezero` tells the MCU "wherever you are now is position zero" — this defines the coordinate origin for all subsequent angle readings.

**Lesson:** Many motor controllers need a homing or rezero step. Without it, the "angle" you read is arbitrary and meaningless. Always rezero before running any position-dependent control.

---

## PID gain upload and initial state

```cpp
m.read_data();
m.dp_ctl.command_word = 0;
m.write_data();
m.dp_ctl.mctl_vq = DEFAULT_MCTL_VQ;
m.write_pctl_data();
```

Step by step:
1. `m.read_data()` — sync the local struct with the MCU's current state
2. `command_word = 0` — disable all motor commands (safe starting state)
3. `m.write_data()` — send that safe state to the MCU
4. `m.dp_ctl.mctl_vq = DEFAULT_MCTL_VQ` — set the PID gain struct in memory
5. `m.write_pctl_data()` — upload the PID gains to the MCU

**Why write a safe state before uploading gains?** If you upload gains first, and the motor was already running (e.g., previous run didn't clean up), the new gains take effect immediately with whatever setpoint was already in the MCU. Writing `command_word = 0` first ensures the motor is stopped before reconfiguring it.

**Lesson: Initialize hardware into a known safe state, then configure, then enable.** The order matters. Initializing gains on a running motor can cause a sudden jerk or spike.

---

## `smooth_mem_t sm = {}; init_smoothing_mem(&sm);`

Initializes the trajectory smoother used in GOLDEN_SNAP mode. The `= {}` zero-initializes the struct (all bytes to zero), then `init_smoothing_mem` sets any non-zero defaults.

**Lesson:** In C/C++, uninitialized local variables have garbage values. For structs that contain state (like a filter or trajectory generator), always explicitly initialize before use. `= {}` is the cleanest way in C++ to zero-initialize a struct.

---

## Next: `05_main_loop.md` — Unsigned timing, operating modes, angle stamping pipeline, write ordering
