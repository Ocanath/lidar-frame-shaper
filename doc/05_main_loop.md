# 05 — The Main Loop

Source: `src/main.cpp` lines 161–241

---

## The code

```cpp
bool running = true;
m.qdset = 0.f;
uint32_t start_time = get_tick32();
uint32_t prev_tick  = 0;

enum {GOLDEN_SNAP, CONSTANT_VELOCITY};

float   gear_ratio = 1.47435294f;
float   rpm        = 1.f;
uint8_t mode       = CONSTANT_VELOCITY;

{
    MotorWebConfig initCfg;
    initCfg.mode       = mode;
    initCfg.rpm        = rpm;
    initCfg.gear_ratio = gear_ratio;
    initCfg.mctl_vq    = m.dp_ctl.mctl_vq;
    forwarder.initMotorConfig(initCfg);
}
forwarder.startWebServer(1050);

while (running)
{
    uint32_t tick   = get_tick32() - start_time;
    float    t_sec  = (float)tick / 1000.f;

    if(mode == GOLDEN_SNAP)
    {
        if((tick - prev_tick) > 2000)
        {
            prev_tick = tick;
            m.qdset  += GOLDEN_ANGLE_RADIANS * gear_ratio;
        }
        smooth_qd(m.qdset, 1.f, m.q, &sm, &m.qd, tick);
    }
    else if(mode == CONSTANT_VELOCITY)
    {
        m.qd = wrap_2pi(t_sec * M_PI * 2 / 60.f * rpm);
    }

    int dartt_rc = m.read_data();
    if(dartt_rc != DARTT_PROTOCOL_SUCCESS)
    {
        printf("critical: dartt read error %d\n", dartt_rc);
    }
    else
    {
        float raw_anglef    = ((float)(-m.dp_periph.theta_rem_m)) / gear_ratio;
        filtfiltPush(raw_anglef);
        float filtered_anglef = filtfiltRead(FILTFILT_ALPHA);
        int32_t lidar_angle   = wrap_2pi_14b((int32_t)(filtered_anglef));
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

lidar.disconnect();
return 0;
```

---

## Timing

### `uint32_t start_time = get_tick32();`

`get_tick32()` returns the current system time in milliseconds as an unsigned 32-bit integer. We record this once at startup.

### `uint32_t tick = get_tick32() - start_time;`

Every iteration, we compute elapsed time since startup.

**Lesson: Unsigned subtraction handles timer overflow correctly.** `uint32_t` holds values 0 to 4,294,967,295. After ~49.7 days, it wraps to zero. But because subtraction of unsigned integers also wraps:

```
start_time = 0xFFFFFFF0 (near overflow)
current    = 0x00000010 (just wrapped)
current - start_time = 0x00000010 - 0xFFFFFFF0
                     = 0x00000020 = 32 (correct elapsed time)
```

If `start_time` and `current` were signed integers, this would be undefined behavior (signed overflow). As unsigned integers, C guarantees modular arithmetic, so it's always correct. This is a fundamental embedded systems idiom.

### `float t_sec = (float)tick / 1000.f;`

Convert milliseconds to seconds. We only use this for the constant-velocity mode formula. Note that `float` has ~7 significant decimal digits — fine for seconds-level time in a control loop.

---

## Operating modes

### `enum {GOLDEN_SNAP, CONSTANT_VELOCITY};`

A local (anonymous) enum defines two symbolic constants. This is equivalent to `#define GOLDEN_SNAP 0` and `#define CONSTANT_VELOCITY 1` but scoped to the function and type-safe.

### Mode 1: GOLDEN_SNAP

```cpp
if((tick - prev_tick) > 2000)
{
    prev_tick = tick;
    m.qdset  += GOLDEN_ANGLE_RADIANS * gear_ratio;
}
smooth_qd(m.qdset, 1.f, m.q, &sm, &m.qd, tick);
```

Every 2 seconds, advance the target position by the **golden angle** (≈137.5°, the irrational angle that optimally distributes points on a sphere — used in sunflower seeds and phyllotaxis). After N shots, the angles cover the sphere with minimal overlap.

`smooth_qd` generates a smooth velocity profile (`m.qd`) that drives the motor from its current position `m.q` toward the new setpoint `m.qdset` — like a trapezoidal velocity profile but implemented as a low-pass filter on position error.

The gear ratio multiplication converts the desired *LiDAR plate angle* into a *motor shaft angle*: if the LiDAR plate needs to move by 137.5°, the motor shaft must move by 137.5° × 1.47 ≈ 202°.

### Mode 2: CONSTANT_VELOCITY

```cpp
m.qd = wrap_2pi(t_sec * M_PI * 2 / 60.f * rpm);
```

Break this down:
- `rpm / 60.f` = revolutions per second
- `* 2 * M_PI` = radians per second
- `* t_sec` = total radians traveled since start

`wrap_2pi` wraps the angle into [0, 2π), so the commanded position cycles continuously. This is a **ramp generator** — we're telling the motor "be at this angle right now" where the angle is a linearly increasing function of time. The motor velocity controller will do whatever it takes to stay on this ramp.

**Lesson: Open-loop ramps work well for slow, lightly loaded systems.** At 0.5 RPM with no external disturbance, the motor is not fighting any significant torque. A position ramp is simpler and more predictable than a pure velocity command because any timing jitter in the loop gets corrected on the next iteration (the commanded position doesn't drift, it's always recomputed from absolute time).

---

## The angle stamping pipeline (the most important part)

```cpp
float raw_anglef    = ((float)(-m.dp_periph.theta_rem_m)) / gear_ratio;
filtfiltPush(raw_anglef);
float filtered_anglef = filtfiltRead(FILTFILT_ALPHA);
int32_t lidar_angle   = wrap_2pi_14b((int32_t)(filtered_anglef));
angleBuffer.push(lidar_angle, get_microsecond64());
```

This is the core bridge between the motor world and the LiDAR world. Step by step:

### Step 1: Read encoder angle

```cpp
float raw_anglef = ((float)(-m.dp_periph.theta_rem_m)) / gear_ratio;
```

`m.dp_periph.theta_rem_m` is the motor shaft angle from the DARTT response, in Q14 fixed-point (value = radians × 2^14 = radians × 16384).

**Why negative?** The motor runs one direction but the LiDAR plate needs to rotate the other way (belt reversal, mechanical mounting). The negative sign corrects this.

**Why divide by gear_ratio?** The encoder reads the motor shaft, but we want the LiDAR plate angle. The belt drive between them has a 1.474:1 ratio — the motor turns 1.474× more than the LiDAR plate. Dividing by gear_ratio converts shaft angle to plate angle.

```
LiDAR_plate_angle = -(motor_shaft_angle) / 1.47435294
```

The result is still in Q14 fixed-point (integer radians, scaled by 16384) — we just cast it as a float for the filter.

### Step 2: Filter

```cpp
filtfiltPush(raw_anglef);
float filtered_anglef = filtfiltRead(FILTFILT_ALPHA);
```

Push the raw angle into the ring buffer; read back the IIR-smoothed version. See `doc/02_iir_filter.md` for details.

### Step 3: Convert back to Q14 integer and wrap

```cpp
int32_t lidar_angle = wrap_2pi_14b((int32_t)(filtered_anglef));
```

`(int32_t)(filtered_anglef)` truncates the float back to an integer (still Q14 scale). `wrap_2pi_14b` ensures the value is in [−π×16384, +π×16384) — the signed range that the viewer expects.

**Lesson: Round-trip float→int introduces truncation error.** At Q14 scale, 1 unit = 1/16384 radians ≈ 0.0035°. This truncation is negligible for point cloud quality.

### Step 4: Timestamp and push to AngleBuffer

```cpp
angleBuffer.push(lidar_angle, get_microsecond64());
```

`get_microsecond64()` reads the Pi's clock in microseconds (64-bit, no wrap for centuries). This timestamp is what the LiDAR thread uses to interpolate — "what angle was the motor at time T?"

**The timestamp must be taken *after* the motor read, not before.** The DARTT read takes ~0.5ms (serial round-trip). The encoder reading reflects the state of the motor at the moment the MCU sampled it, which is somewhere in the middle of the 0.5ms transfer. By timestamping after the read, we're slightly late (by ~0.25ms average). At 0.5 RPM = 3°/sec, 0.25ms corresponds to 0.00075° of error — completely negligible.

---

## `m.write_data()` ordering

```cpp
int dartt_rc = m.read_data();
if(dartt_rc != DARTT_PROTOCOL_SUCCESS)
{
    printf("critical: dartt read error %d\n", dartt_rc);
}
else
{
    // ... angle stamping ...
    m.write_data();   // ← write is INSIDE the else block
}
```

`write_data()` is inside the `else` block. We only write if the read succeeded. If the MCU didn't respond (disconnected, brown-out, noise on the serial line), we do *not* send a command — the MCU will use its last command until we reestablish communication.

**Lesson: On a communication error, the safest behavior is usually "do nothing" rather than resending the previous command.** Resending could lock the motor at an old setpoint. Doing nothing lets the MCU's own watchdog handle the timeout.

---

## `lidar.pollNewFrame()`

```cpp
if (lidar.pollNewFrame())
{
    // printf("lidar timestamp: %u us\n", lidar.latestTimestamp());
}
```

Non-blocking check: did the LiDAR receive thread complete a full 360° rotation? Returns true once per revolution, then resets. Currently commented out — the frame completion is detected but no main-loop action is taken.

**Lesson: Atomic flag polling is a clean way to communicate events between threads without locking.** `pollNewFrame()` uses `std::atomic<bool>::exchange(false)` — reads the current value and clears it in one atomic operation, so no frame completion can be missed even if it happens between the check and the clear.

---

## Web config polling

```cpp
MotorWebConfig newCfg;
if (forwarder.pollMotorConfig(newCfg))
{
    mode       = newCfg.mode;
    rpm        = newCfg.rpm;
    gear_ratio = newCfg.gear_ratio;
    m.dp_ctl.mctl_vq = newCfg.mctl_vq;
    m.write_pctl_data();
}
```

The web server runs in its own thread (inside `UdpForwarder`). When someone changes settings via the web UI, the new config is stored in a mutex-protected struct. `pollMotorConfig` reads it non-blocking and returns true if new data arrived.

**Why apply the config here in the main loop rather than in the web server thread?** The main loop reads and writes motor hardware (`m.*`). Allowing the web thread to write to `m.dp_ctl` directly would require additional locking around all motor operations. By channeling all motor changes through the main loop, there's only one thread touching the motor — no locking needed on the motor side.

**Lesson: Funneling all hardware access through one thread is a common embedded design pattern.** It eliminates a whole class of race conditions at the cost of slight latency in applying config changes (~10ms loop period).

---

## Cleanup

```cpp
lidar.disconnect();
return 0;
```

`lidar.disconnect()` sets `running_ = false` and waits for the receive thread to finish (`join()`). This is critical — without the join, the thread could still be running when the stack unwinds and `lidar` is destroyed, causing a crash.

**Lesson: Always join threads before the objects they reference go out of scope.** The RAII pattern (Resource Acquisition Is Initialization) guarantees this if you put the join in the destructor — which `LidarSystem::~LidarSystem()` does by calling `disconnect()`.

---

## Next: `06_lidar_thread.md` — Blocking I/O, two-clock synchronization, per-block timestamping, and packet injection
