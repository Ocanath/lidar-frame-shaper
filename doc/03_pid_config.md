# 03 — Fixed-Point PID Gains & Motor Controller Architecture

Source: `src/main.cpp` lines 92–103

---

## The code

```cpp
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
```

---

## What is a PID controller?

A **PID controller** (Proportional-Integral-Derivative) is the fundamental building block of motor control. It continuously corrects an error:

```
error = setpoint - measurement

output = Kp * error
       + Ki * integral(error)
       + Kd * derivative(error)
```

| Term | Symbol | Effect |
|------|--------|--------|
| Proportional | Kp | Immediate correction proportional to current error |
| Integral | Ki | Eliminates steady-state error (accumulated past error) |
| Derivative | Kd | Damping — opposes rapid changes in error |

For a motor running at constant velocity:
- **Kp** keeps the motor at the right speed by applying more current when it's slow
- **Ki** eliminates the offset that pure P-control always has (without I, the motor runs slightly slower than commanded because the P term only acts when there's error)
- **Kd** smooths out oscillations when Kp is large

---

## Fixed-point arithmetic

The motor controller MCU (a small microcontroller) almost certainly has no floating-point unit. All math is done in **fixed-point integers**.

### What "radix" means

A fixed-point number stores a real value as an integer scaled by a power of 2:

```
real_value = integer_value / 2^radix
```

For example:
- `kp = {.i32 = 400, .radix = 8}` → real Kp = 400 / 2^8 = 400 / 256 = **1.5625**
- `ki = {.i32 = 3,   .radix = 10}` → real Ki = 3 / 2^10 = 3 / 1024 ≈ **0.00293**
- `kd = {.i32 = 40,  .radix = 5}`  → real Kd = 40 / 2^5 = 40 / 32 = **1.25**

**Why use radix instead of storing the real value?** On an MCU without FPU, every floating-point operation must be emulated in software (dozens of cycles). Fixed-point multiplication is one instruction. The MCU multiplies two integers and then right-shifts by the radix to correct the scale. This is 10–100× faster.

**Lesson:** Fixed-point math is everywhere in embedded motor control. When you see values like `{.i32 = 400, .radix = 8}`, mentally translate: "this is a fractional number represented as an integer, scaled by 2^radix."

---

## The gain values in context

| Gain | Integer | Radix | Real value |
|------|---------|-------|-----------|
| Kp | 400 | 8 | 1.5625 |
| Ki | 3 | 10 | ~0.00293 |
| Kd | 40 | 5 | 1.25 |

These are **velocity loop gains** (`mctl_vq` = motor control, velocity, quadrature). The motor is being commanded to a velocity setpoint (`m.qd` in rad/s equivalent), and the PID controls the motor current to achieve that velocity.

- Kp=1.56 is moderate — responsive without being too aggressive
- Ki≈0.003 is very small — the integrator accumulates slowly (important: a large Ki on a velocity loop can cause "integrator windup" where the integral term builds up so much it causes overshoots and oscillations)
- `x_integral_div = 10` further divides the integral accumulation rate — a safety valve

---

## `out_sat = 3546`

This is the **output saturation** — the maximum PWM duty cycle or current command the PID can produce. The value 3546 is in whatever units the motor controller's output stage uses (likely scaled Q-values for the FOC current controller). Saturation prevents the PID from commanding more current than the motor or power supply can safely provide.

**Lesson:** Every PID in real hardware has an output saturation limit. Without it, integral windup during a large disturbance (someone grabbing the motor) can saturate the output at maximum and the motor becomes uncontrollable when the disturbance is removed.

---

## DARTT motor controller architecture

The motor controller communicates over UART using the DARTT protocol (custom). The Pi has two main operations:

### 1. `m.read_data()`

Asks the MCU for its current peripheral state:
- `m.dp_periph.theta_rem_m` — encoder angle (signed fixed-point)
- `m.dp_periph.omega` — measured angular velocity

### 2. `m.write_data()`

Sends control commands:
- `m.dp_ctl.command_word` — enable/disable motor
- `m.qd` — velocity setpoint

### 3. `m.write_pctl_data()`

Uploads PID gains. This is separate from write_data because gains rarely change and are stored in a different register bank on the MCU.

**Lesson:** In embedded systems, read and write to peripherals are explicit — nothing happens automatically. The Pi must *ask* the MCU for data, then *send* new commands. This is typical of an SPI/I2C/UART bus protocol where the high-level processor is the bus master.

---

## `m.qd` — the velocity setpoint

In DARTT notation:
- `q` = generalized position (angle, in radians equivalent)
- `qd` = derivative of q = velocity (rad/s equivalent)
- `qdset` = desired position (for position control)

The motor is running in **velocity control mode** for the constant-velocity scan (CONSTANT_VELOCITY mode), meaning `m.qd` is directly the commanded angular velocity. In GOLDEN_SNAP mode, a trajectory generator computes `qd` from the position error.

---

## Next: `04_startup.md` — Serial setup, object initialization, motor rezero, and the startup sound as a watchdog
