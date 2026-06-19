# 07 — AngleBuffer: The Interpolating Angle Ring Buffer

Source: `src/angle_buffer.h`

---

## Purpose

`AngleBuffer` is the **bridge between two threads running at different rates**:

| Thread | Rate | Operation |
|--------|------|-----------|
| Main loop | ~100 Hz | `push(angle, timestamp)` |
| LiDAR receive | ~750 Hz | `interpolate(timestamp)` |

The main loop reads the motor encoder every ~10ms and pushes a timestamped angle sample. The LiDAR thread asks "what was the motor angle at time T?" for each of the 12 blocks in each packet. `interpolate` finds the two samples that bracket time T and returns the angle you'd get if the motor moved smoothly between them.

---

## The full code

```cpp
struct AngleSample {
    int32_t  theta_rem_m;   // motor angle in Q14 fixed-point
    uint64_t timestamp_us;  // Pi CLOCK_BOOTTIME in microseconds
};

class AngleBuffer {
public:
    static constexpr int DEPTH = 256;

    void push(int32_t theta, uint64_t us)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        buf_[head_] = { theta, us };
        head_ = (head_ + 1) % DEPTH;
        if (count_ < DEPTH) ++count_;
    }

    AngleSample findNearest(uint64_t us) const { ... }  // kept but superseded

    AngleSample interpolate(uint64_t us) const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (count_ == 0) return {};
        if (count_ == 1) return buf_[0];

        int start = (count_ < DEPTH) ? 0 : head_;

        int beforeIdx = -1;
        int afterIdx  = -1;
        for (int i = 0; i < count_; ++i)
        {
            int idx = (start + i) % DEPTH;
            if (buf_[idx].timestamp_us <= us)
                beforeIdx = idx;
            else if (afterIdx == -1)
            {
                afterIdx = idx;
                break;
            }
        }

        if (beforeIdx == -1) return buf_[afterIdx];
        if (afterIdx  == -1) return buf_[beforeIdx];

        const AngleSample& s0 = buf_[beforeIdx];
        const AngleSample& s1 = buf_[afterIdx];

        uint64_t span = s1.timestamp_us - s0.timestamp_us;
        if (span == 0) return s0;

        float t    = (float)(us - s0.timestamp_us) / (float)span;
        float a0   = (float)s0.theta_rem_m;
        float a1   = (float)s1.theta_rem_m;
        float diff = a1 - a0;

        static constexpr float TWO_PI_Q14 = 102944.f;
        if (diff >  TWO_PI_Q14 * 0.5f) diff -= TWO_PI_Q14;
        if (diff < -TWO_PI_Q14 * 0.5f) diff += TWO_PI_Q14;

        AngleSample result;
        result.theta_rem_m  = (int32_t)(a0 + t * diff);
        result.timestamp_us = us;
        return result;
    }

private:
    AngleSample        buf_[DEPTH] = {};
    int                head_       = 0;
    int                count_      = 0;
    mutable std::mutex mtx_;
};
```

---

## `AngleSample` — the stored type

```cpp
struct AngleSample {
    int32_t  theta_rem_m;   // Q14 fixed-point radians
    uint64_t timestamp_us;  // microseconds since Pi boot
};
```

**Q14 fixed-point:** The value 16384 = 2^14 represents 1 radian. So:
- π radians = 3.14159 × 16384 ≈ 51,472 (raw int value)
- −π radians ≈ −51,472
- 2π radians ≈ 102,944 (this is `TWO_PI_Q14`)

We use integer storage for two reasons:
1. The MCU sends fixed-point integers; we preserve that representation all the way through
2. Integer comparison and arithmetic in the ring buffer are faster than float

---

## `push()` — the ring buffer write

```cpp
void push(int32_t theta, uint64_t us)
{
    std::lock_guard<std::mutex> lk(mtx_);
    buf_[head_] = { theta, us };
    head_ = (head_ + 1) % DEPTH;
    if (count_ < DEPTH) ++count_;
}
```

**The ring buffer mechanics:**

```
Initial state (count=0):
buf_: [_, _, _, _, _, _, ...]
       ^head=0

After push(A, t1) (count=1):
buf_: [A, _, _, _, _, _, ...]
          ^head=1

After push(B, t2), push(C, t3), push(D, t4) (count=4):
buf_: [A, B, C, D, _, _, ...]
                    ^head=4

When full (count=DEPTH=256):
buf_: [S0, S1, S2, ..., S255]
           ^head (oldest slot, about to be overwritten)
```

The newest sample is always at `buf_[(head_ - 1 + DEPTH) % DEPTH]`. The oldest, when full, is at `buf_[head_]`.

**`std::lock_guard<std::mutex> lk(mtx_)`:** The mutex ensures that `push` and `interpolate` don't execute simultaneously on different threads. Without this, one thread could be in the middle of writing `buf_[head_]` while the other thread is reading it — a data race, undefined behavior in C++.

**`mutable std::mutex mtx_`:** `mutable` allows the mutex to be locked even inside `const` member functions (`interpolate` is `const`). This is the standard pattern: the mutex is part of the implementation detail (synchronization), not the logical state of the object.

---

## Why `findNearest` was replaced by `interpolate`

`findNearest` returns whichever sample has the closest timestamp to the query:

```
Motor samples: A(t=0ms)  B(t=10ms)
Query at t=4ms → A (4ms away)
Query at t=6ms → B (4ms away — tie broken by which was found first)
Query at t=5ms → A or B (arbitrary at the exact midpoint)
```

For LiDAR blocks in the same packet (all queried within ~1.3ms of each other), they all get the *same* angle sample from `findNearest`. That's fine.

But between packets, there's a hard **Voronoi boundary** exactly at the midpoint between two motor samples:

```
Motor: ───────A────────────────B────────────────C──────►  time
Voronoi:       ├──── gets A ────┤──── gets B ────┤
                        ↑ hard step
```

When the LiDAR azimuth sweeps through this boundary, one packet gets angle A and the next gets angle B. If A and B differ by even 0.01° (one encoder count at the quantized level), the point cloud sees two slightly different rotations for adjacent scan lines. Points on a flat surface appear split — half slightly ahead, half slightly behind. This creates bidirectional smearing.

**`interpolate` fixes this** by smoothly blending between A and B:

```
Query at t=4ms: result = A + (4/10) * (B - A) = 0.6*A + 0.4*B
Query at t=6ms: result = A + (6/10) * (B - A) = 0.4*A + 0.6*B
```

Now adjacent packets get continuously varying angles — no hard step, no smearing.

---

## `interpolate()` — step by step

### Finding the chronological order of samples

```cpp
int start = (count_ < DEPTH) ? 0 : head_;
```

When the buffer isn't full yet, samples are in slots `[0, count_)` in order. When full, the oldest slot is `buf_[head_]` (the one that will be overwritten next). We iterate from oldest to newest so that the loop finds `beforeIdx` and `afterIdx` in chronological order.

### Finding the bracketing samples

```cpp
int beforeIdx = -1;
int afterIdx  = -1;
for (int i = 0; i < count_; ++i)
{
    int idx = (start + i) % DEPTH;
    if (buf_[idx].timestamp_us <= us)
        beforeIdx = idx;      // update as we find newer samples before `us`
    else if (afterIdx == -1)
    {
        afterIdx = idx;
        break;               // first sample after `us`; we're done
    }
}
```

Walking chronologically:
- Every sample with `timestamp <= us` could be `before` — we keep updating `beforeIdx` so we always end up with the *newest* sample that's still before `us`
- The first sample with `timestamp > us` is `after` — we stop immediately (samples are in order, so no later sample can be a better `after`)

**Edge cases:**

```cpp
if (beforeIdx == -1) return buf_[afterIdx];  // us is before all samples → clamp oldest
if (afterIdx  == -1) return buf_[beforeIdx]; // us is after all samples  → clamp newest
```

If the query time is outside the buffered range, we clamp rather than extrapolate. Extrapolation would amplify any error in the slope estimate.

### Linear interpolation

```cpp
uint64_t span = s1.timestamp_us - s0.timestamp_us;
if (span == 0) return s0;

float t    = (float)(us - s0.timestamp_us) / (float)span;
float a0   = (float)s0.theta_rem_m;
float a1   = (float)s1.theta_rem_m;
float diff = a1 - a0;
```

`t` is the fractional position of the query time between s0 and s1:
- `t = 0` → at s0 exactly → return a0
- `t = 1` → at s1 exactly → return a1
- `t = 0.4` → 40% of the way from s0 to s1

**Why float for the interpolation?** We need sub-integer precision. The Q14 values are integers (1 unit = 1/16384 rad ≈ 0.0035°), but the interpolated value can fall between two integers. We compute in float and round back to int32_t at the end.

### Angle wrap unwrapping

```cpp
static constexpr float TWO_PI_Q14 = 102944.f;   // 2π × 16384
if (diff >  TWO_PI_Q14 * 0.5f) diff -= TWO_PI_Q14;
if (diff < -TWO_PI_Q14 * 0.5f) diff += TWO_PI_Q14;
```

The motor angle is wrapped into [−π, +π) in Q14 units. Near the wrap point:
- s0 might be at +51471 (just below +π)
- s1 might be at −51471 (just above −π, after wrap)

Naive diff: −51471 − 51471 = −102942 ≈ −2π × Q14

But the motor only moved a tiny amount — it wrapped from just below +π to just above −π. The actual angular displacement is +2 units (0.012°), not −102942.

The unwrapping says: "if the difference is more than half a full circle, the short way around must be in the other direction." Subtracting TWO_PI_Q14 flips the −2π jump to a +2-unit jump.

This is the same idea as choosing whether to turn left or right for the shorter path.

### Final result

```cpp
AngleSample result;
result.theta_rem_m  = (int32_t)(a0 + t * diff);
result.timestamp_us = us;
return result;
```

`a0 + t * diff` is the interpolated angle. We cast back to `int32_t` (truncation is fine — 1 unit ≈ 0.0035° error). The result carries the *query* timestamp, not the sample timestamps, so the caller knows exactly what time this angle corresponds to.

---

## Why 256 slots?

At 100Hz loop rate, 256 samples = 2.56 seconds of history. The LiDAR thread queries the buffer for timestamps that are at most a few milliseconds old (network travel time). 256 slots is massive overkill — even 16 would be plenty. The extra depth provides safety margin against temporary slowdowns in the main loop (Linux is not a real-time OS; the scheduler can preempt the main loop for up to ~10ms).

**Lesson: In non-real-time embedded systems (Linux SBCs), always size buffers generously.** You're sharing the CPU with other processes. The motor control loop is "soft real-time" — it aims for 100Hz but might occasionally miss by 5–10ms. A deep buffer absorbs these hiccups without dropping angle data.

---

## Next: `08_concepts.md` — Summary table of embedded systems concepts used in this project
