# 02 — Zero-Phase IIR Filter (filtfilt)

Source: `src/main.cpp` lines 46–90

---

## The code

```cpp
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

    // Backward pass
    float z[FILTFILT_N];
    z[N-1] = y[N-1];
    for (int i = N-2; i >= 0; --i)
        z[i] = alpha * z[i+1] + (1.f - alpha) * y[i];

    return z[N-1];
}
```

---

## Why we need a filter at all

The motor controller reports angle as a fixed-point integer (`theta_rem_m`). The encoder has finite resolution — if the encoder has, say, 4096 counts per revolution, each count is about 0.088°. At low RPM (~0.5 RPM = 3°/sec), the motor barely moves between loop iterations (~10ms), so the reported angle sits at one count for several loops, then jumps to the next count.

This **quantization staircase** is noise. We push the angles into `AngleBuffer` and the LiDAR thread looks them up to stamp each packet block. If the angles are noisy, the point cloud has visible steps and ripples. Smoothing removes those artifacts.

---

## The ring buffer

```cpp
static float filtfiltBuf[FILTFILT_N] = {};  // N=16 slots
static int   filtfiltHead  = 0;             // where next write goes
static int   filtfiltCount = 0;             // how many valid entries so far
```

**Lesson: Ring buffers (circular buffers) are the most common data structure in embedded systems.** They allow you to keep a sliding window of recent data with:
- O(1) push (no shifting)
- Fixed memory (no heap allocation)
- Simple index arithmetic: `(head + 1) % N`

`filtfiltBuf` holds the last 16 angle samples. When a new sample arrives, it overwrites the oldest slot. The buffer never grows or shrinks — it has exactly 16 floats at all times.

### The modulo trick

```cpp
filtfiltHead = (filtfiltHead + 1) % FILTFILT_N;
```

When `filtfiltHead` reaches 15 (the last slot), adding 1 gives 16. `16 % 16 = 0`. The index wraps back to zero. This is the core ring buffer idiom.

### The `filtfiltCount` guard

```cpp
if (filtfiltCount < FILTFILT_N) ++filtfiltCount;
```

On startup the buffer is empty. We track how many entries are actually valid so the filter doesn't read uninitialized memory during the first 16 iterations.

---

## The IIR (first-order low-pass) filter

IIR = **Infinite Impulse Response**. Despite the name, this is just a weighted average:

```
y[i] = alpha * y[i-1]  +  (1 - alpha) * x[i]
```

Where:
- `x[i]` = current raw sample
- `y[i-1]` = previous filtered output
- `alpha` = how much to remember the past (0 = no filter, 1 = never changes)

At `alpha = 0.15`:
- New sample weight: `1 - 0.15 = 0.85` (strongly follows the input)
- History weight: `0.15` (light smoothing)

This is a very mild filter — it just takes the sharp edges off the quantization jumps without introducing much lag.

**Lesson:** An IIR filter on a microcontroller or embedded system is almost always a first-order filter like this. It's one multiply, one add, and one store per sample. Higher-order filters (FIR) require storing and summing many taps — more memory and more cycles. First-order IIR is a 90% solution with 1% of the cost.

---

## The forward–backward pass (why it's called "filtfilt")

A regular IIR filter introduces **phase lag** — the output lags behind the input because every output depends on past inputs. At α=0.15 the lag is small, but it exists.

The classic fix is to filter *forward then backward*:
1. **Forward pass**: filter oldest→newest → produces `y[]`
2. **Backward pass**: filter newest→oldest on `y[]` → produces `z[]`

The backward pass reverses whatever lag the forward pass introduced, giving **zero net phase shift**.

```
Forward:   y[i] = α·y[i-1] + (1-α)·x[i]      oldest → newest
Backward:  z[i] = α·z[i+1] + (1-α)·y[i]      newest → oldest
```

We return `z[N-1]`, which is the zero-phase filtered value at the **newest** (most recent) sample.

### The bug: `z[N-1]` is never modified

```cpp
z[N-1] = y[N-1];           // seed: newest backward value = newest forward value
for (int i = N-2; i >= 0; --i)   // loop starts at N-2, never touches N-1
    z[i] = alpha * z[i+1] + (1.f - alpha) * y[i];

return z[N-1];  // ← this is just y[N-1], never touched by the backward pass
```

The backward pass updates `z[0]` through `z[N-2]` but `z[N-1]` stays equal to `y[N-1]`. So `filtfiltRead` actually returns the **forward IIR output at the newest sample** — not a true zero-phase result.

**In practice this doesn't matter** at 0.34 RPM because:
- The lag at α=0.15 is only ~1.5 samples ≈ 15ms
- Motor angle changes ~0.03° in 15ms at 0.34 RPM
- This is far smaller than the encoder quantization step

The filter is doing its actual job (smoothing out encoder quantization steps) correctly; it's just not truly zero-phase. It's documented here so you understand what the code actually does vs. what the comments claim.

---

## Parameter choices

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `FILTFILT_N` | 16 | 16 samples at ~100Hz loop = 160ms of history |
| `FILTFILT_ALPHA` | 0.15 | Mild smoothing; was 0.5 (caused visible oscillation artifacts) |

With α=0.5, the filter was "ringing" on the quantization steps — when the angle snapped from one count to the next, the IIR would overshoot and undershoot, creating bidirectional smearing in the point cloud. Reducing to 0.15 kept the filter gentle enough to not ring.

---

## Next: `03_pid_config.md` — Fixed-point PID gains and motor controller architecture
