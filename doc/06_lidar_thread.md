# 06 — The LiDAR Receive Thread: Clock Sync, Per-Block Timestamping, Packet Injection

Source: `src/lidar.cpp` (frame shaper version)

---

## Architecture overview

```
Main thread                              LiDAR receive thread (recvLoop)
────────────────                         ───────────────────────────────
angleBuffer.push(angle, us)   ──────►   angleBuffer.interpolate(block_us)
                                         │
                                         ▼
                                         decodePacket()
                                         ├─ clock sync (EMA)
                                         ├─ per-block timestamp
                                         ├─ angle lookup
                                         └─ inject → onPacketReady()
```

Two threads share one `AngleBuffer`. The main thread pushes motor angles at ~100Hz. The LiDAR thread reads (interpolates) them at up to 750 Hz (12 blocks × ~62.5 packets/sec). The `AngleBuffer` mutex makes this thread-safe.

---

## `recvLoop()` — blocking UDP receive

```cpp
void LidarSystem::recvLoop()
{
    uint8_t buf[1206];
    while (running_)
    {
        ssize_t received = ::recvfrom(socket_, buf, sizeof(buf), 0, nullptr, nullptr);
        if (received == 1206)
            decodePacket(buf, static_cast<size_t>(received));
    }
}
```

`recvfrom` is a **blocking system call** — the thread sleeps inside the OS kernel until a UDP packet arrives. When a 1206-byte packet arrives (the standard VLP-16 size), it immediately calls `decodePacket`.

**Why blocking I/O in an embedded receiver?** Alternatives are:
- **Polling**: call recvfrom with `MSG_DONTWAIT`, loop burning CPU. Wastes power, interferes with other threads.
- **Async/callback (epoll/select)**: complex, overkill for a single socket.
- **Blocking in its own thread**: simple. The thread does exactly one thing — wait for packets and process them. CPU is zero when no packets arrive.

The socket has a 200ms receive timeout so the `while (running_)` check runs at least every 200ms — the thread can exit cleanly when `running_` is set to false.

**Lesson:** Blocking I/O in a dedicated thread is the standard UNIX pattern for network receivers. The OS scheduler parks the thread at zero CPU cost until data arrives. This is more efficient than any polling scheme.

---

## `decodePacket()` — the full pipeline

### Step 1: Read the footer timestamp

```cpp
uint32_t lidar_ts = static_cast<uint32_t>(data[1200])
                  | static_cast<uint32_t>(data[1201]) << 8
                  | static_cast<uint32_t>(data[1202]) << 16
                  | static_cast<uint32_t>(data[1203]) << 24;
latestTimestamp_ = lidar_ts;
```

The last 6 bytes of a VLP-16 packet are the footer. Bytes 1200–1203 contain the VLP-16's own timestamp: **microseconds since the top of the current hour**, little-endian.

**Little-endian assembly:** The VLP-16 stores multi-byte numbers with the least-significant byte first (little-endian, like x86 CPUs). To reconstruct the number:
```
byte[1200] = low 8 bits
byte[1201] = bits 8–15
byte[1202] = bits 16–23
byte[1203] = bits 24–31
```

Shifting each byte to its proper position and OR-ing them together reconstructs the 32-bit value. The `static_cast<uint32_t>` before each shift is critical — without it, shifting an `uint8_t` left by 24 bits would be undefined behavior in C++ (integer promotions make it a signed `int`, and shifting into the sign bit is UB).

---

### Step 2: Clock synchronization (EMA)

```cpp
int64_t raw_offset = static_cast<int64_t>(recv_us) - static_cast<int64_t>(lidar_ts);
if (!epochInitialized_)
{
    epochOffsetUs_    = raw_offset;
    epochInitialized_ = true;
}
else
{
    int64_t delta = raw_offset - epochOffsetUs_;
    if (delta < -1800000000LL)       raw_offset += 3600000000LL;
    else if (delta > 1800000000LL)   raw_offset -= 3600000000LL;
    epochOffsetUs_ += (raw_offset - epochOffsetUs_) / 100;
}
```

**The two-clock problem:**

| Clock | What it measures | Reset when |
|-------|-----------------|------------|
| VLP-16 internal | µs since top of current hour | Every 3600 seconds |
| Pi CLOCK_BOOTTIME | µs since Pi boot | On reboot |

To convert a VLP-16 timestamp into a Pi timestamp (so we can look up the motor angle):
```
pi_us = lidar_ts + epochOffsetUs_
```

We need `epochOffsetUs_`. Since the packet arrives shortly after it was generated, we can estimate it:
```
raw_offset = recv_us - lidar_ts
```

This is approximate because of network travel time (~0.1ms on LAN), but for stamping point cloud data, 0.1ms accuracy is plenty.

**EMA (Exponentially Weighted Moving Average):**

```cpp
epochOffsetUs_ += (raw_offset - epochOffsetUs_) / 100;
```

This is the EMA with α = 1/100 = 0.01:
```
new_estimate = 0.99 * old_estimate + 0.01 * new_measurement
```

A large α responds quickly but is noisy. A small α (like 0.01) changes slowly — good for tracking a clock drift that changes at milliseconds per minute. The filter rejects sudden large jumps caused by network jitter.

**Top-of-hour wrap detection:**

```cpp
if (delta < -1800000000LL)       raw_offset += 3600000000LL;
else if (delta > 1800000000LL)   raw_offset -= 3600000000LL;
```

Every hour, `lidar_ts` resets from ~3,599,999,999 to ~0. This makes `raw_offset` jump by −3,600,000,000. The code detects a jump larger than ±1,800,000,000 (±30 minutes) and unwraps it by adding or subtracting 3,600,000,000. This keeps the EMA from "chasing" the wrap.

---

### Step 3: Build the modified packet — outer block loop

```cpp
uint8_t outbuf[1242];

for (int b = 0; b < 12; ++b)
{
    const uint8_t* block    = data + b * 100;
    uint8_t*       outblock = outbuf + b * 103;

    if (block[0] != 0xFF || block[1] != 0xEE)
    {
        memset(outblock, 0, 103);
        continue;
    }
```

A VLP-16 packet has 12 data blocks. Each raw block is 100 bytes; each output block will be 103 bytes (3 extra for motor angle).

The magic bytes `0xFF 0xEE` are the VLP-16 block flag — every valid block starts with them. If a block doesn't have this header, it's garbage or padding; we zero-fill the output block and skip.

**Lesson: Always validate framing bytes before processing binary data.** Without this check, we might try to decode an uninitialized block and produce garbage points.

---

### Step 4: Per-block timestamp and angle lookup

```cpp
uint64_t block_prog_us = static_cast<uint64_t>(
    static_cast<int64_t>(lidar_ts + static_cast<uint32_t>(b) * BLOCK_INTERVAL_US)
    + epochOffsetUs_);
AngleSample sample = angleBuffer->interpolate(block_prog_us);
angle = wrap_2pi_14b(sample.theta_rem_m);
```

**Why per-block timestamps?**

The VLP-16 fires 12 blocks per packet. Each block contains two firing sequences of 16 lasers. The firing sequences are not simultaneous — they happen sequentially, each separated by 55.296µs. The inter-block interval is therefore:

```
BLOCK_INTERVAL_US = 2 × 55.296µs = 110,592µs ≈ 110.6ms
```

Wait — that seems large for a single packet. Let me clarify: the *entire packet* spans 12 × 110.592µs ≈ 1.327ms. The packet *rate* is ~1/(1.327ms) ≈ 754 Hz... but the VLP-16 actually fires at 600 RPM spinning rate, and sends 24 firing sequences per packet. The actual calculation from the VLP-16 manual gives 110,592µs between the *azimuth reference times* of consecutive blocks within a packet.

The motor turns at ~0.5 RPM = 3°/sec. In 110.592µs, it moves:
```
3° / 1,000,000 µs × 110,592 µs = 0.000332°
```

This is negligible — neighboring blocks in the same packet get essentially the same motor angle. The per-block timestamping is still correct and adds no meaningful overhead, and it ensures accuracy if the motor is ever sped up.

**`angleBuffer->interpolate(block_prog_us)`:**

Looks up the motor angle at time `block_prog_us` by finding the two motor angle samples that bracket this timestamp and linearly interpolating between them. See `doc/07_angle_buffer.md` for details on the interpolation algorithm.

---

### Step 5: Inject motor angle bytes into the output block

```cpp
outblock[0] = 0xFF;
outblock[1] = 0xEE;
outblock[2] = block[2];   // azimuth low byte
outblock[3] = block[3];   // azimuth high byte
outblock[4] = static_cast<uint8_t>((angle >>  0) & 0xFF);
outblock[5] = static_cast<uint8_t>((angle >>  8) & 0xFF);
outblock[6] = static_cast<uint8_t>((angle >> 16) & 0xFF);
memcpy(outblock + 7, block + 4, 96);  // 32 channels unchanged
```

The output block layout:
```
Bytes 0–1:  Flag [0xFF 0xEE]
Bytes 2–3:  Azimuth (from VLP-16, unchanged)
Bytes 4–6:  Motor angle (3 bytes = 24 bits, little-endian, Q14 fixed-point)
Bytes 7–102: 32 channels (dist LE 2B + intensity 1B each) — copied unchanged
```

The motor angle is a signed 24-bit integer stored little-endian. We extract the three bytes with masks and right-shifts. The viewer reconstructs it:
```cpp
int32_t motor_angle = (raw & 0x800000) ? (int32_t)(raw | 0xFF000000) : (int32_t)raw;
```
(sign-extend from 24 to 32 bits by checking bit 23)

**`memcpy(outblock + 7, block + 4, 96)`:**

The 32-channel payload (96 bytes) is copied byte-for-byte from the raw block (offset 4 in raw, since the raw block has only 4 bytes of header) to the modified block (offset 7, since the modified block has 7 bytes of header). This is the most efficient possible copy — a single `memcpy` call rather than a loop.

---

### Step 6: Optional Cartesian decode

```cpp
if (enableDecoding)
{
    for (int ch = 0; ch < 32; ++ch)
    {
        int laser_id    = ch % 16;
        uint16_t dist_raw = static_cast<uint16_t>(chan[0] | (chan[1] << 8));
        if (dist_raw == 0) continue;

        float dist_m = dist_raw * 0.01f;
        float az_rad = (azimuth_block / 100.f) * (float)M_PI / 180.f;
        float el_rad = VERT_ANGLES[laser_id] * (float)M_PI / 180.f;

        pending_.push_back(Eigen::Vector3f(
            dist_m * cosf(el_rad) * sinf(az_rad),
            dist_m * cosf(el_rad) * cosf(az_rad),
            dist_m * sinf(el_rad)
        ));
    }
}
```

**Spherical to Cartesian conversion:**

The VLP-16 gives each point as `(azimuth, elevation_angle, distance)` — spherical coordinates. To convert to Cartesian (X, Y, Z):

```
X = dist × cos(elevation) × sin(azimuth)    ← horizontal offset
Y = dist × cos(elevation) × cos(azimuth)    ← forward offset
Z = dist × sin(elevation)                   ← vertical offset
```

`VERT_ANGLES[laser_id]` is the fixed elevation angle for each of the 16 laser beams:
```
{-15°, +1°, -13°, +3°, -11°, +5°, -9°, +7°, -7°, +9°, -5°, +11°, -3°, +13°, -1°, +15°}
```

The interleaved pattern (-15, +1, -13, +3, ...) comes from the physical arrangement of the VLP-16's laser diodes — they fire in pairs with alternating angles to maximize firing rate while minimizing inter-channel optical crosstalk.

`ch % 16` maps channel 0–31 to laser 0–15 because the VLP-16 fires two groups of 16 in each block (dual-return mode support), and both firings use the same 16 elevation angles.

**`azimuth_block / 100.f`:** VLP-16 azimuth is stored in centidegrees (hundredths of degrees) as a uint16, giving 0.01° resolution over 0–359.99°. Dividing by 100 converts to degrees.

---

### Step 7: Frame boundary detection

```cpp
if (lastAzimuth_ > 18000.f && azimuth_block < lastAzimuth_)
{
    if (enableDecoding) commitPending();
    else                newFrameFlag_ = true;
}
lastAzimuth_ = azimuth_block;
```

The VLP-16 spins at 600 RPM and the azimuth goes 0° → 359.99° → 0° → ... We detect the wrap by checking: "did we just go from above 180° back to something lower?" This indicates a full 360° revolution has completed.

When a frame completes:
- In decode mode: `commitPending()` moves the accumulated points into the ring buffer for the main thread to consume
- In routing-only mode: `newFrameFlag_` is set so `pollNewFrame()` returns true

**Why 18000.f (180°)?** We require the azimuth to have passed the halfway point (18000 centidegrees = 180°) before allowing the wrap detection to trigger. This prevents false triggers from minor azimuth jitter near 0°.

---

### Step 8: Footer copy and packet dispatch

```cpp
memcpy(outbuf + 12 * 103, data + 1200, 6);

if (onPacketReady)
    onPacketReady(outbuf, 1242);
```

Copy the 6-byte footer (timestamp + return mode + model ID) from the raw packet to the end of the modified packet, then call the callback (which sends the packet to the viewer via UDP).

**`if (onPacketReady)`:** Check that the callback is not null before calling it. Safe default — if no callback is registered, packets are discarded silently.

---

## `commitPending()` — moving points to the frame buffer

```cpp
void LidarSystem::commitPending()
{
    if (pending_.empty()) return;

    Frame frame;
    if (static_cast<int>(pending_.size()) <= maxPointsPerFrame)
    {
        frame = std::move(pending_);
    }
    else
    {
        frame.reserve(maxPointsPerFrame);
        float stride = static_cast<float>(pending_.size()) / static_cast<float>(maxPointsPerFrame);
        for (int i = 0; i < maxPointsPerFrame; ++i)
            frame.push_back(pending_[static_cast<size_t>(i * stride)]);
    }
    pending_.clear();

    {
        std::lock_guard<std::mutex> lk(framesMutex_);
        frames_.push_back(std::move(frame));
        while (static_cast<int>(frames_.size()) > bufferDepth)
            frames_.pop_front();
    }
    newFrameFlag_ = true;
}
```

**`std::move(pending_)`:** Transfers ownership of the vector's internal memory to `frame` without copying. `pending_` becomes empty. This is O(1) — no data is copied.

**Decimation:** If the frame has more points than `maxPointsPerFrame`, we stride-sample: take every `stride`-th point. The stride is computed so exactly `maxPointsPerFrame` points are selected, evenly spaced across the raw data.

**The ring buffer of frames:** `frames_` is a `std::deque` (double-ended queue). New frames go to the back; if it's full, the oldest frame is popped from the front. This is a ring buffer of frames, ensuring the viewer always has the most recent scan without unbounded memory growth.

**`std::lock_guard<std::mutex>`:** RAII mutex lock. Acquired when `lk` is constructed, released automatically when `lk` goes out of scope at the `}`. No need to manually unlock — the lock is always released, even if an exception is thrown.

---

## Next: `07_angle_buffer.md` — The interpolating angle ring buffer in detail
