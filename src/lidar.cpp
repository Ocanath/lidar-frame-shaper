#include "lidar.h"
#include "tick.h"
#include "trig_fixed.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <algorithm>

// VLP-16 vertical angles (degrees) for lasers 0–15 (interleaved firing order)
static constexpr float VERT_ANGLES[16] = {
    -15.f, 1.f, -13.f, 3.f, -11.f, 5.f, -9.f, 7.f,
     -7.f, 9.f,  -5.f, 11.f, -3.f, 13.f, -1.f, 15.f
};

// Single-return firing interval: 16 lasers × 2.304 µs each + 18.432 µs recharge = 55.296 µs
// Each VLP-16 data block holds two firing sequences, so inter-block interval is 2 × 55.296 = 110.592 µs
static constexpr uint32_t BLOCK_INTERVAL_US = 110592;

LidarSystem::~LidarSystem()
{
    disconnect();
}

bool LidarSystem::connect(uint16_t p)
{
    if (running_) disconnect();

    socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0)
    {
        perror("LidarSystem: socket");
        return false;
    }

    // 200 ms receive timeout so recvLoop can check running_
    struct timeval tv { 0, 200000 };
    setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(p);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (::bind(socket_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("LidarSystem: bind");
        ::close(socket_);
        socket_ = -1;
        return false;
    }

    running_    = true;
    recvThread_ = std::thread(&LidarSystem::recvLoop, this);
    return true;
}

void LidarSystem::disconnect()
{
    running_ = false;
    if (recvThread_.joinable())
        recvThread_.join();
    if (socket_ >= 0) {
        ::close(socket_);
        socket_ = -1;
    }
}

std::vector<LidarSystem::Frame> LidarSystem::getFrames() const
{
    std::lock_guard<std::mutex> lk(framesMutex_);
    return { frames_.begin(), frames_.end() };
}

bool LidarSystem::pollNewFrame()
{
    return newFrameFlag_.exchange(false);
}

// ── Decode a 1242-byte modified packet to LidarPoints ────────────────────────

std::vector<LidarSystem::LidarPoint> LidarSystem::decodeModifiedPacket(const uint8_t* data, size_t len)
{
    std::vector<LidarPoint> points;
    if (len < 1242) return points;

    for (int b = 0; b < 12; ++b)
    {
        const uint8_t* block = data + b * 103;
        if (block[0] != 0xFF || block[1] != 0xEE) continue;

        uint16_t az_raw     = static_cast<uint16_t>(block[2] | (block[3] << 8));
        int32_t  motor_angle = static_cast<int32_t>(
                                   static_cast<uint32_t>(block[4])
                                 | static_cast<uint32_t>(block[5]) << 8
                                 | static_cast<uint32_t>(block[6]) << 16);

        float az_rad = (az_raw / 100.f) * static_cast<float>(M_PI) / 180.f;

        for (int ch = 0; ch < 32; ++ch)
        {
            int            laser_id = ch % 16;
            const uint8_t* chan     = block + 7 + ch * 3;
            uint16_t dist_raw = static_cast<uint16_t>(chan[0] | (chan[1] << 8));
            if (dist_raw == 0) continue;

            float dist_m = dist_raw * 0.01f;
            float el_rad = VERT_ANGLES[laser_id] * static_cast<float>(M_PI) / 180.f;

            LidarPoint lp;
            lp.pos = Eigen::Vector3f(
                dist_m * cosf(el_rad) * sinf(az_rad),
                dist_m * cosf(el_rad) * cosf(az_rad),
                dist_m * sinf(el_rad)
            );
            lp.motor_angle  = motor_angle;
            lp.azimuth_cdeg = az_raw;
            points.push_back(lp);
        }
    }
    return points;
}

// ── Receive thread ────────────────────────────────────────────────────────────

void LidarSystem::recvLoop()
{
    uint8_t buf[1206];
    while (running_)
    {
        ssize_t received = ::recvfrom(socket_, buf, sizeof(buf), 0, nullptr, nullptr);
        if (received == 1206)
            decodePacket(buf, static_cast<size_t>(received));
        // On timeout (EAGAIN/EWOULDBLOCK), received < 0 — just loop and re-check running_
    }
}

// ── VLP-16 packet decoder ────────────────────────────────────────────────────
//
// Input packet layout (1206 bytes):
//   12 blocks × 100 bytes: [0xFF 0xEE][azimuth LE 2B][32 channels × 3B (dist LE 2B + intensity 1B)]
//   Footer: [timestamp 4B][return_mode 1B][model_id 1B]
//
// Output modified packet layout (1242 bytes):
//   12 blocks × 103 bytes: [0xFF 0xEE][azimuth LE 2B][motor_angle LE 3B][32 channels × 3B]
//   Footer: [timestamp 4B][return_mode 1B][model_id 1B]  (unchanged)

void LidarSystem::decodePacket(const uint8_t* data, size_t /*len*/)
{
    uint64_t recv_us = get_microsecond64();

    // Footer: bytes 1200–1203 = timestamp (µs since top of hour, little-endian)
    uint32_t lidar_ts = static_cast<uint32_t>(data[1200])
                      | static_cast<uint32_t>(data[1201]) << 8
                      | static_cast<uint32_t>(data[1202]) << 16
                      | static_cast<uint32_t>(data[1203]) << 24;
    latestTimestamp_ = lidar_ts;

    // ── Clock synchronization ─────────────────────────────────────────────────
    // epochOffsetUs_ maps lidar_ts (µs since top of hour) to program clock (µs since boot)
    // so that: program_us_of_event = lidar_ts_of_event + epochOffsetUs_
    {
        int64_t raw_offset = static_cast<int64_t>(recv_us) - static_cast<int64_t>(lidar_ts);
        if (!epochInitialized_)
        {
            epochOffsetUs_    = raw_offset;
            epochInitialized_ = true;
        }
        else
        {
            // Detect top-of-hour wrap: a sudden jump of ~3600 seconds
            int64_t delta = raw_offset - epochOffsetUs_;
            if (delta < -1800000000LL)       raw_offset += 3600000000LL;
            else if (delta > 1800000000LL)   raw_offset -= 3600000000LL;
            // EMA: alpha ≈ 1/100
            epochOffsetUs_ += (raw_offset - epochOffsetUs_) / 100;
        }
    }

    // ── Build modified 1242-byte packet ──────────────────────────────────────
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

        // Motor angle: look up sample nearest to this block's inferred program timestamp
        int32_t angle = 0;
        if (angleBuffer != nullptr)
        {
            uint64_t block_prog_us = static_cast<uint64_t>(
                static_cast<int64_t>(lidar_ts + static_cast<uint32_t>(b) * BLOCK_INTERVAL_US)
                + epochOffsetUs_);
            AngleSample sample = angleBuffer->findNearest(block_prog_us);
            angle = wrap_2pi_14b(sample.theta_rem_m);
        }

        // Write modified block
        outblock[0] = 0xFF;
        outblock[1] = 0xEE;
        outblock[2] = block[2];   // azimuth low byte
        outblock[3] = block[3];   // azimuth high byte
        outblock[4] = static_cast<uint8_t>((angle >>  0) & 0xFF);
        outblock[5] = static_cast<uint8_t>((angle >>  8) & 0xFF);
        outblock[6] = static_cast<uint8_t>((angle >> 16) & 0xFF);
        memcpy(outblock + 7, block + 4, 96);  // 32 channels unchanged

        uint16_t az_raw        = static_cast<uint16_t>(block[2] | (block[3] << 8));
        float    azimuth_block = static_cast<float>(az_raw);

        // ── Optional Cartesian decode ─────────────────────────────────────────
        if (enableDecoding)
        {
            for (int ch = 0; ch < 32; ++ch)
            {
                int            laser_id = ch % 16;
                const uint8_t* chan     = block + 4 + ch * 3;
                uint16_t dist_raw = static_cast<uint16_t>(chan[0] | (chan[1] << 8));
                if (dist_raw == 0) continue;

                float dist_m = dist_raw * 0.01f;
                float az_rad = (azimuth_block / 100.f) * static_cast<float>(M_PI) / 180.f;
                float el_rad = VERT_ANGLES[laser_id] * static_cast<float>(M_PI) / 180.f;

                pending_.push_back(Eigen::Vector3f(
                    dist_m * cosf(el_rad) * sinf(az_rad),
                    dist_m * cosf(el_rad) * cosf(az_rad),
                    dist_m * sinf(el_rad)
                ));
            }
        }

        // Frame boundary detection always runs so pollNewFrame() works in routing-only mode
        if (lastAzimuth_ > 18000.f && azimuth_block < lastAzimuth_)
        {
            if (enableDecoding) commitPending();
            else                newFrameFlag_ = true;
        }
        lastAzimuth_ = azimuth_block;
    }

    // Copy footer unchanged
    memcpy(outbuf + 12 * 103, data + 1200, 6);

    if (onPacketReady)
        onPacketReady(outbuf, 1242);
}

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
