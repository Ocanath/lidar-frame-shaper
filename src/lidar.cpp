#include "lidar.h"
#include <glm/gtc/constants.hpp>
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

LidarSystem::~LidarSystem()
{
    disconnect();
}

bool LidarSystem::connect(uint16_t p)
{
    if (running_) disconnect();

    socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0) {
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

// ── Receive thread ────────────────────────────────────────────────────────────

void LidarSystem::recvLoop()
{
    uint8_t buf[1206];
    while (running_) {
        ssize_t received = ::recvfrom(socket_, buf, sizeof(buf), 0, nullptr, nullptr);
        if (received == 1206)
            decodePacket(buf, (size_t)received);
        // On timeout (EAGAIN/EWOULDBLOCK), received < 0 — just loop and re-check running_
    }
}

// ── VLP-16 inline decoder ────────────────────────────────────────────────────
//
// Packet layout (1206 bytes):
//   12 blocks × 100 bytes: [0xFF 0xEE][azimuth LE 2B][32 channels × 3B (dist LE 2B + intensity 1B)]
//   Footer: [timestamp 4B][return_mode 1B][model_id 1B]

void LidarSystem::decodePacket(const uint8_t* data, size_t /*len*/)
{
    for (int b = 0; b < 12; ++b) 
	{
        const uint8_t* block = data + b * 100;
        // Sanity: block sync bytes should be 0xFF, 0xEE
        if (block[0] != 0xFF || block[1] != 0xEE) continue;

        uint16_t az_raw  = static_cast<uint16_t>(block[2] | (block[3] << 8));
        float    azimuth = static_cast<float>(az_raw); // centidegrees [0, 35999]

        for (int ch = 0; ch < 32; ++ch) 
		{
            int           laser_id = ch % 16;
            const uint8_t* chan    = block + 4 + ch * 3;
            uint16_t dist_raw = static_cast<uint16_t>(chan[0] | (chan[1] << 8));
            if (dist_raw == 0) continue;

            float dist_m = dist_raw * 0.01f;
            float az_rad = glm::radians(azimuth / 100.f);
            float el_rad = glm::radians(VERT_ANGLES[laser_id]);

            pending_.push_back({
                dist_m * cosf(el_rad) * sinf(az_rad),
                dist_m * cosf(el_rad) * cosf(az_rad),
                dist_m * sinf(el_rad)
            });
        }

        // Frame boundary: azimuth wrapped from ~35999 back toward 0
        if (lastAzimuth_ > 18000.f && azimuth < lastAzimuth_)
		{
			commitPending();
		}
        lastAzimuth_ = azimuth;
    }
}

void LidarSystem::commitPending()
{
    if (pending_.empty()) return;

    Frame frame;
    if ((int)pending_.size() <= maxPointsPerFrame) 
	{
        frame = std::move(pending_);
    } 
	else 
	{
        frame.reserve(maxPointsPerFrame);
        float stride = static_cast<float>(pending_.size()) / static_cast<float>(maxPointsPerFrame);
        for (int i = 0; i < maxPointsPerFrame; ++i)
		{
            frame.push_back(pending_[static_cast<size_t>(i * stride)]);
		}
    }
    pending_.clear();

    {
        std::lock_guard<std::mutex> lk(framesMutex_);
        frames_.push_back(std::move(frame));
        while ((int)frames_.size() > bufferDepth)
		{
            frames_.pop_front();
		}
    }
    newFrameFlag_ = true;
}
