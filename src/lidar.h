#pragma once
#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <mutex>
#include <thread>
#include <atomic>
#include <cstdint>
#include <functional>

class LidarSystem {
public:
    using Frame = std::vector<Eigen::Vector3f>;

    LidarSystem() = default;
    ~LidarSystem();

    bool connect(uint16_t port);
    void disconnect();
    bool isConnected() const { return running_; }

    // Thread-safe snapshot of all buffered frames
    std::vector<Frame> getFrames() const;

    // Returns true and consumes the flag if a new frame arrived since last call
    bool pollNewFrame();

    // Latest packet timestamp (microseconds since top of hour), 0 if none received
    uint32_t latestTimestamp() const { return latestTimestamp_.load(); }

    // Settings (read/write from main thread only, before connect)
    int      bufferDepth       = 1024;
    int      maxPointsPerFrame = 2000;
    uint16_t port              = 2381;

    // Optional callback — called from the receive thread each time a raw
    // 1206-byte packet arrives.  Set this before calling connect().
    // main.cpp uses it to inject the motor angle and forward the packet.
    std::function<void(const uint8_t*, size_t)> onRawPacket;

private:
    int                 socket_      = -1;   // POSIX UDP socket fd, -1 = invalid
    std::thread         recvThread_;
    std::atomic<bool>   running_     = false;

    mutable std::mutex  framesMutex_;
    std::deque<Frame>   frames_;             // circular buffer, max bufferDepth entries
    Frame               pending_;            // accumulating current scan
    float               lastAzimuth_ = -1.f;
    std::atomic<bool>     newFrameFlag_     = false;
    std::atomic<uint32_t> latestTimestamp_  = 0;

    void recvLoop();
    void decodePacket(const uint8_t* data, size_t len);
    void commitPending();
};
