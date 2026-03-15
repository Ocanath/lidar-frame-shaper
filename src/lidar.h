#pragma once
#include <glm/glm.hpp>
#include <vector>
#include <deque>
#include <mutex>
#include <thread>
#include <atomic>
#include <cstdint>

class LidarSystem {
public:
    using Frame = std::vector<glm::vec3>;

    LidarSystem() = default;
    ~LidarSystem();

    bool connect(uint16_t port);
    void disconnect();
    bool isConnected() const { return running_; }

    // Thread-safe snapshot of all buffered frames
    std::vector<Frame> getFrames() const;

    // Returns true and consumes the flag if a new frame arrived since last call
    bool pollNewFrame();

    // Settings (read/write from main thread only, before connect)
    int      bufferDepth       = 1024;
    int      maxPointsPerFrame = 2000;
    uint16_t port              = 2381;

private:
    int                 socket_      = -1;   // POSIX UDP socket fd, -1 = invalid
    std::thread         recvThread_;
    std::atomic<bool>   running_     = false;

    mutable std::mutex  framesMutex_;
    std::deque<Frame>   frames_;             // circular buffer, max bufferDepth entries
    Frame               pending_;            // accumulating current scan
    float               lastAzimuth_ = -1.f;
    std::atomic<bool>   newFrameFlag_ = false;

    void recvLoop();
    void decodePacket(const uint8_t* data, size_t len);
    void commitPending();
};
