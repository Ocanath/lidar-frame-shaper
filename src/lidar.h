#pragma once
#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <mutex>
#include <thread>
#include <atomic>
#include <cstdint>
#include <functional>
#include "angle_buffer.h"

class LidarSystem {
public:
    using Frame = std::vector<Eigen::Vector3f>;

    struct LidarPoint {
        Eigen::Vector3f pos;
        int32_t         motor_angle;    // wrap_2pi_14b result restored from 3 packed bytes
        uint16_t        azimuth_cdeg;
    };

    LidarSystem() = default;
    ~LidarSystem();

    bool connect(uint16_t port);
    void disconnect();
    bool isConnected() const { return running_; }

    // Decode a 1242-byte modified packet back to LidarPoints with motor angle
    static std::vector<LidarPoint> decodeModifiedPacket(const uint8_t* data, size_t len);

    // Thread-safe snapshot of all buffered frames (only populated when enableDecoding = true)
    std::vector<Frame> getFrames() const;

    // Returns true and consumes the flag if a new frame arrived since last call
    // (only fires when enableDecoding = true)
    bool pollNewFrame();

    // Latest packet timestamp (microseconds since top of hour), 0 if none received
    uint32_t latestTimestamp() const { return latestTimestamp_.load(); }

    // Settings — set before connect()
    int      bufferDepth       = 1024;
    int      maxPointsPerFrame = 2000;
    uint16_t port              = 2381;

    // Set before connect(). If null, motor_angle bytes in output are written as 0.
    AngleBuffer* angleBuffer = nullptr;

    // Called from the recv thread for each modified 1242-byte packet.
    std::function<void(const uint8_t*, size_t)> onPacketReady;

    // If true, also decode packets to Cartesian XYZ and populate the frames_ buffer.
    bool enableDecoding = false;

private:
    int                 socket_      = -1;
    std::thread         recvThread_;
    std::atomic<bool>   running_     = false;

    mutable std::mutex  framesMutex_;
    std::deque<Frame>   frames_;
    Frame               pending_;
    float               lastAzimuth_ = -1.f;
    std::atomic<bool>     newFrameFlag_    = false;
    std::atomic<uint32_t> latestTimestamp_ = 0;

    int64_t  epochOffsetUs_    = 0;
    bool     epochInitialized_ = false;

    void recvLoop();
    void decodePacket(const uint8_t* data, size_t len);
    void commitPending();
};
