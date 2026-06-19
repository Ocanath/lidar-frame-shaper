#pragma once

#include <stdint.h>
#include <mutex>

struct AngleSample {
    int32_t  theta_rem_m;
    uint64_t timestamp_us;
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

    // Returns the sample with the closest timestamp — used as a fallback.
    AngleSample findNearest(uint64_t us) const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (count_ == 0) return {};
        AngleSample best = buf_[0];
        uint64_t bestDist = us >= buf_[0].timestamp_us
                          ? us - buf_[0].timestamp_us
                          : buf_[0].timestamp_us - us;
        for (int i = 1; i < count_; ++i) {
            uint64_t d = us >= buf_[i].timestamp_us
                       ? us - buf_[i].timestamp_us
                       : buf_[i].timestamp_us - us;
            if (d < bestDist) {
                bestDist = d;
                best = buf_[i];
            }
        }
        return best;
    }

    // Linearly interpolates between the two samples that bracket `us`.
    // This eliminates the hard snap discontinuity of findNearest, which causes
    // blocks near the midpoint between two motor samples to get different angles
    // and produce bidirectional smearing in the point cloud.
    //
    // Falls back to the nearest sample if `us` is outside the buffered range.
    AngleSample interpolate(uint64_t us) const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (count_ == 0) return {};
        if (count_ == 1) return buf_[0];

        // Samples are stored in insertion (chronological) order.
        // When buffer is full, oldest is at buf_[head_].
        int start = (count_ < DEPTH) ? 0 : head_;

        // Walk chronologically to find the last sample before us (before)
        // and the first sample after us (after).
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
                break;  // samples are chronological, no need to continue
            }
        }

        // us is before all buffered samples — clamp to oldest
        if (beforeIdx == -1) return buf_[afterIdx];
        // us is after all buffered samples — clamp to newest
        if (afterIdx  == -1) return buf_[beforeIdx];

        const AngleSample& s0 = buf_[beforeIdx];
        const AngleSample& s1 = buf_[afterIdx];

        uint64_t span = s1.timestamp_us - s0.timestamp_us;
        if (span == 0) return s0;

        float t  = (float)(us - s0.timestamp_us) / (float)span;

        float a0   = (float)s0.theta_rem_m;
        float a1   = (float)s1.theta_rem_m;
        float diff = a1 - a0;

        // Unwrap: consecutive samples are tiny fractions of a revolution apart.
        // A jump larger than ±π in Q14 means the angle wrapped at ±π.
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
