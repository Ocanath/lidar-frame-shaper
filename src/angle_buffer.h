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

private:
    AngleSample        buf_[DEPTH] = {};
    int                head_       = 0;
    int                count_      = 0;
    mutable std::mutex mtx_;
};
