#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace imu_safety
{
constexpr double kMaxInterval = 0.1;  // Do not integrate across a missing IMU segment.
constexpr std::size_t kMaxQueueSize = 20000;

inline bool validInterval(double dt)
{
    return std::isfinite(dt) && dt > 0.0 && dt <= kMaxInterval;
}

class TimestampTracker
{
public:
    enum class Status { First, Valid, Gap, Invalid };

    Status observe(double stamp, double& dt)
    {
        dt = 0.0;
        if (!std::isfinite(stamp) || stamp < 0.0 || stamp <= last_stamp_)
            return Status::Invalid;
        if (last_stamp_ < 0.0)
        {
            last_stamp_ = stamp;
            return Status::First;
        }
        dt = stamp - last_stamp_;
        last_stamp_ = stamp;
        return validInterval(dt) ? Status::Valid : Status::Gap;
    }

private:
    double last_stamp_ = -1.0;
};

// Dynamically sized rotation history: a high-rate scan must not overflow a
// fixed 500-sample array. Its input queue is bounded separately by the caller.
class RotationHistory
{
public:
    struct Sample { double time, x, y, z; };

    void clear() { samples_.clear(); }
    std::size_t size() const { return samples_.size(); }

    bool append(double stamp, double wx, double wy, double wz)
    {
        if (!std::isfinite(stamp) || stamp < 0.0 || !std::isfinite(wx) ||
            !std::isfinite(wy) || !std::isfinite(wz))
            return false;
        Sample sample{stamp, 0.0, 0.0, 0.0};
        if (!samples_.empty())
        {
            const auto& previous = samples_.back();
            const double dt = stamp - previous.time;
            if (!validInterval(dt))
                return false;
            sample.x = previous.x + wx * dt;
            sample.y = previous.y + wy * dt;
            sample.z = previous.z + wz * dt;
            if (!std::isfinite(sample.x) || !std::isfinite(sample.y) || !std::isfinite(sample.z))
                return false;
        }
        samples_.push_back(sample);
        return true;
    }

    bool interpolate(double stamp, float& x, float& y, float& z) const
    {
        x = y = z = 0.0f;
        if (samples_.empty() || !std::isfinite(stamp))
            return false;
        auto next = std::lower_bound(samples_.begin(), samples_.end(), stamp,
            [](const Sample& sample, double time) { return sample.time < time; });
        if (next == samples_.end())
            next = samples_.end() - 1;
        if (next == samples_.begin() || stamp >= next->time)
        {
            x = next->x; y = next->y; z = next->z;
        }
        else
        {
            const auto& previous = *(next - 1);
            const double weight = (stamp - previous.time) / (next->time - previous.time);
            x = previous.x + weight * (next->x - previous.x);
            y = previous.y + weight * (next->y - previous.y);
            z = previous.z + weight * (next->z - previous.z);
        }
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }

private:
    std::vector<Sample> samples_;
};
}  // namespace imu_safety