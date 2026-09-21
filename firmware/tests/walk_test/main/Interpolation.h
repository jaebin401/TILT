#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace tilt_walk_test {

class Interpolation {
public:
    // A changed target starts a new linear ramp from the value reached now.
    float go(float target, std::uint32_t duration_ms, std::uint32_t now_ms) {
        if (!initialized_) {
            reset(target, now_ms);
            return current_;
        }

        sample(now_ms);
        if (std::fabs(target - target_value_) > 0.0001f) {
            start_value_ = current_;
            target_value_ = target;
            start_ms_ = now_ms;
            duration_ms_ = std::max<std::uint32_t>(duration_ms, 1);
        }
        sample(now_ms);
        return current_;
    }

    void reset(float value, std::uint32_t now_ms) {
        start_value_ = value;
        target_value_ = value;
        current_ = value;
        start_ms_ = now_ms;
        duration_ms_ = 1;
        initialized_ = true;
    }

    float current() const { return current_; }

private:
    void sample(std::uint32_t now_ms) {
        const std::uint32_t elapsed_ms = now_ms - start_ms_;
        if (elapsed_ms >= duration_ms_) {
            current_ = target_value_;
            return;
        }
        const float t = static_cast<float>(elapsed_ms) /
                        static_cast<float>(duration_ms_);
        current_ = start_value_ + (target_value_ - start_value_) * t;
    }

    float start_value_ = 0.0f;
    float target_value_ = 0.0f;
    float current_ = 0.0f;
    std::uint32_t start_ms_ = 0;
    std::uint32_t duration_ms_ = 1;
    bool initialized_ = false;
};

}  // namespace tilt_walk_test
