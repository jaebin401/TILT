#pragma once

#include <cstdint>

#include "tilt_mpu6050.h"

namespace tilt_orbit {

struct RollState {
    float roll_deg = 0.0f;
    float rate_deg_s = 0.0f;
    float lateral_mm = 0.0f;
    bool valid = false;
};

class RollEstimator {
public:
    bool begin();
    RollState sample(std::uint32_t now_ms);
    bool zero();
    void setSign(int sign);
    int sign() const { return sign_; }
    bool available() const { return available_; }
    float rawRollDeg() const { return raw_roll_deg_; }
    RollState state() const { return state_; }

private:
    tilt::ComplementaryFilter filter_;
    RollState state_{};
    std::uint32_t previous_ms_ = 0;
    float raw_roll_deg_ = 0.0f;
    float zero_roll_deg_ = 0.0f;
    float filtered_rate_deg_s_ = 0.0f;
    int sign_ = +1;
    bool available_ = false;
    bool rate_initialized_ = false;
};

}  // namespace tilt_orbit
