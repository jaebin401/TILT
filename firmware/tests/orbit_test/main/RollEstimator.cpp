#include "RollEstimator.h"

#include <cmath>

#include "OrbitConfig.h"

namespace tilt_orbit {

bool RollEstimator::begin() {
    available_ = tilt::imu_init();
    filter_.reset();
    state_ = {};
    previous_ms_ = 0;
    rejected_samples_ = 0;
    rate_initialized_ = false;
    sign_ = kRollSign >= 0 ? +1 : -1;
    return available_;
}

RollState RollEstimator::sample(std::uint32_t now_ms) {
    if (!available_) {
        state_.valid = false;
        return state_;
    }
    tilt::ImuRaw raw{};
    if (!tilt::imu_read_raw(raw)) {
        state_.valid = false;
        previous_ms_ = 0;
        rate_initialized_ = false;
        return state_;
    }
    const float dt_s = previous_ms_ == 0
        ? kLoopPeriodMs / 1000.0f
        : static_cast<float>(now_ms - previous_ms_) / 1000.0f;
    previous_ms_ = now_ms;
    // imu_read_raw() has already applied the configured axis mapping. The
    // complementary filter integrates gx for roll, so reject an implausible
    // gx before it can contaminate either the filter or the crossing trigger.
    const float raw_rate = tilt::gyro_to_rad_s(raw.gx) / tilt::DEG2RAD;
    const bool rate_spike = !std::isfinite(raw_rate) ||
        std::fabs(raw_rate) > kMaxPlausibleRateDegS;
    if (rate_spike) {
        ++rejected_samples_;
        return state_;
    }
    const bool abnormal_dt = !std::isfinite(dt_s) || dt_s <= 0.0f ||
                             dt_s > kMaxImuDtS;
    if (abnormal_dt) {
        // Reset makes the next update initialize directly from accelerometer
        // attitude, without integrating gyro across a stalled loop.
        filter_.reset();
        ++rejected_samples_;
    }
    const tilt::Attitude attitude = filter_.update(
        raw, abnormal_dt ? kLoopPeriodMs / 1000.0f : dt_s);
    if (!filter_.initialized()) {
        state_.valid = false;
        return state_;
    }
    if (!abnormal_dt) {
        filtered_rate_deg_s_ = rate_initialized_
            ? 0.5f * raw_rate + 0.5f * filtered_rate_deg_s_
            : raw_rate;
        rate_initialized_ = true;
    }
    raw_roll_deg_ = attitude.roll_rad / tilt::DEG2RAD;
    state_.roll_deg = (raw_roll_deg_ - zero_roll_deg_) * sign_;
    state_.rate_deg_s = filtered_rate_deg_s_ * sign_;
    state_.lateral_mm = kCoMHeightMm *
        std::tan(state_.roll_deg * tilt::DEG2RAD);
    state_.valid = std::isfinite(state_.roll_deg) &&
                   std::isfinite(state_.rate_deg_s) &&
                   std::isfinite(state_.lateral_mm);
    return state_;
}

bool RollEstimator::zero() {
    if (!state_.valid || std::fabs(state_.rate_deg_s) > 5.0f) return false;
    zero_roll_deg_ = raw_roll_deg_;
    state_.roll_deg = 0.0f;
    state_.lateral_mm = 0.0f;
    return true;
}

void RollEstimator::setSign(int sign) {
    const int next = sign >= 0 ? +1 : -1;
    if (next == sign_) return;
    sign_ = next;
    state_.roll_deg = (raw_roll_deg_ - zero_roll_deg_) * sign_;
    state_.rate_deg_s = filtered_rate_deg_s_ * sign_;
    state_.lateral_mm = kCoMHeightMm *
        std::tan(state_.roll_deg * tilt::DEG2RAD);
}

}  // namespace tilt_orbit
