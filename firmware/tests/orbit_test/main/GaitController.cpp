#include "GaitController.h"

#include <algorithm>
#include <cmath>

#include "Bezier.h"

namespace tilt_orbit {

namespace {
constexpr float kPi = 3.14159265358979323846f;

float median3(float a, float b, float c) {
    return std::max(std::min(a, b), std::min(std::max(a, b), c));
}
}

const char* modeName(GaitMode mode) {
    switch (mode) {
        case GaitMode::ROCK: return "ROCK";
        case GaitMode::OPEN: return "OPEN";
        case GaitMode::ORBIT: return "ORBIT";
    }
    return "?";
}

const char* stateName(GaitState state) {
    switch (state) {
        case GaitState::IDLE: return "IDLE";
        case GaitState::STARTUP: return "STARTUP";
        case GaitState::SSP_LEFT: return "SSP_LEFT";
        case GaitState::SSP_RIGHT: return "SSP_RIGHT";
    }
    return "?";
}

bool GaitController::selectMode(GaitMode mode) {
    if (running()) return false;
    mode_ = mode;
    return true;
}

void GaitController::setPeriodS(float seconds) {
    if (std::isfinite(seconds))
        period_s_ = std::clamp(seconds, kTMinS, kTMaxS);
}

void GaitController::setLiftMm(float mm) {
    if (std::isfinite(mm)) lift_mm_ = std::clamp(mm, 0.0f, kLiftMaxMm);
}

void GaitController::setPushMm(float mm) {
    if (std::isfinite(mm)) push_mm_ = std::clamp(mm, 0.0f, kPushMaxMm);
}

void GaitController::setStartupShiftMm(float mm) {
    if (std::isfinite(mm)) startup_shift_mm_ = std::clamp(mm, 0.0f, 10.0f);
}

bool GaitController::start(std::uint32_t now_ms) {
    if (running()) return false;
    state_ = GaitState::STARTUP;
    state_started_ms_ = previous_half_ms_ = now_ms;
    startup_halves_ = switches_ = watchdogs_ = 0;
    startup_good_halves_ = 0;
    startup_warned_ = previous_roll_valid_ = false;
    roll_history_count_ = roll_history_next_ = 0;
    period_min_roll_deg_ = period_max_roll_deg_ = 0.0f;
    active_push_mm_ = push_mm_;
    peak_measured_ = false;
    return true;
}

void GaitController::stop() {
    state_ = GaitState::IDLE;
    previous_roll_valid_ = false;
    roll_history_count_ = roll_history_next_ = 0;
}

void GaitController::beginSsp(std::uint32_t now_ms, GaitState stance) {
    state_ = stance;
    state_started_ms_ = now_ms;
    peak_toward_stance_deg_ = 0.0f;
    peak_measured_ = false;
    period_min_roll_deg_ = period_max_roll_deg_ = 0.0f;
}

GaitOutput GaitController::update(std::uint32_t now_ms,
                                  const RollState& roll) {
    GaitOutput output{};
    if (!running()) return output;
    const std::uint32_t period_ms = static_cast<std::uint32_t>(
        std::lround(period_s_ * 1000.0f));
    if (roll.valid) {
        roll_history_[roll_history_next_] = roll.roll_deg;
        roll_history_next_ = static_cast<std::uint8_t>(
            (roll_history_next_ + 1) % 3);
        if (roll_history_count_ < 3) ++roll_history_count_;
        period_min_roll_deg_ = std::min(period_min_roll_deg_, roll.roll_deg);
        period_max_roll_deg_ = std::max(period_max_roll_deg_, roll.roll_deg);
    }

    if (state_ == GaitState::STARTUP) {
        const std::uint32_t elapsed = now_ms - state_started_ms_;
        if (elapsed >= period_ms) {
            ++startup_halves_;
            const float threshold = 0.7f * predict(period_s_).roll_amp_deg;
            const float observed_peak = std::max(
                std::fabs(period_min_roll_deg_),
                std::fabs(period_max_roll_deg_));
            startup_good_halves_ = roll.valid && observed_peak >= threshold
                ? startup_good_halves_ + 1 : 0;
            output.event = {GaitEventKind::NONE, mode_, GaitState::STARTUP,
                            now_ms - previous_half_ms_, roll.rate_deg_s,
                            observed_peak, period_min_roll_deg_,
                            period_max_roll_deg_, 0.0f, false};
            if (mode_ == GaitMode::ROCK && startup_halves_ % 2 == 0) {
                output.event.kind = GaitEventKind::ROCK_CYCLE;
            } else if (mode_ != GaitMode::ROCK &&
                       startup_good_halves_ >= 2) {
                output.event.kind = GaitEventKind::STARTUP_READY;
                beginSsp(now_ms, roll.valid && roll.rate_deg_s < 0.0f
                    ? GaitState::SSP_RIGHT : GaitState::SSP_LEFT);
                active_push_mm_ = push_mm_;
            } else if (mode_ != GaitMode::ROCK && !startup_warned_ &&
                       startup_halves_ >= 2 * kStartupMaxCycles) {
                startup_warned_ = true;
                output.event.kind = GaitEventKind::STARTUP_WARNING;
            }
            if (state_ == GaitState::STARTUP) {
                previous_half_ms_ = now_ms;
                state_started_ms_ = now_ms;
                if (startup_halves_ % 2 == 0) {
                    period_min_roll_deg_ = period_max_roll_deg_ = 0.0f;
                }
            }
        }
        if (state_ == GaitState::STARTUP) {
            const float phase = std::min(1.0f,
                static_cast<float>(now_ms - state_started_ms_) / period_ms);
            const float sign = startup_halves_ % 2 == 0 ? +1.0f : -1.0f;
            const float shift = sign * startup_shift_mm_ *
                                std::sin(kPi * phase);
            output.left_mm = kNominalHeightMm - shift / 2.0f;
            output.right_mm = kNominalHeightMm + shift / 2.0f;
            return output;
        }
    }

    const GaitState old_stance = state_;
    const std::uint32_t elapsed = now_ms - state_started_ms_;
    if (roll.valid) {
        const float toward = state_ == GaitState::SSP_LEFT
            ? roll.roll_deg : -roll.roll_deg;
        peak_toward_stance_deg_ = std::max(
            peak_toward_stance_deg_, toward);
    }
    bool switch_stance = false;
    bool watchdog = false;
    if (mode_ == GaitMode::OPEN) {
        switch_stance = elapsed >= period_ms;
    } else if (mode_ == GaitMode::ORBIT) {
        const bool median_valid = roll.valid && roll_history_count_ == 3;
        const float crossing_roll = median_valid
            ? median3(roll_history_[0], roll_history_[1], roll_history_[2])
            : 0.0f;
        const bool crossed = median_valid && previous_roll_valid_ &&
            ((previous_roll_deg_ < 0.0f && crossing_roll >= 0.0f) ||
             (previous_roll_deg_ > 0.0f && crossing_roll <= 0.0f));
        switch_stance = crossed &&
            std::fabs(roll.rate_deg_s) >= kCrossMinRateDegS &&
            elapsed >= period_ms / 2;
        if (!switch_stance && elapsed >= period_ms * 3 / 2) {
            switch_stance = watchdog = true;
        }
    }
    if (switch_stance) {
        output.event = {GaitEventKind::HALF_CYCLE, mode_, old_stance,
                        elapsed, roll.rate_deg_s,
                        old_stance == GaitState::SSP_LEFT
                            ? peak_toward_stance_deg_
                            : -peak_toward_stance_deg_,
                        period_min_roll_deg_,
                        period_max_roll_deg_, active_push_mm_, watchdog};
        ++switches_;
        if (watchdog) ++watchdogs_;
        if (mode_ == GaitMode::ORBIT && energy_enabled_ && roll.valid) {
            const float target = predict(period_s_).roll_rate_target_deg_s;
            push_mm_ = std::clamp(push_mm_ +
                kEnergyGainMmPerDegS *
                (target - std::fabs(roll.rate_deg_s)),
                0.0f, kPushMaxMm);
        }
        const GaitState next = mode_ == GaitMode::ORBIT &&
            !watchdog && roll.valid
            ? (roll.rate_deg_s > 0.0f
                ? GaitState::SSP_LEFT : GaitState::SSP_RIGHT)
            : (old_stance == GaitState::SSP_LEFT
                ? GaitState::SSP_RIGHT : GaitState::SSP_LEFT);
        beginSsp(now_ms, next);
        active_push_mm_ = push_mm_;
    }
    if (roll_history_count_ == 3) {
        previous_roll_valid_ = roll.valid;
        if (roll.valid) {
            previous_roll_deg_ = median3(
                roll_history_[0], roll_history_[1], roll_history_[2]);
        }
    }

    const float phase = std::clamp(
        static_cast<float>(now_ms - state_started_ms_) / period_ms,
        0.0f, 1.0f);
    const float lift = swingBezier(phase, lift_mm_, kLiftZnegMm);
    const float push = phase < 1.0f - kPushWindow ? 0.0f :
        active_push_mm_ * (phase - (1.0f - kPushWindow)) / kPushWindow;
    output.phase = phase;
    output.delta_mm = push;
    if (state_ == GaitState::SSP_LEFT) {
        output.left_mm = kNominalHeightMm + push;
        output.right_mm = kNominalHeightMm - lift;
        output.swing_leg = 1;
    } else {
        output.left_mm = kNominalHeightMm - lift;
        output.right_mm = kNominalHeightMm + push;
        output.swing_leg = 0;
    }
    if (phase >= 0.5f && !peak_measured_) {
        peak_measured_ = true;
        output.measure_swing = true;
    }
    return output;
}

}  // namespace tilt_orbit
