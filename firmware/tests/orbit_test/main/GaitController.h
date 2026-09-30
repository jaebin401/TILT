#pragma once

#include <cstdint>

#include "OrbitModel.h"
#include "RollEstimator.h"

namespace tilt_orbit {

enum class GaitMode : std::uint8_t { ROCK, OPEN, ORBIT };
enum class GaitState : std::uint8_t { IDLE, STARTUP, SSP_LEFT, SSP_RIGHT };
enum class GaitEventKind : std::uint8_t {
    NONE, STARTUP_WARNING, STARTUP_READY, HALF_CYCLE, ROCK_CYCLE,
};

struct GaitEvent {
    GaitEventKind kind = GaitEventKind::NONE;
    GaitMode mode = GaitMode::ROCK;
    GaitState stance = GaitState::IDLE;
    std::uint32_t elapsed_ms = 0;
    float rate_deg_s = 0.0f;
    float peak_deg = 0.0f;
    float minimum_deg = 0.0f;
    float maximum_deg = 0.0f;
    float delta_mm = 0.0f;
    bool watchdog = false;
};

struct GaitOutput {
    float left_mm = kNominalHeightMm;
    float right_mm = kNominalHeightMm;
    float delta_mm = 0.0f;
    float phase = 0.0f;
    int swing_leg = -1;
    bool measure_swing = false;
    GaitEvent event{};
};

class GaitController {
public:
    bool start(std::uint32_t now_ms);
    void stop();
    GaitOutput update(std::uint32_t now_ms, const RollState& roll);

    bool selectMode(GaitMode mode);
    GaitMode mode() const { return mode_; }
    GaitState state() const { return state_; }
    bool running() const { return state_ != GaitState::IDLE; }

    void setPeriodS(float period_s);
    float periodS() const { return period_s_; }
    void setLiftMm(float mm);
    float liftMm() const { return lift_mm_; }
    void setPushMm(float mm);
    float pushMm() const { return push_mm_; }
    void setStartupShiftMm(float mm);
    float startupShiftMm() const { return startup_shift_mm_; }
    void toggleEnergy() { energy_enabled_ = !energy_enabled_; }
    bool energyEnabled() const { return energy_enabled_; }
    std::uint32_t switches() const { return switches_; }
    std::uint32_t watchdogs() const { return watchdogs_; }

private:
    void beginSsp(std::uint32_t now_ms, GaitState stance);
    GaitMode mode_ = GaitMode::ROCK;
    GaitState state_ = GaitState::IDLE;
    float period_s_ = computeParams().t_star_s;
    float lift_mm_ = kLiftZmaxDefaultMm;
    float push_mm_ = kPushDefaultMm;
    float active_push_mm_ = kPushDefaultMm;
    float startup_shift_mm_ = kStartupShiftMm;
    float last_left_mm_ = kNominalHeightMm;
    float last_right_mm_ = kNominalHeightMm;
    float stance_start_mm_ = kNominalHeightMm;
    float swing_carry_mm_ = 0.0f;
    bool energy_enabled_ = false;
    bool peak_measured_ = false;
    bool startup_warned_ = false;
    bool previous_roll_valid_ = false;
    float previous_roll_deg_ = 0.0f;
    float roll_history_[3]{};
    std::uint8_t roll_history_count_ = 0;
    std::uint8_t roll_history_next_ = 0;
    float peak_toward_stance_deg_ = 0.0f;
    float period_min_roll_deg_ = 0.0f;
    float period_max_roll_deg_ = 0.0f;
    int startup_good_halves_ = 0;
    std::uint32_t startup_halves_ = 0;
    std::uint32_t switches_ = 0;
    std::uint32_t watchdogs_ = 0;
    std::uint32_t state_started_ms_ = 0;
    std::uint32_t previous_half_ms_ = 0;
};

const char* modeName(GaitMode mode);
const char* stateName(GaitState state);

}  // namespace tilt_orbit
