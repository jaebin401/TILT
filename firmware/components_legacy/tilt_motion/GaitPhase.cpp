#include <tilt/motion/GaitPhase.h>

#include <cmath>

namespace tilt {
namespace motion {

namespace {

bool isValidTiming(const GaitTiming& timing) {
    return std::isfinite(timing.double_support_seconds) &&
           std::isfinite(timing.swing_seconds) &&
           timing.double_support_seconds > 0.0f &&
           timing.swing_seconds > 0.0f;
}

LegSide opposite(LegSide side) {
    return side == LegSide::Left ? LegSide::Right : LegSide::Left;
}

SupportMode supportForSwing(LegSide swing_leg) {
    return swing_leg == LegSide::Left ? SupportMode::Right
                                      : SupportMode::Left;
}

float progress(float elapsed, float duration) {
    return elapsed / duration;
}

}  // namespace

std::optional<float> gaitCycleDurationSeconds(const GaitTiming& timing) {
    if (!isValidTiming(timing)) {
        return std::nullopt;
    }

    const float duration =
        2.0f * (timing.double_support_seconds + timing.swing_seconds);
    if (!std::isfinite(duration)) {
        return std::nullopt;
    }
    return duration;
}

std::optional<GaitPhaseSample> sampleGaitPhase(
    const GaitTiming& timing,
    LegSide first_swing_leg,
    float elapsed_seconds) {
    const auto cycle_duration = gaitCycleDurationSeconds(timing);
    if (!cycle_duration.has_value() || !std::isfinite(elapsed_seconds)) {
        return std::nullopt;
    }

    const float clamped_elapsed = elapsed_seconds > 0.0f ? elapsed_seconds : 0.0f;
    float cycle_time = std::fmod(clamped_elapsed, *cycle_duration);
    if (cycle_time < timing.double_support_seconds) {
        return GaitPhaseSample{
            SupportMode::Double,
            std::nullopt,
            first_swing_leg,
            progress(cycle_time, timing.double_support_seconds),
        };
    }

    cycle_time -= timing.double_support_seconds;
    if (cycle_time < timing.swing_seconds) {
        return GaitPhaseSample{
            supportForSwing(first_swing_leg),
            first_swing_leg,
            first_swing_leg,
            progress(cycle_time, timing.swing_seconds),
        };
    }

    const LegSide second_swing_leg = opposite(first_swing_leg);
    cycle_time -= timing.swing_seconds;
    if (cycle_time < timing.double_support_seconds) {
        return GaitPhaseSample{
            SupportMode::Double,
            std::nullopt,
            second_swing_leg,
            progress(cycle_time, timing.double_support_seconds),
        };
    }

    cycle_time -= timing.double_support_seconds;
    return GaitPhaseSample{
        supportForSwing(second_swing_leg),
        second_swing_leg,
        second_swing_leg,
        progress(cycle_time, timing.swing_seconds),
    };
}

}  // namespace motion
}  // namespace tilt
