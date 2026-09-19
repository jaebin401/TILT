#include <tilt/motion/FootTrajectory.h>

#include <cmath>

namespace tilt {
namespace motion {

namespace {

struct QuinticSample {
    float position;
    float velocity;
    float acceleration;
};

QuinticSample sampleQuintic(float normalized_time,
                            float normalized_velocity,
                            float normalized_acceleration) {
    const float u2 = normalized_time * normalized_time;
    const float u3 = u2 * normalized_time;
    const float u4 = u3 * normalized_time;
    const float u5 = u4 * normalized_time;

    return {
        6.0f * u5 - 15.0f * u4 + 10.0f * u3,
        (30.0f * u4 - 60.0f * u3 + 30.0f * u2) * normalized_velocity,
        (120.0f * u3 - 180.0f * u2 + 60.0f * normalized_time) *
            normalized_acceleration,
    };
}

FootTrajectorySample stationarySample(const FootPosition& position,
                                      bool finished) {
    FootTrajectorySample sample{};
    sample.position = position;
    sample.finished = finished;
    return sample;
}

}  // namespace

std::optional<FootTrajectorySample> sampleFootTrajectory(
    const FootPosition& start,
    const FootPosition& target,
    float clearance_mm,
    float elapsed_seconds,
    float duration_seconds) {
    if (!isFiniteCartesianPoint(start) || !isFiniteCartesianPoint(target) ||
        !std::isfinite(clearance_mm) || clearance_mm < 0.0f ||
        !std::isfinite(elapsed_seconds) || !std::isfinite(duration_seconds) ||
        duration_seconds <= 0.0f) {
        return std::nullopt;
    }

    if (elapsed_seconds <= 0.0f) {
        return stationarySample(start, false);
    }
    if (elapsed_seconds >= duration_seconds) {
        return stationarySample(target, true);
    }

    const float normalized_time = elapsed_seconds / duration_seconds;
    const float inverse_duration = 1.0f / duration_seconds;
    const QuinticSample travel = sampleQuintic(
        normalized_time,
        inverse_duration,
        inverse_duration * inverse_duration);

    // 상승과 하강을 각각 quintic으로 만들어 시작·정점·끝에서 속도와 가속도를 0으로 둔다.
    const bool ascending = normalized_time < 0.5f;
    const float lift_time = ascending
                                ? normalized_time * 2.0f
                                : (1.0f - normalized_time) * 2.0f;
    const float lift_direction = ascending ? 1.0f : -1.0f;
    const QuinticSample lift = sampleQuintic(
        lift_time,
        lift_direction * 2.0f * inverse_duration,
        4.0f * inverse_duration * inverse_duration);

    const float delta_x = target.x_mm - start.x_mm;
    const float delta_y = target.y_mm - start.y_mm;
    const float delta_z = target.z_mm - start.z_mm;

    FootTrajectorySample result{};
    result.position = {
        start.x_mm + delta_x * travel.position,
        start.y_mm + delta_y * travel.position,
        start.z_mm + delta_z * travel.position + clearance_mm * lift.position,
    };
    result.velocity_mm_per_sec = {
        delta_x * travel.velocity,
        delta_y * travel.velocity,
        delta_z * travel.velocity + clearance_mm * lift.velocity,
    };
    result.acceleration_mm_per_sec2 = {
        delta_x * travel.acceleration,
        delta_y * travel.acceleration,
        delta_z * travel.acceleration + clearance_mm * lift.acceleration,
    };

    if (!isFiniteCartesianPoint(result.position) ||
        !isFiniteCartesianPoint(result.velocity_mm_per_sec) ||
        !isFiniteCartesianPoint(result.acceleration_mm_per_sec2)) {
        return std::nullopt;
    }
    return result;
}

}  // namespace motion
}  // namespace tilt
