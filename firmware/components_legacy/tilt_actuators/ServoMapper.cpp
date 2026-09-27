#include <tilt/actuators/ServoMapper.h>

#include <cmath>

namespace tilt {
namespace actuators {

namespace {

constexpr float kTicksPerRadian = 4096.0f / 6.2831853071795864769f;

bool hasUniqueIds(const ServoCalibrationTable& calibration) {
    for (std::size_t outer = 0; outer < calibration.size(); ++outer) {
        for (std::size_t inner = outer + 1; inner < calibration.size(); ++inner) {
            if (calibration[outer].id == calibration[inner].id) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace

bool isValidCalibration(const ServoCalibrationTable& calibration) {
    if (!hasUniqueIds(calibration)) {
        return false;
    }

    for (const auto& servo : calibration) {
        if ((servo.direction != 1 && servo.direction != -1) ||
            servo.min_raw > servo.zero_raw || servo.zero_raw > servo.max_raw ||
            servo.max_raw > kStsPositionMax || servo.id >= 0xFE) {
            return false;
        }
    }
    return true;
}

std::optional<ServoTargetBatch> mapToServoTargets(
    const core::JointTargetBatch& batch,
    const ServoCalibrationTable& calibration) {
    if (!isValidCalibration(calibration)) {
        return std::nullopt;
    }

    ServoTargetBatch result{};
    result.seq = batch.seq;
    result.duration_ms = batch.duration_ms;
    for (std::size_t joint = 0; joint < kServoCount; ++joint) {
        const float angle = batch.angle_rad[joint];
        if (!std::isfinite(angle)) {
            return std::nullopt;
        }

        const auto& servo = calibration[joint];
        const float raw_float = static_cast<float>(servo.zero_raw) +
            static_cast<float>(servo.direction) * angle * kTicksPerRadian;
        if (!std::isfinite(raw_float)) {
            return std::nullopt;
        }

        const long raw_long = std::lround(raw_float);
        if (raw_long < servo.min_raw || raw_long > servo.max_raw ||
            raw_long < kStsPositionMin || raw_long > kStsPositionMax) {
            return std::nullopt;
        }

        result.id[joint] = servo.id;
        result.position_raw[joint] = static_cast<uint16_t>(raw_long);
    }
    return result;
}

std::optional<core::JointPositionReference> mapToJointReference(
    const std::array<uint16_t, kServoCount>& position_raw,
    const ServoCalibrationTable& calibration) {
    if (!isValidCalibration(calibration)) {
        return std::nullopt;
    }

    core::JointPositionReference result{};
    for (std::size_t joint = 0; joint < kServoCount; ++joint) {
        const auto& servo = calibration[joint];
        const uint16_t raw = position_raw[joint];
        if (raw < servo.min_raw || raw > servo.max_raw) {
            return std::nullopt;
        }
        result.angle_rad[joint] =
            static_cast<float>(servo.direction) *
            (static_cast<float>(raw) - static_cast<float>(servo.zero_raw)) /
            kTicksPerRadian;
    }
    return result;
}

}  // namespace actuators
}  // namespace tilt
