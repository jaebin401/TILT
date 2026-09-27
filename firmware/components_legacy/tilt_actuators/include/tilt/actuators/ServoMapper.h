#pragma once

#include <array>
#include <optional>
#include <stdint.h>

#include <tilt/core/JointPositionReference.h>
#include <tilt/core/JointTargetBatch.h>

namespace tilt {
namespace actuators {

constexpr std::size_t kServoCount = 6;
constexpr uint16_t kStsPositionMin = 0;
constexpr uint16_t kStsPositionMax = 4095;

struct ServoCalibration {
    uint8_t id;
    int8_t direction;  // 논리 각도 증가가 raw 증가면 +1, 감소면 -1.
    uint16_t zero_raw;
    uint16_t min_raw;
    uint16_t max_raw;
};

using ServoCalibrationTable = std::array<ServoCalibration, kServoCount>;

struct ServoTargetBatch {
    uint32_t seq;
    uint16_t duration_ms;
    std::array<uint8_t, kServoCount> id;
    std::array<uint16_t, kServoCount> position_raw;
};

bool isValidCalibration(const ServoCalibrationTable& calibration);

// Safety가 승인한 논리 rad 명령을 STS3215 raw position으로 변환한다.
std::optional<ServoTargetBatch> mapToServoTargets(
    const core::JointTargetBatch& batch,
    const ServoCalibrationTable& calibration);

// 서보 피드백을 Safety가 사용하는 현재 논리 관절각으로 역변환한다.
std::optional<core::JointPositionReference> mapToJointReference(
    const std::array<uint16_t, kServoCount>& position_raw,
    const ServoCalibrationTable& calibration);

}  // namespace actuators
}  // namespace tilt
