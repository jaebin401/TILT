#pragma once

#include <array>
#include <cmath>

#include "driver/gpio.h"
#include "driver/uart.h"

#include <tilt/actuators/ServoMapper.h>
#include <tilt/core/JointPositionLimits.h>
#include <tilt/core/JointVelocityLimits.h>
#include <tilt/sts3215/Sts3215Bus.h>

namespace tilt_pose_test {

constexpr float degreesToRadians(float degrees) {
    return degrees * 0.01745329251994329577f;
}

// 하드웨어 설정은 이 파일 한 곳에서만 조정한다.
constexpr tilt::sts3215::BusConfig kBusConfig{
    UART_NUM_1,
    GPIO_NUM_17,
    GPIO_NUM_18,
    1000000,
    20,
};

constexpr uint16_t kMotorZeroRaw = 2047;
constexpr uint16_t kRawSoftMinimum = 1500;
constexpr uint16_t kRawSoftMaximum = 2600;

// 방향은 기구 조립 방향에 따라 달라진다. 최초 값은 모두 +이며, 콘솔의
// `direction <joint> <+|->`로 바꾼 뒤 `confirm`해야 pose를 실행할 수 있다.
constexpr tilt::actuators::ServoCalibrationTable kInitialCalibration{{
    {11, +1, kMotorZeroRaw, kRawSoftMinimum, kRawSoftMaximum},  // LHY
    {12, +1, kMotorZeroRaw, kRawSoftMinimum, kRawSoftMaximum},  // LHP
    {13, +1, kMotorZeroRaw, kRawSoftMinimum, kRawSoftMaximum},  // LKP
    {21, +1, kMotorZeroRaw, kRawSoftMinimum, kRawSoftMaximum},  // RHY
    {22, +1, kMotorZeroRaw, kRawSoftMinimum, kRawSoftMaximum},  // RHP
    {23, +1, kMotorZeroRaw, kRawSoftMinimum, kRawSoftMaximum},  // RKP
}};

constexpr std::array<const char*, 6> kJointNames{{
    "LHY", "LHP", "LKP", "RHY", "RHP", "RKP",
}};

inline tilt::core::JointPositionLimits positionLimits() {
    tilt::core::JointPositionLimits limits{};
    // 실제 기구 한계가 측정되기 전 pose 검증용 보수 범위다.
    limits.min_rad[0] = degreesToRadians(-10.0f);
    limits.max_rad[0] = degreesToRadians(+10.0f);
    limits.min_rad[1] = degreesToRadians(-35.0f);
    limits.max_rad[1] = degreesToRadians(+5.0f);
    limits.min_rad[2] = degreesToRadians(-5.0f);
    limits.max_rad[2] = degreesToRadians(+40.0f);
    limits.min_rad[3] = degreesToRadians(-10.0f);
    limits.max_rad[3] = degreesToRadians(+10.0f);
    limits.min_rad[4] = degreesToRadians(-35.0f);
    limits.max_rad[4] = degreesToRadians(+5.0f);
    limits.min_rad[5] = degreesToRadians(-5.0f);
    limits.max_rad[5] = degreesToRadians(+40.0f);
    return limits;
}

inline tilt::core::JointVelocityLimits velocityLimits() {
    tilt::core::JointVelocityLimits limits{};
    for (float& maximum : limits.max_rad_per_sec) {
        maximum = degreesToRadians(35.0f);
    }
    return limits;
}

constexpr uint16_t kServoSpeedRaw = 300;
constexpr uint8_t kServoAcceleration = 10;
constexpr uint32_t kPoseDurationMs = 3000;
constexpr uint32_t kJointDurationMs = 1000;
constexpr uint16_t kControlPeriodMs = 100;
constexpr float kMaximumJointMonkeyDeltaDegrees = 5.0f;

}  // namespace tilt_pose_test
