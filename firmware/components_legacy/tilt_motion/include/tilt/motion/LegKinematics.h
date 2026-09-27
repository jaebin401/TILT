#pragma once

#include <optional>

#include <tilt/motion/CartesianPoint.h>

namespace tilt {
namespace motion {

enum class LegSide {
    Left,
    Right,
};

// ADR-008의 torso 원점과 Hip Pitch 축을 기준으로 한 다리 형상이다.
struct LegGeometry {
    float hip_offset_y_mm;
    float thigh_length_mm;
    float shin_length_mm;
};

struct LegJointAngles {
    float hip_yaw_rad;
    float hip_pitch_rad;
    float knee_pitch_rad;
};

// ADR-008에서 확정된 Hip 간격과 50 mm 링크 두 개를 반환한다.
const LegGeometry& tiltLegGeometry();

// Hip Pitch와 Knee Pitch로 결정되는 Ankle 고정점 O4의 위치를 계산한다.
// 고정 발 브라켓과 접지점 E의 오프셋은 포함하지 않는다.
std::optional<CartesianPoint> forwardAnkleKinematics(
    const LegGeometry& geometry,
    LegSide side,
    const LegJointAngles& angles);

// 주어진 Hip Yaw 평면 안에서 Ankle 고정점 O4를 만족하는 Pitch/Knee를 푼다.
// Knee Pitch는 ADR-008의 양(+) 굽힘 해를 사용한다.
std::optional<LegJointAngles> inverseAnkleKinematics(
    const LegGeometry& geometry,
    LegSide side,
    const CartesianPoint& ankle_target,
    float hip_yaw_rad);

}  // namespace motion
}  // namespace tilt
