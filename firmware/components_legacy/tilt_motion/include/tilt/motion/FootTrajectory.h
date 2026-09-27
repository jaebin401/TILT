#pragma once

#include <optional>

#include <tilt/motion/CartesianPoint.h>

namespace tilt {
namespace motion {

using FootPosition = CartesianPoint;

// 한 시각의 발 위치와 시간에 대한 해석적 미분값이다.
struct FootTrajectorySample {
    FootPosition position;
    FootPosition velocity_mm_per_sec;
    FootPosition acceleration_mm_per_sec2;
    bool finished = false;
};

// 시작점에서 목표점까지 이동하며 z+ 방향으로 clearance만큼 들어 올린다.
// clearance가 0이면 지지발을 포함한 일반적인 Cartesian 이동으로 사용할 수 있다.
std::optional<FootTrajectorySample> sampleFootTrajectory(
    const FootPosition& start,
    const FootPosition& target,
    float clearance_mm,
    float elapsed_seconds,
    float duration_seconds);

}  // namespace motion
}  // namespace tilt
