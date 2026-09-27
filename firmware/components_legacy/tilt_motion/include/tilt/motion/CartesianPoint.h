#pragma once

#include <cmath>

namespace tilt {
namespace motion {

// torso 좌표계(x=전진, y=왼쪽, z=위)의 위치다.
struct CartesianPoint {
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float z_mm = 0.0f;
};

inline bool isFiniteCartesianPoint(const CartesianPoint& point) {
    return std::isfinite(point.x_mm) && std::isfinite(point.y_mm) &&
           std::isfinite(point.z_mm);
}

}  // namespace motion
}  // namespace tilt
