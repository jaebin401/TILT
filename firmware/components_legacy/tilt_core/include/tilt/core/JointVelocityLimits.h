#pragma once

namespace tilt {
namespace core {

// Maximum required average velocity for each logical joint, in rad/s.
// Values must be supplied by the caller; no robot limits are defined here.
struct JointVelocityLimits {
    float max_rad_per_sec[6];
};

}  // namespace core
}  // namespace tilt
