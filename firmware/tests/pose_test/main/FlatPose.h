#pragma once

namespace tilt_pose_test {

// Builds [hip yaw, hip pitch, knee pitch] while keeping the foot horizontal.
// Returns false when the requested height is outside the closed-form workspace.
bool flatPoseForHeight(float leg_height_mm, float out[3]);

// Applies a fore/aft torso shift while preserving theta2 + theta3 == 0.
// The requested height remains the nominal height; with no ankle DOF, an
// exact independent x/z target and a horizontal foot cannot all be enforced.
bool flatPoseForHeightAndBodyX(float leg_height_mm, float body_x_mm,
                               float out[3]);

}  // namespace tilt_pose_test
