#include <tilt/motion/MotionLibrary.h>

#include <tilt/motion/PoseInterpolator.h>
#include <tilt/motion/PoseLibrary.h>

namespace tilt {
namespace motion {

std::optional<PoseSample> sampleReturnHomeMotion(const JointPose& initial_pose,
                                                  float elapsed_seconds,
                                                  float duration_seconds) {
    return samplePoseTransition(initial_pose,
                                homePose(),
                                elapsed_seconds,
                                duration_seconds);
}

}  // namespace motion
}  // namespace tilt
