#include <tilt/motion/BipedStep.h>
#include <tilt/motion/ContactKinematics.h>
#include <tilt/motion/ContactStep.h>
#include <tilt/motion/FootTrajectory.h>
#include <tilt/motion/GaitPhase.h>
#include <tilt/motion/LegKinematics.h>
#include <tilt/motion/MotionClip.h>
#include <tilt/motion/MotionLibrary.h>
#include <tilt/motion/PoseInterpolator.h>
#include <tilt/motion/PoseLibrary.h>
#include <tilt/motion/RobotKinematics.h>

#include <cmath>
#include <cstdio>
#include <limits>

namespace {

constexpr float kTolerance = 1.0e-5f;

bool check(bool condition, const char* case_name) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", case_name);
    }
    return condition;
}

bool nearlyEqual(float left, float right, float tolerance = kTolerance) {
    return std::fabs(left - right) <= tolerance;
}

bool posesEqual(const tilt::motion::JointPose& left,
                const tilt::motion::JointPose& right,
                float tolerance = kTolerance) {
    for (std::size_t joint = 0; joint < tilt::motion::kLegJointCount; ++joint) {
        if (!nearlyEqual(left.angle_rad[joint], right.angle_rad[joint], tolerance)) {
            return false;
        }
    }
    return true;
}

bool positionsEqual(const tilt::motion::FootPosition& left,
                    const tilt::motion::FootPosition& right,
                    float tolerance = kTolerance) {
    return nearlyEqual(left.x_mm, right.x_mm, tolerance) &&
           nearlyEqual(left.y_mm, right.y_mm, tolerance) &&
           nearlyEqual(left.z_mm, right.z_mm, tolerance);
}

tilt::motion::JointPose filledPose(float value) {
    tilt::motion::JointPose pose{};
    pose.angle_rad.fill(value);
    return pose;
}

}  // namespace

int main() {
    using tilt::motion::JointIndex;
    using tilt::motion::JointPose;
    using tilt::motion::MotionClipView;
    using tilt::motion::PoseKeyframe;

    const JointPose& home = tilt::motion::homePose();
    constexpr float twenty_degrees = 0.3490658504f;
    if (!check(tilt::motion::isFiniteJointPose(home),
               "Home Pose contains only finite angles")) return 1;
    if (!check(nearlyEqual(home.angle_rad[static_cast<std::size_t>(JointIndex::LeftHipYaw)], 0.0f) &&
                   nearlyEqual(home.angle_rad[static_cast<std::size_t>(JointIndex::LeftHipPitch)], -twenty_degrees) &&
                   nearlyEqual(home.angle_rad[static_cast<std::size_t>(JointIndex::LeftKneePitch)], twenty_degrees) &&
                   nearlyEqual(home.angle_rad[static_cast<std::size_t>(JointIndex::RightHipYaw)], 0.0f) &&
                   nearlyEqual(home.angle_rad[static_cast<std::size_t>(JointIndex::RightHipPitch)], -twenty_degrees) &&
                   nearlyEqual(home.angle_rad[static_cast<std::size_t>(JointIndex::RightKneePitch)], twenty_degrees),
               "Home Pose matches ADR-008 logical joint angles")) return 1;

    if (!check(tilt::motion::poseLibrarySize() == 5 &&
                   tilt::motion::poseIdForName("home") ==
                       tilt::motion::PoseId::Home &&
                   tilt::motion::poseIdForName("crouch") ==
                       tilt::motion::PoseId::Crouch &&
                   !tilt::motion::poseIdForName("unknown").has_value() &&
                   posesEqual(tilt::motion::poseFor(tilt::motion::PoseId::Home),
                              home),
               "named Pose library exposes five deterministic test poses")) return 1;

    const JointPose start = filledPose(0.0f);
    const JointPose target = filledPose(1.0f);

    const auto at_start = tilt::motion::interpolatePose(start, target, 0.0f, 2.0f);
    if (!check(at_start.has_value() && posesEqual(*at_start, start),
               "interpolation starts at the supplied Pose")) return 1;

    const auto before_start = tilt::motion::interpolatePose(start, target, -1.0f, 2.0f);
    if (!check(before_start.has_value() && posesEqual(*before_start, start),
               "negative elapsed time clamps to the start Pose")) return 1;

    const auto midpoint = tilt::motion::interpolatePose(start, target, 1.0f, 2.0f);
    if (!check(midpoint.has_value() && posesEqual(*midpoint, filledPose(0.5f)),
               "smoothstep midpoint is halfway between Poses")) return 1;

    const auto midpoint_sample =
        tilt::motion::samplePoseTransition(start, target, 1.0f, 2.0f);
    if (!check(midpoint_sample.has_value() &&
                   posesEqual(midpoint_sample->pose, filledPose(0.5f)) &&
                   nearlyEqual(midpoint_sample->velocity_rad_per_sec[0], 0.9375f) &&
                   nearlyEqual(midpoint_sample->acceleration_rad_per_sec2[0], 0.0f) &&
                   !midpoint_sample->finished,
               "trajectory sample includes analytic velocity and acceleration")) return 1;

    const auto at_end = tilt::motion::interpolatePose(start, target, 2.0f, 2.0f);
    if (!check(at_end.has_value() && posesEqual(*at_end, target),
               "interpolation reaches the target Pose")) return 1;

    const auto completed_sample =
        tilt::motion::samplePoseTransition(start, target, 2.0f, 2.0f);
    if (!check(completed_sample.has_value() && completed_sample->finished &&
                   nearlyEqual(completed_sample->velocity_rad_per_sec[0], 0.0f) &&
                   nearlyEqual(completed_sample->acceleration_rad_per_sec2[0], 0.0f),
               "completed transition is stationary")) return 1;

    const auto after_end = tilt::motion::interpolatePose(start, target, 10.0f, 2.0f);
    if (!check(after_end.has_value() && posesEqual(*after_end, target),
               "elapsed time after duration holds the target Pose")) return 1;

    const auto near_start = tilt::motion::interpolatePose(start, target, 0.001f, 1.0f);
    const auto near_end = tilt::motion::interpolatePose(start, target, 0.999f, 1.0f);
    if (!check(near_start.has_value() && near_end.has_value() &&
                   near_start->angle_rad[0] < 0.00001f &&
                   (1.0f - near_end->angle_rad[0]) < 0.00001f,
               "smoothstep approaches both endpoints with zero slope")) return 1;

    if (!check(!tilt::motion::interpolatePose(start, target, 0.0f, 0.0f).has_value() &&
                   !tilt::motion::interpolatePose(start, target, 0.0f, -1.0f).has_value(),
               "non-positive duration is rejected")) return 1;

    JointPose invalid_pose = target;
    invalid_pose.angle_rad[2] = std::numeric_limits<float>::quiet_NaN();
    if (!check(!tilt::motion::interpolatePose(start, invalid_pose, 0.5f, 1.0f).has_value(),
               "non-finite Pose is rejected")) return 1;
    if (!check(!tilt::motion::interpolatePose(
                    start,
                    target,
                    std::numeric_limits<float>::infinity(),
                    1.0f).has_value(),
               "non-finite elapsed time is rejected")) return 1;

    const PoseKeyframe keyframes[] = {
        {filledPose(1.0f), 1.0f},
        {filledPose(-1.0f), 2.0f},
    };
    const MotionClipView clip{keyframes, 2};
    const auto total_duration = tilt::motion::motionDurationSeconds(clip);
    if (!check(total_duration.has_value() && nearlyEqual(*total_duration, 3.0f),
               "Motion Clip duration is the sum of segment durations")) return 1;

    const auto first_midpoint = tilt::motion::sampleMotion(start, clip, 0.5f);
    if (!check(first_midpoint.has_value() &&
                   posesEqual(*first_midpoint, filledPose(0.5f)),
               "Motion Clip samples the first segment")) return 1;

    const auto first_midpoint_state =
        tilt::motion::sampleMotionState(start, clip, 0.5f);
    if (!check(first_midpoint_state.has_value() &&
                   nearlyEqual(first_midpoint_state->velocity_rad_per_sec[0], 1.875f) &&
                   nearlyEqual(first_midpoint_state->acceleration_rad_per_sec2[0], 0.0f) &&
                   !first_midpoint_state->finished,
               "Motion Clip exposes the active segment derivatives")) return 1;

    const auto boundary = tilt::motion::sampleMotion(start, clip, 1.0f);
    if (!check(boundary.has_value() && posesEqual(*boundary, keyframes[0].target),
               "Motion Clip reaches an exact keyframe boundary")) return 1;

    const auto boundary_state = tilt::motion::sampleMotionState(start, clip, 1.0f);
    if (!check(boundary_state.has_value() && !boundary_state->finished &&
                   nearlyEqual(boundary_state->velocity_rad_per_sec[0], 0.0f) &&
                   nearlyEqual(boundary_state->acceleration_rad_per_sec2[0], 0.0f),
               "intermediate keyframe boundary is stationary but not complete")) return 1;

    const auto second_midpoint = tilt::motion::sampleMotion(start, clip, 2.0f);
    if (!check(second_midpoint.has_value() &&
                   posesEqual(*second_midpoint, filledPose(0.0f)),
               "Motion Clip samples a later segment from its previous keyframe")) return 1;

    const auto after_clip = tilt::motion::sampleMotion(start, clip, 10.0f);
    if (!check(after_clip.has_value() && posesEqual(*after_clip, keyframes[1].target),
               "Motion Clip holds its final Pose")) return 1;

    const auto after_clip_state = tilt::motion::sampleMotionState(start, clip, 10.0f);
    if (!check(after_clip_state.has_value() && after_clip_state->finished &&
                   nearlyEqual(after_clip_state->velocity_rad_per_sec[0], 0.0f) &&
                   nearlyEqual(after_clip_state->acceleration_rad_per_sec2[0], 0.0f),
               "completed Motion Clip is stationary")) return 1;

    const MotionClipView empty_clip{};
    const auto empty_result = tilt::motion::sampleMotion(start, empty_clip, 5.0f);
    if (!check(empty_result.has_value() && posesEqual(*empty_result, start),
               "empty Motion Clip is a valid no-op")) return 1;
    const auto empty_state = tilt::motion::sampleMotionState(start, empty_clip, 5.0f);
    if (!check(empty_state.has_value() && empty_state->finished,
               "empty Motion Clip is immediately complete")) return 1;

    const MotionClipView missing_keyframes{nullptr, 1};
    if (!check(!tilt::motion::motionDurationSeconds(missing_keyframes).has_value() &&
                   !tilt::motion::sampleMotion(start, missing_keyframes, 0.0f).has_value(),
               "non-empty Motion Clip requires keyframe storage")) return 1;

    const PoseKeyframe invalid_keyframes[] = {
        {filledPose(1.0f), 1.0f},
        {filledPose(2.0f), 0.0f},
    };
    const MotionClipView invalid_clip{invalid_keyframes, 2};
    if (!check(!tilt::motion::motionDurationSeconds(invalid_clip).has_value() &&
                   !tilt::motion::sampleMotion(start, invalid_clip, 0.5f).has_value(),
               "an invalid later keyframe rejects the whole Motion Clip")) return 1;

    const auto returning_home =
        tilt::motion::sampleReturnHomeMotion(start, 2.0f, 2.0f);
    if (!check(returning_home.has_value() && returning_home->finished &&
                   posesEqual(returning_home->pose, home) &&
                   nearlyEqual(returning_home->velocity_rad_per_sec[0], 0.0f) &&
                   nearlyEqual(returning_home->acceleration_rad_per_sec2[0], 0.0f),
               "Return Home Motion reaches the documented Home Pose")) return 1;

    if (!check(!tilt::motion::sampleReturnHomeMotion(start, 0.0f, 0.0f).has_value(),
               "Return Home Motion rejects an invalid duration")) return 1;

    const tilt::motion::FootPosition foot_start{0.0f, 36.2f, -104.0f};
    const tilt::motion::FootPosition foot_target{30.0f, 36.2f, -104.0f};
    const auto foot_at_start = tilt::motion::sampleFootTrajectory(
        foot_start, foot_target, 12.0f, 0.0f, 2.0f);
    if (!check(foot_at_start.has_value() &&
                   positionsEqual(foot_at_start->position, foot_start) &&
                   positionsEqual(foot_at_start->velocity_mm_per_sec, {}) &&
                   positionsEqual(foot_at_start->acceleration_mm_per_sec2, {}) &&
                   !foot_at_start->finished,
               "Foot trajectory starts stationary at the supplied point")) return 1;

    const auto foot_at_apex = tilt::motion::sampleFootTrajectory(
        foot_start, foot_target, 12.0f, 1.0f, 2.0f);
    const tilt::motion::FootPosition expected_apex{15.0f, 36.2f, -92.0f};
    if (!check(foot_at_apex.has_value() &&
                   positionsEqual(foot_at_apex->position, expected_apex) &&
                   nearlyEqual(foot_at_apex->velocity_mm_per_sec.z_mm, 0.0f) &&
                   nearlyEqual(foot_at_apex->acceleration_mm_per_sec2.z_mm, 0.0f),
               "Foot trajectory reaches its requested clearance at mid-step")) return 1;

    const auto foot_at_end = tilt::motion::sampleFootTrajectory(
        foot_start, foot_target, 12.0f, 2.0f, 2.0f);
    if (!check(foot_at_end.has_value() &&
                   positionsEqual(foot_at_end->position, foot_target) &&
                   positionsEqual(foot_at_end->velocity_mm_per_sec, {}) &&
                   positionsEqual(foot_at_end->acceleration_mm_per_sec2, {}) &&
                   foot_at_end->finished,
               "Foot trajectory lands stationary at the target point")) return 1;

    const auto rising_foot = tilt::motion::sampleFootTrajectory(
        foot_start, foot_target, 12.0f, 0.5f, 2.0f);
    const auto falling_foot = tilt::motion::sampleFootTrajectory(
        foot_start, foot_target, 12.0f, 1.5f, 2.0f);
    if (!check(rising_foot.has_value() && falling_foot.has_value() &&
                   nearlyEqual(rising_foot->position.z_mm,
                               falling_foot->position.z_mm) &&
                   rising_foot->velocity_mm_per_sec.z_mm > 0.0f &&
                   falling_foot->velocity_mm_per_sec.z_mm < 0.0f,
               "Foot lift is symmetric with opposite vertical velocity")) return 1;

    if (!check(!tilt::motion::sampleFootTrajectory(
                    foot_start, foot_target, -1.0f, 0.5f, 1.0f).has_value() &&
                   !tilt::motion::sampleFootTrajectory(
                    foot_start, foot_target, 1.0f, 0.5f, 0.0f).has_value(),
               "Foot trajectory rejects invalid clearance and duration")) return 1;

    const tilt::motion::LegGeometry& geometry =
        tilt::motion::tiltLegGeometry();
    if (!check(nearlyEqual(geometry.hip_offset_y_mm, 36.2f) &&
                   nearlyEqual(geometry.thigh_length_mm, 50.0f) &&
                   nearlyEqual(geometry.shin_length_mm, 50.0f),
               "Leg geometry matches ADR-008 dimensions")) return 1;

    const tilt::motion::LegJointAngles home_leg_angles{
        0.0f,
        -twenty_degrees,
        twenty_degrees,
    };
    const auto left_home_ankle = tilt::motion::forwardAnkleKinematics(
        geometry, tilt::motion::LegSide::Left, home_leg_angles);
    const auto right_home_ankle = tilt::motion::forwardAnkleKinematics(
        geometry, tilt::motion::LegSide::Right, home_leg_angles);
    const tilt::motion::CartesianPoint expected_left_home_ankle{
        17.101007f,
        36.2f,
        -96.984634f,
    };
    const tilt::motion::CartesianPoint expected_right_home_ankle{
        17.101007f,
        -36.2f,
        -96.984634f,
    };
    if (!check(left_home_ankle.has_value() && right_home_ankle.has_value() &&
                   positionsEqual(*left_home_ankle,
                                  expected_left_home_ankle,
                                  1.0e-4f) &&
                   positionsEqual(*right_home_ankle,
                                  expected_right_home_ankle,
                                  1.0e-4f),
               "Forward kinematics mirrors the documented Hip offsets")) return 1;

    const auto recovered_home_angles = tilt::motion::inverseAnkleKinematics(
        geometry,
        tilt::motion::LegSide::Left,
        *left_home_ankle,
        home_leg_angles.hip_yaw_rad);
    if (!check(recovered_home_angles.has_value() &&
                   nearlyEqual(recovered_home_angles->hip_yaw_rad,
                               home_leg_angles.hip_yaw_rad) &&
                   nearlyEqual(recovered_home_angles->hip_pitch_rad,
                               home_leg_angles.hip_pitch_rad,
                               1.0e-4f) &&
                   nearlyEqual(recovered_home_angles->knee_pitch_rad,
                               home_leg_angles.knee_pitch_rad,
                               1.0e-4f),
               "Inverse kinematics recovers the Home leg angles")) return 1;

    const tilt::motion::LegJointAngles backward_leg_angles{
        0.0f,
        twenty_degrees * 0.5f,
        twenty_degrees,
    };
    const auto backward_ankle = tilt::motion::forwardAnkleKinematics(
        geometry, tilt::motion::LegSide::Right, backward_leg_angles);
    const auto recovered_backward_angles =
        backward_ankle.has_value()
            ? tilt::motion::inverseAnkleKinematics(
                  geometry,
                  tilt::motion::LegSide::Right,
                  *backward_ankle,
                  backward_leg_angles.hip_yaw_rad)
            : std::nullopt;
    if (!check(recovered_backward_angles.has_value() &&
                   nearlyEqual(recovered_backward_angles->hip_pitch_rad,
                               backward_leg_angles.hip_pitch_rad,
                               1.0e-4f) &&
                   nearlyEqual(recovered_backward_angles->knee_pitch_rad,
                               backward_leg_angles.knee_pitch_rad,
                               1.0e-4f),
               "Inverse kinematics preserves a backward ankle target")) return 1;

    constexpr float half_pi = 1.5707963268f;
    tilt::motion::LegJointAngles yawed_angles = home_leg_angles;
    yawed_angles.hip_yaw_rad = half_pi;
    const auto yawed_ankle = tilt::motion::forwardAnkleKinematics(
        geometry, tilt::motion::LegSide::Left, yawed_angles);
    if (!check(yawed_ankle.has_value() &&
                   nearlyEqual(yawed_ankle->x_mm, 0.0f, 1.0e-4f) &&
                   nearlyEqual(yawed_ankle->y_mm,
                               36.2f + expected_left_home_ankle.x_mm,
                               1.0e-4f),
               "Positive Hip Yaw rotates the leg plane toward robot-left")) return 1;

    tilt::motion::CartesianPoint off_plane_target = expected_left_home_ankle;
    off_plane_target.y_mm += 1.0f;
    const tilt::motion::CartesianPoint unreachable_target{
        101.0f,
        36.2f,
        0.0f,
    };
    if (!check(!tilt::motion::inverseAnkleKinematics(
                    geometry,
                    tilt::motion::LegSide::Left,
                    off_plane_target,
                    0.0f).has_value() &&
                   !tilt::motion::inverseAnkleKinematics(
                    geometry,
                    tilt::motion::LegSide::Left,
                    unreachable_target,
                    0.0f).has_value(),
               "Inverse kinematics rejects off-plane and unreachable targets")) return 1;

    tilt::motion::LegGeometry invalid_geometry = geometry;
    invalid_geometry.shin_length_mm = 0.0f;
    if (!check(!tilt::motion::forwardAnkleKinematics(
                    invalid_geometry,
                    tilt::motion::LegSide::Left,
                    home_leg_angles).has_value(),
               "Kinematics rejects invalid geometry")) return 1;

    const auto home_ankles =
        tilt::motion::forwardRobotAnkleKinematics(home);
    const auto recovered_home_pose =
        home_ankles.has_value()
            ? tilt::motion::inverseRobotAnkleKinematics(
                  *home_ankles, 0.0f, 0.0f)
            : std::nullopt;
    if (!check(home_ankles.has_value() && recovered_home_pose.has_value() &&
                   posesEqual(*recovered_home_pose, home, 1.0e-4f),
               "Robot FK and IK round-trip the documented Home Pose")) return 1;

    JointPose asymmetric_pose = home;
    asymmetric_pose.angle_rad[static_cast<std::size_t>(JointIndex::LeftHipYaw)] =
        0.1f;
    asymmetric_pose.angle_rad[static_cast<std::size_t>(JointIndex::LeftHipPitch)] =
        -0.25f;
    asymmetric_pose.angle_rad[static_cast<std::size_t>(JointIndex::LeftKneePitch)] =
        0.45f;
    asymmetric_pose.angle_rad[static_cast<std::size_t>(JointIndex::RightHipYaw)] =
        -0.08f;
    asymmetric_pose.angle_rad[static_cast<std::size_t>(JointIndex::RightHipPitch)] =
        0.12f;
    asymmetric_pose.angle_rad[static_cast<std::size_t>(JointIndex::RightKneePitch)] =
        0.38f;
    const auto asymmetric_ankles =
        tilt::motion::forwardRobotAnkleKinematics(asymmetric_pose);
    const auto recovered_asymmetric_pose =
        asymmetric_ankles.has_value()
            ? tilt::motion::inverseRobotAnkleKinematics(
                  *asymmetric_ankles, 0.1f, -0.08f)
            : std::nullopt;
    if (!check(recovered_asymmetric_pose.has_value() &&
                   posesEqual(*recovered_asymmetric_pose,
                              asymmetric_pose,
                              1.0e-4f),
               "Robot FK and IK round-trip an asymmetric Pose")) return 1;

    const auto home_contacts =
        tilt::motion::forwardRobotContactKinematics(home);
    const tilt::motion::CartesianPoint expected_left_contact{
        0.0f,
        36.2f,
        -104.0f,
    };
    const tilt::motion::CartesianPoint expected_right_contact{
        0.0f,
        -36.2f,
        -104.0f,
    };
    if (!check(home_contacts.has_value() &&
                   positionsEqual(home_contacts->left,
                                  expected_left_contact,
                                  1.0e-4f) &&
                   positionsEqual(home_contacts->right,
                                  expected_right_contact,
                                  1.0e-4f),
               "Provisional contact geometry reproduces the ADR-008 Home contacts")) return 1;

    const auto recovered_contact_home =
        home_contacts.has_value()
            ? tilt::motion::inverseRobotContactKinematics(
                  *home_contacts, 0.0f, 0.0f)
            : std::nullopt;
    if (!check(recovered_contact_home.has_value() &&
                   posesEqual(*recovered_contact_home, home, 1.0e-4f),
               "Contact FK and IK round-trip the Home Pose")) return 1;

    const auto asymmetric_contacts =
        tilt::motion::forwardRobotContactKinematics(asymmetric_pose);
    const auto recovered_contact_asymmetric =
        asymmetric_contacts.has_value()
            ? tilt::motion::inverseRobotContactKinematics(
                  *asymmetric_contacts, 0.1f, -0.08f)
            : std::nullopt;
    if (!check(recovered_contact_asymmetric.has_value() &&
                   posesEqual(*recovered_contact_asymmetric,
                              asymmetric_pose,
                              1.0e-4f),
               "Contact FK and IK round-trip an asymmetric Pose")) return 1;

    const tilt::motion::GaitTiming gait_timing{0.2f, 0.8f};
    const auto gait_duration =
        tilt::motion::gaitCycleDurationSeconds(gait_timing);
    if (!check(gait_duration.has_value() &&
                   nearlyEqual(*gait_duration, 2.0f),
               "Gait cycle duration contains two support and swing phases")) return 1;

    const auto gait_start = tilt::motion::sampleGaitPhase(
        gait_timing, tilt::motion::LegSide::Right, -1.0f);
    if (!check(gait_start.has_value() &&
                   gait_start->support_mode == tilt::motion::SupportMode::Double &&
                   !gait_start->swing_leg.has_value() &&
                   gait_start->next_swing_leg == tilt::motion::LegSide::Right &&
                   nearlyEqual(gait_start->phase_progress, 0.0f),
               "Gait starts in Double Support before the selected first swing")) return 1;

    const auto first_swing = tilt::motion::sampleGaitPhase(
        gait_timing, tilt::motion::LegSide::Right, 0.6f);
    if (!check(first_swing.has_value() &&
                   first_swing->support_mode == tilt::motion::SupportMode::Left &&
                   first_swing->swing_leg == tilt::motion::LegSide::Right &&
                   nearlyEqual(first_swing->phase_progress, 0.5f),
               "Right Swing reports Left support and normalized progress")) return 1;

    const auto middle_support = tilt::motion::sampleGaitPhase(
        gait_timing, tilt::motion::LegSide::Right, 1.1f);
    if (!check(middle_support.has_value() &&
                   middle_support->support_mode == tilt::motion::SupportMode::Double &&
                   !middle_support->swing_leg.has_value() &&
                   middle_support->next_swing_leg == tilt::motion::LegSide::Left &&
                   nearlyEqual(middle_support->phase_progress, 0.5f),
               "Middle Double Support announces the opposite swing leg")) return 1;

    const auto second_swing = tilt::motion::sampleGaitPhase(
        gait_timing, tilt::motion::LegSide::Right, 1.6f);
    if (!check(second_swing.has_value() &&
                   second_swing->support_mode == tilt::motion::SupportMode::Right &&
                   second_swing->swing_leg == tilt::motion::LegSide::Left &&
                   nearlyEqual(second_swing->phase_progress, 0.5f),
               "Left Swing reports Right support and normalized progress")) return 1;

    const auto wrapped_gait = tilt::motion::sampleGaitPhase(
        gait_timing, tilt::motion::LegSide::Right, 2.0f);
    if (!check(wrapped_gait.has_value() &&
                   wrapped_gait->support_mode == tilt::motion::SupportMode::Double &&
                   wrapped_gait->next_swing_leg == tilt::motion::LegSide::Right &&
                   nearlyEqual(wrapped_gait->phase_progress, 0.0f),
               "Gait phase wraps exactly at the cycle boundary")) return 1;

    const tilt::motion::GaitTiming invalid_gait_timing{0.0f, 0.8f};
    if (!check(!tilt::motion::sampleGaitPhase(
                    invalid_gait_timing,
                    tilt::motion::LegSide::Right,
                    0.0f).has_value(),
               "Gait phase rejects non-positive phase durations")) return 1;

    tilt::motion::CartesianPoint right_step_target = home_ankles->right;
    right_step_target.x_mm += 5.0f;
    const auto step_apex = tilt::motion::sampleBipedStep(
        *home_ankles,
        tilt::motion::LegSide::Right,
        right_step_target,
        5.0f,
        0.5f,
        1.0f);
    if (!check(step_apex.has_value() &&
                   positionsEqual(step_apex->position.left,
                                  home_ankles->left) &&
                   nearlyEqual(step_apex->position.right.x_mm,
                               home_ankles->right.x_mm + 2.5f,
                               1.0e-4f) &&
                   nearlyEqual(step_apex->position.right.z_mm,
                               home_ankles->right.z_mm + 5.0f,
                               1.0e-4f) &&
                   positionsEqual(step_apex->velocity_mm_per_sec.left, {}) &&
                   !step_apex->finished,
               "Biped step holds the support Ankle and lifts the swing Ankle")) return 1;

    const auto step_apex_pose =
        step_apex.has_value()
            ? tilt::motion::inverseRobotAnkleKinematics(
                  step_apex->position, 0.0f, 0.0f)
            : std::nullopt;
    if (!check(step_apex_pose.has_value(),
               "Biped step apex remains inside the IK workspace")) return 1;

    const auto direct_step_pose = tilt::motion::sampleBipedStepPose(
        *home_ankles,
        tilt::motion::LegSide::Right,
        right_step_target,
        0.0f,
        0.0f,
        5.0f,
        0.5f,
        1.0f);
    const auto direct_step_ankles =
        direct_step_pose.has_value()
            ? tilt::motion::forwardRobotAnkleKinematics(direct_step_pose->pose)
            : std::nullopt;
    if (!check(direct_step_pose.has_value() &&
                   direct_step_ankles.has_value() &&
                   positionsEqual(direct_step_ankles->left,
                                  step_apex->position.left,
                                  1.0e-4f) &&
                   positionsEqual(direct_step_ankles->right,
                                  step_apex->position.right,
                                  1.0e-4f) &&
                   !direct_step_pose->finished,
               "Biped step converts directly from Cartesian trajectory to JointPose")) return 1;

    const auto completed_step = tilt::motion::sampleBipedStep(
        *home_ankles,
        tilt::motion::LegSide::Right,
        right_step_target,
        5.0f,
        1.0f,
        1.0f);
    if (!check(completed_step.has_value() && completed_step->finished &&
                   positionsEqual(completed_step->position.left,
                                  home_ankles->left) &&
                   positionsEqual(completed_step->position.right,
                                  right_step_target) &&
                   positionsEqual(completed_step->velocity_mm_per_sec.right, {}) &&
                   positionsEqual(completed_step->acceleration_mm_per_sec2.right, {}),
               "Biped step lands at the target while support remains fixed")) return 1;

    tilt::motion::CartesianPoint right_contact_target = home_contacts->right;
    right_contact_target.x_mm += 5.0f;
    const auto contact_step_apex = tilt::motion::sampleContactStep(
        *home_contacts,
        tilt::motion::LegSide::Right,
        right_contact_target,
        5.0f,
        0.5f,
        1.0f);
    if (!check(contact_step_apex.has_value() &&
                   positionsEqual(contact_step_apex->position.left,
                                  home_contacts->left) &&
                   nearlyEqual(contact_step_apex->position.right.x_mm,
                               2.5f,
                               1.0e-4f) &&
                   nearlyEqual(contact_step_apex->position.right.z_mm,
                               -99.0f,
                               1.0e-4f),
               "Contact step holds support and lifts the swing contact")) return 1;

    const auto contact_step_pose = tilt::motion::sampleContactStepPose(
        *home_contacts,
        tilt::motion::LegSide::Right,
        right_contact_target,
        0.0f,
        0.0f,
        5.0f,
        0.5f,
        1.0f);
    const auto recovered_contact_step =
        contact_step_pose.has_value()
            ? tilt::motion::forwardRobotContactKinematics(
                  contact_step_pose->pose)
            : std::nullopt;
    if (!check(contact_step_pose.has_value() &&
                   recovered_contact_step.has_value() &&
                   positionsEqual(recovered_contact_step->left,
                                  contact_step_apex->position.left,
                                  1.0e-4f) &&
                   positionsEqual(recovered_contact_step->right,
                                  contact_step_apex->position.right,
                                  1.0e-4f),
               "Contact trajectory converts to JointPose and back")) return 1;

    std::puts("Pose motion host tests passed");
    return 0;
}
