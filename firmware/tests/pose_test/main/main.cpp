#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <strings.h>

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "tilt/sts3215/Sts3215Bus.h"
#include "tilt_config.h"
#include "tilt_kinematics.h"
#include "tilt_mpu6050.h"

#include "Interpolator.h"
#include "JointMapper.h"
#include "PoseTestConfig.h"
#include "FlatPose.h"

namespace {

constexpr char kTag[] = "pose_test";
constexpr std::size_t kConsoleLineCapacity = 96;
constexpr UBaseType_t kCommandQueueDepth = 20;
constexpr float kRadToDeg = 57.2957795130823208768f;

enum class SafetyState : std::uint8_t { DISARMED, ARMED, ESTOP };
enum class RockAutomation : std::uint8_t { NONE, SWEEP, ALTERNATE };
enum class RockSweepPhase : std::uint8_t { MOVING, HOLDING };
enum class RockMethod : std::uint8_t { IK, FLAT };
enum class StepTestPhase : std::uint8_t { IDLE, DISTURBANCE, RECORDING };
enum class CommandType : std::uint8_t {
    HELP,
    STATUS,
    CHECK,
    ARM,
    DISARM,
    RECOVER,
    HOME,
    POSE,
    JOINT,
    FK,
    IK_ENTER,
    IK_UP,
    IK_DOWN,
    IK_RIGHT,
    IK_LEFT,
    IK_TOGGLE_STEP,
    IK_HOME,
    IK_VERIFY,
    IK_QUIT,
    STAND_ENTER,
    STAND_UP,
    STAND_DOWN,
    STAND_TOGGLE_COMP,
    STAND_KP_UP,
    STAND_KP_DOWN,
    STAND_LPF_DOWN,
    STAND_LPF_UP,
    STAND_RESET_ATTITUDE,
    STAND_ZERO,
    STAND_STEP_TEST,
    STAND_VERIFY,
    STAND_QUIT,
    ROCK_ENTER,
    ROCK_CONFIRM,
    ROCK_CANCEL_ENTRY,
    ROCK_UP,
    ROCK_DOWN,
    ROCK_RIGHT,
    ROCK_LEFT,
    ROCK_TOGGLE_STEP,
    ROCK_TOGGLE_METHOD,
    ROCK_TOGGLE_PITCH_COMP,
    ROCK_TOGGLE_CENTER_COMP,
    ROCK_TOGGLE_AMP_COMP,
    ROCK_KP_UP,
    ROCK_KP_DOWN,
    ROCK_ZERO,
    ROCK_SWEEP,
    ROCK_ALTERNATE,
    ROCK_RECORD,
    ROCK_RESET_ROLL,
    ROCK_VERIFY,
    ROCK_QUIT,
    ROCK_PERIOD_DOWN,
    ROCK_PERIOD_UP,
    ROCK_CANCEL_AUTOMATION,
    ROCK_CANCEL_AUTOMATION_KEEP_POSITION,
    WALK_ENTER,
    WALK_CONFIRM,
    WALK_CANCEL_ENTRY,
    WALK_TOGGLE,
    WALK_UP,
    WALK_DOWN,
    WALK_RIGHT,
    WALK_LEFT,
    WALK_DELTA_UP,
    WALK_DELTA_DOWN,
    WALK_PERIOD_DOWN,
    WALK_PERIOD_UP,
    WALK_TOGGLE_PITCH_COMP,
    WALK_TOGGLE_CENTER_COMP,
    WALK_ZERO_X,
    WALK_RESET_ATTITUDE,
    WALK_VERIFY,
    WALK_QUIT,
};

struct Command {
    CommandType type;
    char name[16]{};
    float value = 0.0f;
};

constexpr tilt::sts3215::BusConfig kBusConfig{
    static_cast<uart_port_t>(tilt::SERVO_UART_PORT),
    static_cast<gpio_num_t>(tilt::SERVO_UART_TX_PIN),
    static_cast<gpio_num_t>(tilt::SERVO_UART_RX_PIN),
    tilt::SERVO_UART_BAUD,
    50,
};

tilt::sts3215::Sts3215Bus g_bus(kBusConfig);
tilt_pose_test::Interpolator g_interpolator;
tilt::ComplementaryFilter g_imu_filter;
QueueHandle_t g_command_queue = nullptr;
std::atomic<SafetyState> g_state{SafetyState::DISARMED};
std::atomic<bool> g_estop_requested{false};
std::atomic<bool> g_ik_mode{false};
std::atomic<bool> g_stand_mode{false};
std::atomic<bool> g_stand_ready{false};
std::atomic<bool> g_rock_mode{false};
std::atomic<bool> g_rock_ready{false};
std::atomic<bool> g_rock_confirmation_pending{false};
std::atomic<RockAutomation> g_rock_automation{RockAutomation::NONE};
std::atomic<bool> g_walk_mode{false};
std::atomic<bool> g_walk_ready{false};
std::atomic<bool> g_walk_confirmation_pending{false};
std::atomic<bool> g_walk_running{false};
float g_goal_rad[tilt::NUM_JOINTS]{};
float g_body_x_mm = 0.0f;
float g_body_height_mm = tilt::ZERO_POSE_HEIGHT_MM;
bool g_ik_coarse = false;
bool g_start_ik_when_idle = false;
bool g_start_stand_when_idle = false;
bool g_start_rock_when_idle = false;
bool g_start_walk_when_idle = false;
bool g_speed_guard_active = false;
int g_speed_guard_joint = -1;

bool g_imu_available = false;
bool g_roll_valid = false;
float g_roll_deg = 0.0f;
bool g_pitch_valid = false;
float g_pitch_deg = 0.0f;
std::int64_t g_last_imu_sample_us = 0;
bool g_roll_zero_valid = false;
float g_roll_zero_deg = 0.0f;
bool g_pitch_zero_valid = false;
float g_pitch_zero_deg = 0.0f;
std::uint32_t g_imu_consecutive_failures = 0;
bool g_imu_failure_reported = false;
bool g_imu_comp_fault_requested = false;

bool g_pitch_comp_enabled = false;
float g_comp_kp_pitch = tilt_pose_test::kCompKpPitchDefault;
float g_comp_lpf_alpha = tilt_pose_test::kCompLpfAlpha;
float g_comp_body_x_mm = 0.0f;
float g_comp_body_x_min_mm = 0.0f;
float g_comp_body_x_max_mm = 0.0f;
std::uint32_t g_last_status_print_ms = 0;

float g_stand_base_height_mm = tilt::ZERO_POSE_HEIGHT_MM;
float g_stand_last_sent_body_x_mm = 0.0f;
float g_stand_last_sent_height_mm = tilt::ZERO_POSE_HEIGHT_MM;
StepTestPhase g_step_test_phase = StepTestPhase::IDLE;
bool g_step_test_disturbance_arrived = false;
std::uint32_t g_step_test_phase_started_ms = 0;
bool g_step_test_was_unsettled = false;
bool g_step_test_settled = false;
std::uint32_t g_step_test_settle_ms = 0;
float g_step_test_peak_pitch_deg = 0.0f;
float g_step_test_last_pitch_deg = 0.0f;
int g_step_test_last_sign = 0;
std::uint32_t g_step_test_sign_reversals = 0;

float g_rock_delta_mm = 0.0f;
float g_rock_applied_delta_mm = 0.0f;
float g_rock_base_height_mm = tilt::ZERO_POSE_HEIGHT_MM;
float g_rock_max_abs_delta_mm = 0.0f;
bool g_rock_coarse = false;
RockMethod g_rock_method = RockMethod::FLAT;
float g_rock_target_rad[tilt::NUM_JOINTS]{};
bool g_rock_target_valid = false;
std::uint32_t g_rock_alternate_period_ms =
    tilt_pose_test::kRockAlternatePeriodMs;
std::uint32_t g_rock_next_action_ms = 0;
float g_rock_alternate_amplitude_mm = 0.0f;
std::uint32_t g_rock_alternate_move_duration_ms =
    tilt_pose_test::kMinMoveDurationMs;
RockSweepPhase g_rock_sweep_phase = RockSweepPhase::MOVING;
float g_rock_sweep_max_delta_mm = 0.0f;
bool g_rock_sweep_roll_valid = false;
float g_rock_sweep_roll_deg = 0.0f;
bool g_warn_rock_zero_on_arrival = false;
bool g_rock_center_comp_enabled = false;
bool g_rock_amp_comp_enabled = false;
float g_rock_delta_bias_mm = 0.0f;
float g_rock_delta_scale = 1.0f;
float g_rock_last_applied_body_x_mm = 0.0f;
bool g_rock_cycle_roll_valid = false;
bool g_rock_cycle_started = false;
float g_rock_cycle_roll_min_deg = 0.0f;
float g_rock_cycle_roll_max_deg = 0.0f;
bool g_rock_first_mid_valid = false;
float g_rock_first_mid_deg = 0.0f;
float g_rock_last_mid_deg = 0.0f;

struct AlternateStats {
    bool active = false;
    bool move_pending = false;
    std::uint32_t cycles = 0;
    bool roll_valid = false;
    float roll_min_deg = 0.0f;
    float roll_max_deg = 0.0f;
    bool pitch_valid = false;
    float pitch_min_deg = 0.0f;
    float pitch_max_deg = 0.0f;
};

AlternateStats g_alternate_stats{};

struct FootLiftRecord {
    bool valid = false;
    float delta_mm = 0.0f;
    bool roll_valid = false;
    float roll_deg = 0.0f;
};

FootLiftRecord g_left_lift{};
FootLiftRecord g_right_lift{};

struct WalkLegState {
    std::uint8_t phase = 0;
    float x_mm = 0.0f;
    bool short_leg = false;
};

struct WalkStats {
    std::uint32_t cycles = 0;
    std::uint32_t transitions = 0;
    std::uint32_t rejected_steps = 0;
    std::uint32_t consecutive_rejects = 0;
    bool roll_valid = false;
    float roll_min_deg = 0.0f;
    float roll_max_deg = 0.0f;
    bool pitch_valid = false;
    float pitch_min_deg = 0.0f;
    float pitch_max_deg = 0.0f;
    bool cycle_roll_valid = false;
    float cycle_roll_min_deg = 0.0f;
    float cycle_roll_max_deg = 0.0f;
};

WalkLegState g_walk_left{};
WalkLegState g_walk_right{};
WalkStats g_walk_stats{};
float g_walk_legx_mm = 0.0f;
float g_walk_delta_mm = tilt_pose_test::kWalkDeltaDefaultMm;
float g_walk_base_height_mm = tilt::ZERO_POSE_HEIGHT_MM;
float g_walk_applied_delta_mm = 0.0f;
float g_walk_last_left_x_mm = 0.0f;
float g_walk_last_right_x_mm = 0.0f;
float g_walk_last_left_height_mm = tilt::ZERO_POSE_HEIGHT_MM;
float g_walk_last_right_height_mm = tilt::ZERO_POSE_HEIGHT_MM;
std::uint32_t g_walk_leg_time_ms = tilt_pose_test::kWalkLegTimeDefaultMs;
std::uint32_t g_walk_next_action_ms = 0;
bool g_walk_last_cycle_roll_valid = false;
float g_walk_last_cycle_roll_min_deg = 0.0f;
float g_walk_last_cycle_roll_max_deg = 0.0f;
float g_walk_summary_legx_mm = 0.0f;
float g_walk_summary_delta_mm = tilt_pose_test::kWalkDeltaDefaultMm;
std::uint32_t g_walk_summary_leg_time_ms =
    tilt_pose_test::kWalkLegTimeDefaultMs;

void resetCompensationOutputs();
void stopWalkMotion(const char* reason, bool return_home);

const char* stateName(SafetyState state) {
    switch (state) {
        case SafetyState::DISARMED: return "DISARMED";
        case SafetyState::ARMED: return "ARMED";
        case SafetyState::ESTOP: return "ESTOP";
    }
    return "UNKNOWN";
}

std::uint32_t nowMs() {
    return static_cast<std::uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

void noteImuReadFailure() {
    if (g_imu_consecutive_failures < UINT32_MAX) {
        ++g_imu_consecutive_failures;
    }
    if (g_imu_consecutive_failures >= 5 && !g_imu_failure_reported) {
        g_imu_failure_reported = true;
        g_imu_comp_fault_requested = true;
    }
}

void serviceImu() {
    if (!g_imu_available) {
        noteImuReadFailure();
        return;
    }
    tilt::ImuRaw raw{};
    if (!tilt::imu_read_raw(raw)) {
        noteImuReadFailure();
        return;
    }
    g_imu_consecutive_failures = 0;
    g_imu_failure_reported = false;
    const std::int64_t now_us = esp_timer_get_time();
    const float dt_s = g_last_imu_sample_us == 0
                           ? 0.0f
                           : static_cast<float>(now_us - g_last_imu_sample_us) /
                                 1'000'000.0f;
    g_last_imu_sample_us = now_us;
    const tilt::Attitude attitude = g_imu_filter.update(raw, dt_s);
    if (g_imu_filter.initialized()) {
        if (std::isfinite(attitude.roll_rad)) {
            g_roll_deg = attitude.roll_rad * kRadToDeg;
            g_roll_valid = true;
        }
        if (std::isfinite(attitude.pitch_rad)) {
            g_pitch_deg = attitude.pitch_rad * kRadToDeg;
            g_pitch_valid = true;
        }
    }
}

bool isArmed(const char* command) {
    if (g_state.load() == SafetyState::ARMED) {
        return true;
    }
    std::printf("%s: ARMED 상태에서만 실행할 수 있습니다 (현재 %s).\n",
                command, stateName(g_state.load()));
    return false;
}

bool busOk(esp_err_t result, const char* operation) {
    if (result == ESP_OK) {
        return true;
    }
    ESP_LOGE(kTag, "%s failed: %s", operation, esp_err_to_name(result));
    return false;
}

void emergencyStopNow() {
    g_estop_requested.store(true);
    g_ik_mode.store(false);
    g_stand_mode.store(false);
    g_stand_ready.store(false);
    g_rock_mode.store(false);
    g_rock_ready.store(false);
    g_rock_confirmation_pending.store(false);
    g_rock_automation.store(RockAutomation::NONE);
    g_walk_mode.store(false);
    g_walk_ready.store(false);
    g_walk_confirmation_pending.store(false);
    g_walk_running.store(false);
    g_state.store(SafetyState::ESTOP);
    const esp_err_t result = g_bus.emergencyStop(
        tilt::SERVO_ID, tilt::NUM_JOINTS);
    if (result != ESP_OK) {
        ESP_LOGE(kTag, "E-STOP torque OFF failed: %s. Cut servo power now.",
                 esp_err_to_name(result));
    }
    std::printf("\n!! E-STOP: torque OFF requested. Keep physical power cut within reach. !!\n");
}

void torqueOffBestEffort() {
    const esp_err_t result = g_bus.setTorqueAll(
        tilt::SERVO_ID, tilt::NUM_JOINTS, false);
    if (result != ESP_OK) {
        ESP_LOGE(kTag, "Torque OFF failed: %s. Cut servo power if necessary.",
                 esp_err_to_name(result));
    }
}

bool readRawPositions(std::uint16_t raw[tilt::NUM_JOINTS]) {
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const esp_err_t result = g_bus.readPosition(tilt::SERVO_ID[joint], raw[joint]);
        if (result != ESP_OK) {
            ESP_LOGE(kTag, "Read %s (ID %u) failed: %s", tilt::JOINT_NAME[joint],
                     tilt::SERVO_ID[joint], esp_err_to_name(result));
            return false;
        }
    }
    return true;
}

void rawToRadians(const std::uint16_t raw[tilt::NUM_JOINTS],
                  float rad[tilt::NUM_JOINTS]) {
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        rad[joint] = tilt_pose_test::tickToRad(joint, raw[joint]);
    }
}

bool targetWithinLogicalLimits(const float target[tilt::NUM_JOINTS]) {
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const int leg = joint < 3 ? 0 : 1;
        const int leg_joint = joint % 3;
        const tilt::JointLimit& limit = tilt::JOINT_LIMIT[leg][leg_joint];
        if (!std::isfinite(target[joint]) || target[joint] < limit.minimum_rad ||
            target[joint] > limit.maximum_rad) {
            std::printf("%s target %+0.2f deg rejected (limit %+0.2f .. %+0.2f deg).\n",
                        tilt::JOINT_NAME[joint], target[joint] * kRadToDeg,
                        limit.minimum_rad * kRadToDeg, limit.maximum_rad * kRadToDeg);
            return false;
        }
    }
    return true;
}

bool sendSixJointGoal(const float rad[tilt::NUM_JOINTS]) {
    std::uint16_t ticks[tilt::NUM_JOINTS]{};
    tilt_pose_test::radArrayToTicks(rad, ticks);
    return busOk(g_bus.syncWritePositions(tilt::SERVO_ID, ticks, tilt::NUM_JOINTS,
                                           tilt_pose_test::kServoSpeedRaw),
                 "syncWritePositions");
}

void printPositions() {
    std::uint16_t raw[tilt::NUM_JOINTS]{};
    if (!readRawPositions(raw)) {
        return;
    }
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        std::printf("  %-3s ID %u: raw %4u  logical %+7.2f deg\n",
                    tilt::JOINT_NAME[joint], tilt::SERVO_ID[joint], raw[joint],
                    tilt_pose_test::tickToRad(joint, raw[joint]) * kRadToDeg);
    }
}

void printStatus() {
    std::printf("state=%s  IK=%s  STAND=%s  ROCK=%s  WALK=%s\n", stateName(g_state.load()),
                g_ik_mode.load() ? "active" : "off",
                g_stand_mode.load() ? "active" : "off",
                g_rock_mode.load() ? "active" : "off",
                g_walk_mode.load() ? (g_walk_running.load() ? "running" : "stopped")
                                   : "off");
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const esp_err_t result = g_bus.ping(tilt::SERVO_ID[joint]);
        std::printf("  %-3s ID %u: %s\n", tilt::JOINT_NAME[joint],
                    tilt::SERVO_ID[joint], result == ESP_OK ? "OK" : esp_err_to_name(result));
    }
    printPositions();
}

void printCheck() {
    std::uint16_t raw[tilt::NUM_JOINTS]{};
    if (!readRawPositions(raw)) {
        return;
    }
    std::printf("zero-pose calibration check:\n");
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const float actual_deg = tilt_pose_test::tickToRad(joint, raw[joint]) * kRadToDeg;
        const float delta_deg = actual_deg - tilt::ZERO_POSE_RAD[joint] * kRadToDeg;
        std::printf("  %-3s actual %+7.2f, zero %+7.2f, difference %+7.2f deg\n",
                    tilt::JOINT_NAME[joint], actual_deg,
                    tilt::ZERO_POSE_RAD[joint] * kRadToDeg, delta_deg);
    }
}

void printHelp() {
    std::printf(
        "\nTILT pose_test commands (Enter to submit)\n"
        "  help | status | check | arm | disarm | recover | !\n"
        "  home | pose <home|tall|crouch|yaw-left|yaw-right>\n"
        "  joint <LHY|LHP|LKP|RHY|RHP|RKP> <-5..+5> | fk | ik | stand | rock | walk\n"
        "  In IK: arrows/WASD move, m fine/coarse, 0 home, v FK check, q exit.\n"
        "  In STAND: c comp, +/- Kp, [/] LPF, arrows height, t step test, q exit.\n"
        "  In ROCK: arrows/WASD move, f IK/FLAT, 1 sweep, 2 alternate, "
        "c/x/z compensation, k record, r attitude zero, v status, q exit.\n"
        "  In WALK: space start/stop, arrows/WASD tune, +/- delta, [/] timing, "
        "c/x compensation, 0 zero legx, r attitude zero, v status, q exit.\n\n");
}

std::uint32_t requiredDurationMs(const float from[tilt::NUM_JOINTS],
                                 const float to[tilt::NUM_JOINTS],
                                 std::uint32_t requested_ms) {
    float max_delta_rad = 0.0f;
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        max_delta_rad = std::fmax(max_delta_rad, std::fabs(to[joint] - from[joint]));
    }

    const float max_velocity_rad_s =
        tilt_pose_test::kMaxJointVelocityDegS * tilt::DEG2RAD;
    const auto velocity_limited_ms = static_cast<std::uint32_t>(std::ceil(
        max_delta_rad / max_velocity_rad_s * 1000.0f *
        tilt_pose_test::kDurationSafetyFactor));
    std::uint32_t duration_ms = requested_ms;
    if (duration_ms < velocity_limited_ms) {
        duration_ms = velocity_limited_ms;
    }
    if (duration_ms < tilt_pose_test::kMinMoveDurationMs) {
        duration_ms = tilt_pose_test::kMinMoveDurationMs;
    }
    return duration_ms;
}

bool beginMove(const float target[tilt::NUM_JOINTS], std::uint32_t duration_ms,
               bool begin_ik_after_arrival = false,
               bool begin_rock_after_arrival = false,
               bool begin_stand_after_arrival = false,
               bool begin_walk_after_arrival = false) {
    if (!isArmed("move") || !targetWithinLogicalLimits(target)) {
        return false;
    }
    g_start_ik_when_idle = begin_ik_after_arrival;
    g_start_rock_when_idle = begin_rock_after_arrival;
    g_start_stand_when_idle = begin_stand_after_arrival;
    g_start_walk_when_idle = begin_walk_after_arrival;
    g_interpolator.start(g_goal_rad, target,
                         requiredDurationMs(g_goal_rad, target, duration_ms), nowMs());
    return true;
}

void arm() {
    if (g_state.load() != SafetyState::DISARMED) {
        std::printf("arm requires DISARMED state (current %s).\n", stateName(g_state.load()));
        return;
    }

    std::uint16_t current_ticks[tilt::NUM_JOINTS]{};
    if (!readRawPositions(current_ticks)) {
        std::printf("arm rejected: could not read all six current positions.\n");
        return;
    }
    rawToRadians(current_ticks, g_goal_rad);

    // Record and write the present position before torque ON so an old hardware
    // goal cannot pull the robot when torque is restored.
    if (!busOk(g_bus.syncWritePositions(tilt::SERVO_ID, current_ticks, tilt::NUM_JOINTS,
                                        tilt_pose_test::kServoSpeedRaw),
               "arm hold target")) {
        return;
    }
    g_speed_guard_active = false;
    g_speed_guard_joint = -1;
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        if (!busOk(g_bus.setAcceleration(tilt::SERVO_ID[joint],
                                         tilt_pose_test::kServoAcceleration),
                   "setAcceleration")) {
            torqueOffBestEffort();
            return;
        }
    }
    if (!busOk(g_bus.setTorqueAll(tilt::SERVO_ID, tilt::NUM_JOINTS, true),
               "setTorqueAll ON")) {
        torqueOffBestEffort();
        return;
    }
    g_state.store(SafetyState::ARMED);
    std::printf("ARMED: current six-joint position is held. '!' is E-STOP.\n");
}

void disarm() {
    if (g_state.load() == SafetyState::ESTOP) {
        std::printf("ESTOP remains active; use recover, then arm.\n");
        return;
    }
    g_interpolator.abort();
    g_start_ik_when_idle = false;
    g_start_stand_when_idle = false;
    g_start_rock_when_idle = false;
    g_start_walk_when_idle = false;
    g_ik_mode.store(false);
    g_stand_mode.store(false);
    g_stand_ready.store(false);
    g_rock_mode.store(false);
    g_rock_ready.store(false);
    g_rock_confirmation_pending.store(false);
    g_rock_automation.store(RockAutomation::NONE);
    g_walk_mode.store(false);
    g_walk_ready.store(false);
    g_walk_confirmation_pending.store(false);
    g_walk_running.store(false);
    g_roll_zero_valid = false;
    g_pitch_zero_valid = false;
    g_warn_rock_zero_on_arrival = false;
    g_alternate_stats = {};
    g_pitch_comp_enabled = false;
    g_rock_center_comp_enabled = false;
    g_rock_amp_comp_enabled = false;
    g_comp_body_x_mm = 0.0f;
    if (g_step_test_phase != StepTestPhase::IDLE) {
        g_step_test_phase = StepTestPhase::IDLE;
        std::printf("Step response test aborted.\n");
    }
    resetCompensationOutputs();
    torqueOffBestEffort();
    g_state.store(SafetyState::DISARMED);
    std::printf("DISARMED: torque OFF requested.\n");
}

void recover() {
    if (g_state.load() != SafetyState::ESTOP) {
        std::printf("recover is only needed in ESTOP.\n");
        return;
    }
    torqueOffBestEffort();
    bool all_reachable = true;
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        if (g_bus.ping(tilt::SERVO_ID[joint]) != ESP_OK) {
            all_reachable = false;
        }
    }
    if (!all_reachable) {
        std::printf("recover rejected: all six servos must respond first.\n");
        return;
    }
    g_estop_requested.store(false);
    g_state.store(SafetyState::DISARMED);
    std::printf("Recovered to DISARMED. Run arm before any move.\n");
}

void printFk() {
    std::uint16_t raw[tilt::NUM_JOINTS]{};
    float rad[tilt::NUM_JOINTS]{};
    if (!readRawPositions(raw)) {
        return;
    }
    rawToRadians(raw, rad);
    const tilt::Vec3 left = tilt::fk_foot(tilt::Leg::LEFT, rad);
    const tilt::Vec3 right = tilt::fk_foot(tilt::Leg::RIGHT, rad + 3);
    std::printf("FK foot positions (torso frame):\n"
                "  L (%+.2f, %+.2f, %+.2f) mm\n"
                "  R (%+.2f, %+.2f, %+.2f) mm\n",
                left.x, left.y, left.z, right.x, right.y, right.z);
}

void printIkVerification() {
    std::uint16_t raw[tilt::NUM_JOINTS]{};
    float rad[tilt::NUM_JOINTS]{};
    if (!readRawPositions(raw)) {
        return;
    }
    rawToRadians(raw, rad);
    const tilt::Vec3 target_left{-g_body_x_mm, +tilt::Y_HIP_MM, -g_body_height_mm};
    const tilt::Vec3 target_right{-g_body_x_mm, -tilt::Y_HIP_MM, -g_body_height_mm};
    const tilt::Vec3 actual_left = tilt::fk_foot(tilt::Leg::LEFT, rad);
    const tilt::Vec3 actual_right = tilt::fk_foot(tilt::Leg::RIGHT, rad + 3);
    std::printf("FK verification\n"
                "  L target (%+.2f, %+.2f, %+.2f), actual (%+.2f, %+.2f, %+.2f), "
                "error (%+.2f, %+.2f, %+.2f) mm\n"
                "  R target (%+.2f, %+.2f, %+.2f), actual (%+.2f, %+.2f, %+.2f), "
                "error (%+.2f, %+.2f, %+.2f) mm\n"
                "  Hint: both legs large = link dimensions/KNEE_OFFSET; one leg large = "
                "that leg's zero or JOINT_SIGN; pose-specific = IK or joint limits.\n",
                target_left.x, target_left.y, target_left.z,
                actual_left.x, actual_left.y, actual_left.z,
                actual_left.x - target_left.x, actual_left.y - target_left.y,
                actual_left.z - target_left.z,
                target_right.x, target_right.y, target_right.z,
                actual_right.x, actual_right.y, actual_right.z,
                actual_right.x - target_right.x, actual_right.y - target_right.y,
                actual_right.z - target_right.z);
}

void printIkState(const float left[3], const float right[3]) {
    std::printf("body: x=%+.2f h=%.2f [%s] | L yaw=%+.2f hip=%+.2f knee=%+.2f | "
                "R yaw=%+.2f hip=%+.2f knee=%+.2f\n",
                g_body_x_mm, g_body_height_mm, g_ik_coarse ? "COARSE" : "FINE",
                left[0] * kRadToDeg, left[1] * kRadToDeg, left[2] * kRadToDeg,
                right[0] * kRadToDeg, right[1] * kRadToDeg, right[2] * kRadToDeg);
}

void handleIkMove(float x_delta_mm, float height_delta_mm) {
    if (!g_ik_mode.load() || !isArmed("ik")) {
        return;
    }
    const float requested_x = g_body_x_mm + x_delta_mm;
    const float requested_height = g_body_height_mm + height_delta_mm;
    const float candidate_x = std::fmax(tilt_pose_test::kBodyXMinMm,
                                        std::fmin(tilt_pose_test::kBodyXMaxMm, requested_x));
    const float candidate_height = std::fmax(tilt_pose_test::kBodyHeightMinMm,
                                             std::fmin(tilt_pose_test::kBodyHeightMaxMm,
                                                       requested_height));
    if (candidate_x != requested_x || candidate_height != requested_height) {
        std::printf("IK workspace input clamped: x=%+.2f h=%.2f\n",
                    candidate_x, candidate_height);
    }

    const tilt::Vec3 left_target{-candidate_x, +tilt::Y_HIP_MM, -candidate_height};
    const tilt::Vec3 right_target{-candidate_x, -tilt::Y_HIP_MM, -candidate_height};
    tilt::IkResult left = tilt::ik_foot(tilt::Leg::LEFT, left_target, 0.0f);
    tilt::IkResult right = tilt::ik_foot(tilt::Leg::RIGHT, right_target, 0.0f);
    if (!left.reachable || !right.reachable) {
        std::printf("IK target rejected: unreachable; body position is unchanged.\n");
        return;
    }

    float unclamped_left[3] = {left.theta[0], left.theta[1], left.theta[2]};
    float unclamped_right[3] = {right.theta[0], right.theta[1], right.theta[2]};
    if (!tilt::clamp_to_limits(tilt::Leg::LEFT, left.theta) ||
        !tilt::clamp_to_limits(tilt::Leg::RIGHT, right.theta)) {
        for (int joint = 0; joint < 3; ++joint) {
            if (left.theta[joint] != unclamped_left[joint]) {
                std::printf("IK target rejected: %s reached its joint limit.\n",
                            tilt::JOINT_NAME[joint]);
            }
            if (right.theta[joint] != unclamped_right[joint]) {
                std::printf("IK target rejected: %s reached its joint limit.\n",
                            tilt::JOINT_NAME[joint + 3]);
            }
        }
        return;
    }

    float target[tilt::NUM_JOINTS] = {
        left.theta[0], left.theta[1], left.theta[2],
        right.theta[0], right.theta[1], right.theta[2],
    };
    if (!beginMove(target, tilt_pose_test::kIkStepDurationMs)) {
        return;
    }
    g_body_x_mm = candidate_x;
    g_body_height_mm = candidate_height;
    printIkState(left.theta, right.theta);
}

void enterIkMode() {
    if (!isArmed("ik")) {
        return;
    }
    g_body_x_mm = 0.0f;
    g_body_height_mm = tilt::ZERO_POSE_HEIGHT_MM;
    g_ik_coarse = false;
    if (beginMove(tilt::ZERO_POSE_RAD, tilt_pose_test::kPoseDurationMs, true)) {
        std::printf("IK will start after returning home.\n");
    }
}

bool buildStandTarget(float body_x_mm, float height_mm,
                      float target[tilt::NUM_JOINTS]) {
    const tilt::Vec3 left_target{-body_x_mm, +tilt::Y_HIP_MM, -height_mm};
    const tilt::Vec3 right_target{-body_x_mm, -tilt::Y_HIP_MM, -height_mm};
    tilt::IkResult left = tilt::ik_foot(tilt::Leg::LEFT, left_target, 0.0f);
    tilt::IkResult right = tilt::ik_foot(tilt::Leg::RIGHT, right_target, 0.0f);
    if (!left.reachable || !right.reachable) {
        std::printf("Stand target rejected: unreachable.\n");
        return false;
    }

    const float original_left[3] = {left.theta[0], left.theta[1], left.theta[2]};
    const float original_right[3] = {right.theta[0], right.theta[1], right.theta[2]};
    const bool left_within = tilt::clamp_to_limits(tilt::Leg::LEFT, left.theta);
    const bool right_within = tilt::clamp_to_limits(tilt::Leg::RIGHT, right.theta);
    if (!left_within || !right_within) {
        for (int joint = 0; joint < 3; ++joint) {
            if (left.theta[joint] != original_left[joint]) {
                std::printf("Stand target rejected: %s reached its joint limit.\n",
                            tilt::JOINT_NAME[joint]);
            }
            if (right.theta[joint] != original_right[joint]) {
                std::printf("Stand target rejected: %s reached its joint limit.\n",
                            tilt::JOINT_NAME[joint + 3]);
            }
        }
        return false;
    }
    for (int joint = 0; joint < 3; ++joint) {
        target[joint] = left.theta[joint];
        target[joint + 3] = right.theta[joint];
    }
    return true;
}

bool applyStandTarget(float body_x_mm, float height_mm,
                      std::uint32_t requested_duration_ms =
                          tilt_pose_test::kMinMoveDurationMs) {
    float target[tilt::NUM_JOINTS]{};
    if (!buildStandTarget(body_x_mm, height_mm, target) ||
        !beginMove(target, requested_duration_ms)) {
        return false;
    }
    g_stand_last_sent_body_x_mm = body_x_mm;
    g_stand_last_sent_height_mm = height_mm;
    return true;
}

void resetCompensationOutputs();

void enterStandMode() {
    if (!isArmed("stand")) {
        return;
    }
    std::printf("\nSTAND SAFETY: place the robot on the floor and keep both hands "
                "ready to catch it.\n");
    g_stand_mode.store(true);
    g_stand_ready.store(false);
    g_pitch_comp_enabled = false;
    g_rock_center_comp_enabled = false;
    g_rock_amp_comp_enabled = false;
    g_comp_body_x_mm = 0.0f;
    g_comp_body_x_mm = 0.0f;
    g_stand_base_height_mm = tilt::ZERO_POSE_HEIGHT_MM;
    g_step_test_phase = StepTestPhase::IDLE;
    if (beginMove(tilt::ZERO_POSE_RAD, tilt_pose_test::kPoseDurationMs,
                  false, false, true)) {
        std::printf("Returning home before stand mode starts.\n");
    } else {
        g_stand_mode.store(false);
    }
}

bool currentRelativeRoll(float& relative_roll_deg) {
    if (!g_roll_valid || !g_roll_zero_valid) {
        return false;
    }
    relative_roll_deg = g_roll_deg - g_roll_zero_deg;
    return true;
}

bool currentRelativePitch(float& relative_pitch_deg) {
    if (!g_pitch_valid || !g_pitch_zero_valid) {
        return false;
    }
    relative_pitch_deg = g_pitch_deg - g_pitch_zero_deg;
    return true;
}

void printRockState();
void printStandState();
void printWalkState();

void resetRockAttitudeZero() {
    if (!g_roll_valid && !g_pitch_valid) {
        std::printf("Cannot reset attitude zero: IMU attitude is unavailable.\n");
        return;
    }
    if (g_roll_valid) {
        g_roll_zero_deg = g_roll_deg;
        g_roll_zero_valid = true;
    }
    if (g_pitch_valid) {
        g_pitch_zero_deg = g_pitch_deg;
        g_pitch_zero_valid = true;
    }
    std::printf("Attitude zero reset: roll=");
    if (g_roll_zero_valid) {
        std::printf("%+.1fdeg", g_roll_zero_deg);
    } else {
        std::printf("--");
    }
    std::printf(" pitch=");
    if (g_pitch_zero_valid) {
        std::printf("%+.1fdeg.\n", g_pitch_zero_deg);
    } else {
        std::printf("--.\n");
    }
    if (g_stand_ready.load()) {
        printStandState();
    } else if (g_rock_ready.load()) {
        printRockState();
    } else if (g_walk_ready.load()) {
        printWalkState();
    }
}

void printRockState() {
    const float left_height =
        g_rock_base_height_mm - g_rock_applied_delta_mm * 0.5f;
    const float right_height =
        g_rock_base_height_mm + g_rock_applied_delta_mm * 0.5f;
    std::printf("[%s] delta=%+.1f applied=%+.1f  base=%.2f  L=%.2f R=%.2f  ",
                g_rock_method == RockMethod::FLAT ? "FLAT" : "IK",
                g_rock_delta_mm, g_rock_applied_delta_mm,
                g_rock_base_height_mm, left_height, right_height);

    if (!g_rock_target_valid) {
        std::printf("joints=--  ");
    } else if (g_rock_method == RockMethod::FLAT) {
        std::printf("a: L%.1f R%.1f  ",
                    g_rock_target_rad[tilt::L_KNEE_PITCH] * kRadToDeg,
                    g_rock_target_rad[tilt::R_KNEE_PITCH] * kRadToDeg);
    } else {
        const float left_foot_tilt =
            g_rock_target_rad[tilt::L_HIP_PITCH] +
            g_rock_target_rad[tilt::L_KNEE_PITCH];
        const float right_foot_tilt =
            g_rock_target_rad[tilt::R_HIP_PITCH] +
            g_rock_target_rad[tilt::R_KNEE_PITCH];
        std::printf("foot_tilt: L%+.1f R%+.1fdeg  ",
                    left_foot_tilt * kRadToDeg, right_foot_tilt * kRadToDeg);
    }

    if (g_roll_valid) {
        float relative_roll_deg = 0.0f;
        if (currentRelativeRoll(relative_roll_deg)) {
            std::printf("roll=%+.1f (rel %+.1f)  ", g_roll_deg, relative_roll_deg);
        } else {
            std::printf("roll=%+.1f (rel --)  ", g_roll_deg);
        }
    } else {
        std::printf("roll=-- (rel --)  ");
    }

    if (g_pitch_valid) {
        float relative_pitch_deg = 0.0f;
        if (currentRelativePitch(relative_pitch_deg)) {
            std::printf("pitch=%+.1f (rel %+.1f) [%s]\n",
                        g_pitch_deg, relative_pitch_deg,
                        g_rock_coarse ? "COARSE" : "FINE");
        } else {
            std::printf("pitch=%+.1f (rel --) [%s]\n",
                        g_pitch_deg, g_rock_coarse ? "COARSE" : "FINE");
        }
    } else {
        std::printf("pitch=-- (rel --) [%s]\n",
                    g_rock_coarse ? "COARSE" : "FINE");
    }
    std::printf("  comp pitch=%s(Kp%.1f) center=%s bias=%+.1fmm "
                "amp=%s scale=%.2f body_x=%+.1fmm\n",
                g_pitch_comp_enabled ? "ON" : "OFF", g_comp_kp_pitch,
                g_rock_center_comp_enabled ? "ON" : "OFF",
                g_rock_delta_bias_mm,
                g_rock_amp_comp_enabled ? "ON" : "OFF",
                g_rock_delta_scale, g_comp_body_x_mm);
}

void warnRockRollResidualIfNeeded() {
    if (!g_warn_rock_zero_on_arrival) {
        return;
    }
    g_warn_rock_zero_on_arrival = false;

    float relative_roll_deg = 0.0f;
    if (g_rock_mode.load() && std::fabs(g_rock_delta_mm) < 0.001f &&
        currentRelativeRoll(relative_roll_deg) && std::fabs(relative_roll_deg) > 1.0f) {
        std::printf("주의: delta=0 인데 roll 이 %+.1fdeg 남아 있습니다.\n"
                    "      곡면 발판이 굴러간 자리에서 돌아오지 않은 상태입니다.\n"
                    "      로봇을 손으로 바로 세운 뒤 r 로 기준을 다시 잡으세요.\n",
                    relative_roll_deg);
    }
}

bool rockLegWithinLimits(tilt::Leg leg, float theta[3]) {
    const float original[3] = {theta[0], theta[1], theta[2]};
    if (tilt::clamp_to_limits(leg, theta)) {
        return true;
    }
    const int offset = leg == tilt::Leg::LEFT ? 0 : 3;
    for (int joint = 0; joint < 3; ++joint) {
        if (theta[joint] != original[joint]) {
            std::printf("Rock target rejected: %s reached its joint limit.\n",
                        tilt::JOINT_NAME[joint + offset]);
        }
    }
    return false;
}

bool buildRockTargetForMethod(RockMethod method, float delta_mm,
                              float base_height_mm, float body_x_mm,
                              float target[tilt::NUM_JOINTS]) {
    const float left_height = base_height_mm - delta_mm * 0.5f;
    const float right_height = base_height_mm + delta_mm * 0.5f;
    float left_theta[3]{};
    float right_theta[3]{};

    if (method == RockMethod::FLAT) {
        const bool left_reachable =
            tilt_pose_test::flatPoseForHeightAndBodyX(
                left_height, body_x_mm, left_theta);
        const bool right_reachable =
            tilt_pose_test::flatPoseForHeightAndBodyX(
                right_height, body_x_mm, right_theta);
        if (!left_reachable || !right_reachable) {
            std::printf("FLAT rock target rejected: height outside workspace (%s%s%s).\n",
                        !left_reachable ? "LEFT" : "",
                        !left_reachable && !right_reachable ? "+" : "",
                        !right_reachable ? "RIGHT" : "");
            return false;
        }
    } else {
        const tilt::Vec3 left_target{-body_x_mm, +tilt::Y_HIP_MM, -left_height};
        const tilt::Vec3 right_target{-body_x_mm, -tilt::Y_HIP_MM, -right_height};
        tilt::IkResult left = tilt::ik_foot(tilt::Leg::LEFT, left_target, 0.0f);
        tilt::IkResult right = tilt::ik_foot(tilt::Leg::RIGHT, right_target, 0.0f);
        if (!left.reachable || !right.reachable) {
            std::printf("IK rock target rejected: unreachable (%s%s%s).\n",
                        !left.reachable ? "LEFT" : "",
                        !left.reachable && !right.reachable ? "+" : "",
                        !right.reachable ? "RIGHT" : "");
            return false;
        }
        for (int joint = 0; joint < 3; ++joint) {
            left_theta[joint] = left.theta[joint];
            right_theta[joint] = right.theta[joint];
        }
    }

    // clamp_to_limits is used only as a detector: a changed value rejects the
    // complete target and is never transmitted.
    const bool left_within_limits =
        rockLegWithinLimits(tilt::Leg::LEFT, left_theta);
    const bool right_within_limits =
        rockLegWithinLimits(tilt::Leg::RIGHT, right_theta);
    if (!left_within_limits || !right_within_limits) {
        return false;
    }

    for (int joint = 0; joint < 3; ++joint) {
        target[joint] = left_theta[joint];
        target[joint + 3] = right_theta[joint];
    }
    return true;
}

bool buildRockTarget(float delta_mm, float base_height_mm,
                     float target[tilt::NUM_JOINTS]) {
    return buildRockTargetForMethod(
        g_rock_method, delta_mm, base_height_mm,
        g_pitch_comp_enabled ? g_comp_body_x_mm : 0.0f, target);
}

bool applyRockTarget(float requested_delta_mm, float requested_base_height_mm,
                     std::uint32_t duration_ms, bool print_state = true) {
    const float commanded_delta_mm =
        std::fmax(-tilt_pose_test::kRockDeltaMaxMm,
                  std::fmin(tilt_pose_test::kRockDeltaMaxMm,
                            requested_delta_mm));
    const float base_height_mm = std::fmax(
        tilt_pose_test::kBodyHeightMinMm,
        std::fmin(tilt_pose_test::kBodyHeightMaxMm, requested_base_height_mm));
    if (commanded_delta_mm != requested_delta_mm ||
        base_height_mm != requested_base_height_mm) {
        std::printf("Rock input clamped: delta=%+.1f base=%.2f\n",
                    commanded_delta_mm, base_height_mm);
    }

    float applied_delta_mm = commanded_delta_mm;
    if (g_rock_automation.load() == RockAutomation::ALTERNATE) {
        applied_delta_mm = commanded_delta_mm * g_rock_delta_scale +
                           g_rock_delta_bias_mm;
    }
    if (std::fabs(applied_delta_mm) > tilt_pose_test::kRockDeltaMaxMm) {
        std::printf("Rock compensation rejected: applied delta %+.1fmm exceeds "
                    "limit ±%.1fmm.\n",
                    applied_delta_mm, tilt_pose_test::kRockDeltaMaxMm);
        return false;
    }

    float target[tilt::NUM_JOINTS]{};
    if (!buildRockTarget(applied_delta_mm, base_height_mm, target) ||
        !beginMove(target, duration_ms)) {
        return false;
    }
    g_rock_delta_mm = commanded_delta_mm;
    g_rock_applied_delta_mm = applied_delta_mm;
    g_rock_base_height_mm = base_height_mm;
    g_rock_max_abs_delta_mm = std::fmax(g_rock_max_abs_delta_mm,
                                        std::fabs(applied_delta_mm));
    std::memcpy(g_rock_target_rad, target, sizeof(g_rock_target_rad));
    g_rock_target_valid = true;
    g_rock_last_applied_body_x_mm =
        g_pitch_comp_enabled ? g_comp_body_x_mm : 0.0f;
    g_warn_rock_zero_on_arrival = std::fabs(applied_delta_mm) < 0.001f;
    if (print_state) {
        printRockState();
    }
    return true;
}

void toggleRockMethod() {
    if (!g_rock_ready.load()) {
        std::printf("Rock mode is not ready yet.\n");
        return;
    }
    if (g_rock_automation.load() != RockAutomation::NONE) {
        std::printf("Stop the active rock automation before changing method.\n");
        return;
    }

    const RockMethod previous_method = g_rock_method;
    g_rock_method = previous_method == RockMethod::FLAT
                        ? RockMethod::IK
                        : RockMethod::FLAT;
    if (!applyRockTarget(g_rock_delta_mm, g_rock_base_height_mm,
                         tilt_pose_test::kRockStepDurationMs, false)) {
        g_rock_method = previous_method;
        std::printf("Rock method change rejected; keeping %s.\n",
                    previous_method == RockMethod::FLAT ? "FLAT" : "IK");
        return;
    }
    std::printf("Rock method changed to %s.\n",
                g_rock_method == RockMethod::FLAT ? "FLAT" : "IK");
    printRockState();
}

void printLiftRecord(const char* label, const FootLiftRecord& record) {
    if (!record.valid) {
        std::printf("  %s: --\n", label);
    } else if (record.roll_valid) {
        std::printf("  %s: delta = %+.1fmm  (roll %+.1fdeg)\n",
                    label, record.delta_mm, record.roll_deg);
    } else {
        std::printf("  %s: delta = %+.1fmm  (roll --)\n", label, record.delta_mm);
    }
}

void printRockSummary() {
    std::printf("\n-- rocking test result --\n");
    printLiftRecord("left foot lift ", g_left_lift);
    printLiftRecord("right foot lift", g_right_lift);
    std::printf("  maximum |delta|: %.1fmm\n", g_rock_max_abs_delta_mm);
    const float available_range = tilt_pose_test::kBodyHeightMaxMm -
                                  tilt_pose_test::kBodyHeightMinMm;
    std::printf("  range usage: %.1f / %.1fmm\n", g_rock_max_abs_delta_mm,
                available_range);
    std::printf("\n  -> tilt_motion parameter suggestion\n"
                "      longLeg  = %.2fmm  (base_height)\n"
                "      shortLeg = %.2fmm  (base - maximum delta)\n",
                g_rock_base_height_mm,
                g_rock_base_height_mm - g_rock_max_abs_delta_mm);

    float recorded_delta = 0.0f;
    if (g_left_lift.valid) {
        recorded_delta = std::fmax(recorded_delta, std::fabs(g_left_lift.delta_mm));
    }
    if (g_right_lift.valid) {
        recorded_delta = std::fmax(recorded_delta, std::fabs(g_right_lift.delta_mm));
    }
    if (recorded_delta > 0.0f) {
        const float recommendation = std::fmin(
            tilt_pose_test::kRockDeltaMaxMm,
            recorded_delta + tilt_pose_test::kRockRecommendedMarginMm);
        std::printf("      recommended delta: %.1fmm (foot-lift point + %.1fmm)\n",
                    recommendation, tilt_pose_test::kRockRecommendedMarginMm);
    } else {
        std::printf("      recommended delta: -- (record a foot-lift point with k)\n");
    }
}

void sampleAlternateAttitude() {
    if (!g_alternate_stats.active) {
        return;
    }

    float relative_roll_deg = 0.0f;
    if (currentRelativeRoll(relative_roll_deg)) {
        if (!g_alternate_stats.roll_valid) {
            g_alternate_stats.roll_min_deg = relative_roll_deg;
            g_alternate_stats.roll_max_deg = relative_roll_deg;
            g_alternate_stats.roll_valid = true;
        } else {
            g_alternate_stats.roll_min_deg =
                std::fmin(g_alternate_stats.roll_min_deg, relative_roll_deg);
            g_alternate_stats.roll_max_deg =
                std::fmax(g_alternate_stats.roll_max_deg, relative_roll_deg);
        }
        if (g_rock_cycle_started) {
            if (!g_rock_cycle_roll_valid) {
                g_rock_cycle_roll_valid = true;
                g_rock_cycle_roll_min_deg = relative_roll_deg;
                g_rock_cycle_roll_max_deg = relative_roll_deg;
            } else {
                g_rock_cycle_roll_min_deg =
                    std::fmin(g_rock_cycle_roll_min_deg, relative_roll_deg);
                g_rock_cycle_roll_max_deg =
                    std::fmax(g_rock_cycle_roll_max_deg, relative_roll_deg);
            }
        }
    }

    float relative_pitch_deg = 0.0f;
    if (currentRelativePitch(relative_pitch_deg)) {
        if (!g_alternate_stats.pitch_valid) {
            g_alternate_stats.pitch_min_deg = relative_pitch_deg;
            g_alternate_stats.pitch_max_deg = relative_pitch_deg;
            g_alternate_stats.pitch_valid = true;
        } else {
            g_alternate_stats.pitch_min_deg =
                std::fmin(g_alternate_stats.pitch_min_deg, relative_pitch_deg);
            g_alternate_stats.pitch_max_deg =
                std::fmax(g_alternate_stats.pitch_max_deg, relative_pitch_deg);
        }
    }
}

void printAlternateSummary() {
    if (!g_alternate_stats.active) {
        return;
    }
    sampleAlternateAttitude();
    if (g_rock_cycle_roll_valid) {
        const float final_mid =
            (g_rock_cycle_roll_min_deg + g_rock_cycle_roll_max_deg) * 0.5f;
        if (!g_rock_first_mid_valid) {
            g_rock_first_mid_valid = true;
            g_rock_first_mid_deg = final_mid;
        }
        g_rock_last_mid_deg = final_mid;
    }

    std::printf("\n── 교대 rocking 결과 ──\n"
                "  진폭 %.1fmm, 주기 %lums, %lu 사이클, %s 방식\n"
                "  보상: pitch=%s(Kp%.1f)  roll중심=%s(Kr%.1f)  "
                "진폭추종=%s\n\n",
                g_rock_alternate_amplitude_mm,
                static_cast<unsigned long>(g_rock_alternate_period_ms),
                static_cast<unsigned long>(g_alternate_stats.cycles),
                g_rock_method == RockMethod::FLAT ? "FLAT" : "IK",
                g_pitch_comp_enabled ? "ON" : "OFF", g_comp_kp_pitch,
                g_rock_center_comp_enabled ? "ON" : "OFF",
                tilt_pose_test::kCompKrCenterDefault,
                g_rock_amp_comp_enabled ? "ON" : "OFF");

    float roll_peak_to_peak = 0.0f;
    if (g_alternate_stats.roll_valid) {
        roll_peak_to_peak = g_alternate_stats.roll_max_deg -
                            g_alternate_stats.roll_min_deg;
        std::printf("  roll  진동폭: %+.1f ~ %+.1f deg  (peak-to-peak %.1f)\n",
                    g_alternate_stats.roll_min_deg,
                    g_alternate_stats.roll_max_deg, roll_peak_to_peak);
    } else {
        std::printf("  roll  진동폭: --\n");
    }

    float pitch_peak_to_peak = 0.0f;
    if (g_alternate_stats.pitch_valid) {
        pitch_peak_to_peak = g_alternate_stats.pitch_max_deg -
                             g_alternate_stats.pitch_min_deg;
        std::printf("  pitch 진동폭: %+.1f ~ %+.1f deg  (peak-to-peak %.1f)\n",
                    g_alternate_stats.pitch_min_deg,
                    g_alternate_stats.pitch_max_deg, pitch_peak_to_peak);
    } else {
        std::printf("  pitch 진동폭: --\n");
    }

    float pitch_residual_deg = 0.0f;
    const bool pitch_residual_valid = currentRelativePitch(pitch_residual_deg);
    if (pitch_residual_valid) {
        std::printf("  종료 시 pitch 잔차: %+.1f deg\n", pitch_residual_deg);
    } else {
        std::printf("  종료 시 pitch 잔차: --\n");
    }
    if (g_rock_first_mid_valid) {
        std::printf("  roll 중심 이동: %+.1f → %+.1f deg\n",
                    g_rock_first_mid_deg, g_rock_last_mid_deg);
    } else {
        std::printf("  roll 중심 이동: --\n");
    }
    std::printf("  최종 delta_bias: %+.1fmm, delta_scale: %.2f\n"
                "  body_x 사용 범위: %+.1f ~ %+.1f mm (한계 ±%.1f 대비)\n",
                g_rock_delta_bias_mm, g_rock_delta_scale,
                g_comp_body_x_min_mm, g_comp_body_x_max_mm,
                tilt_pose_test::kCompBodyXMaxMm);
    if (g_comp_body_x_min_mm <= -0.95f * tilt_pose_test::kCompBodyXMaxMm ||
        g_comp_body_x_max_mm >= +0.95f * tilt_pose_test::kCompBodyXMaxMm) {
        std::printf("주의: body_x 보상이 포화되었습니다. 발판 앞뒤 연장이 필요합니다.\n");
    }

    if (g_alternate_stats.roll_valid && g_alternate_stats.pitch_valid &&
        pitch_peak_to_peak > roll_peak_to_peak) {
        std::printf("주의: 좌우 rocking 중 pitch 진동폭이 roll 보다 큽니다.\n");
    }
    if (pitch_residual_valid && std::fabs(pitch_residual_deg) >= 3.0f) {
        std::printf("주의: pitch 잔차 %+.1fdeg — 로봇이 뒤로 밀린 상태일 수 있습니다.\n"
                    "      손으로 바로 세운 뒤 r 로 기준을 다시 잡으세요.\n",
                    pitch_residual_deg);
    }
    g_alternate_stats.active = false;
    g_alternate_stats.move_pending = false;
}

bool calculateAlternateMoveDuration(std::uint32_t& duration_ms) {
    float positive_target[tilt::NUM_JOINTS]{};
    float negative_target[tilt::NUM_JOINTS]{};
    const float positive_applied =
        +g_rock_alternate_amplitude_mm * g_rock_delta_scale +
        g_rock_delta_bias_mm;
    const float negative_applied =
        -g_rock_alternate_amplitude_mm * g_rock_delta_scale +
        g_rock_delta_bias_mm;
    if (std::fabs(positive_applied) > tilt_pose_test::kRockDeltaMaxMm ||
        std::fabs(negative_applied) > tilt_pose_test::kRockDeltaMaxMm ||
        !buildRockTarget(positive_applied,
                         g_rock_base_height_mm, positive_target) ||
        !buildRockTarget(negative_applied,
                         g_rock_base_height_mm, negative_target)) {
        return false;
    }
    duration_ms = requiredDurationMs(positive_target, negative_target,
                                     tilt_pose_test::kMinMoveDurationMs);
    return true;
}

void ensureAlternatePeriod(std::uint32_t required_duration_ms) {
    if (g_rock_alternate_period_ms >= required_duration_ms) {
        return;
    }
    std::printf("주기가 짧아 이동을 따라가지 못합니다 (필요 %lums).\n",
                static_cast<unsigned long>(required_duration_ms));
    g_rock_alternate_period_ms = required_duration_ms;
    std::printf("교대 rocking 주기를 %lums로 자동 확대했습니다.\n",
                static_cast<unsigned long>(g_rock_alternate_period_ms));
}

void returnRockDeltaToZero() {
    if (g_rock_mode.load() && g_state.load() == SafetyState::ARMED) {
        applyRockTarget(0.0f, g_rock_base_height_mm,
                        tilt_pose_test::kRockStepDurationMs);
    }
}

void stopRockAutomation(const char* reason, bool return_to_zero) {
    const RockAutomation stopped =
        g_rock_automation.exchange(RockAutomation::NONE);
    if (stopped == RockAutomation::NONE) {
        return;
    }
    g_interpolator.abort();
    if (stopped == RockAutomation::ALTERNATE) {
        printAlternateSummary();
    }
    std::printf("Rock automation stopped: %s.\n", reason);
    if (return_to_zero) {
        returnRockDeltaToZero();
    }
}

void resetCompensationOutputs() {
    g_comp_body_x_mm = 0.0f;
    g_comp_body_x_min_mm = 0.0f;
    g_comp_body_x_max_mm = 0.0f;
    g_rock_delta_bias_mm = 0.0f;
    g_rock_delta_scale = 1.0f;
}

void disableAllCompensation(const char* reason, bool stop_motion) {
    const bool was_enabled = g_pitch_comp_enabled ||
                             g_rock_center_comp_enabled ||
                             g_rock_amp_comp_enabled;
    g_pitch_comp_enabled = false;
    g_rock_center_comp_enabled = false;
    g_rock_amp_comp_enabled = false;
    g_comp_body_x_mm = 0.0f;
    if (g_step_test_phase != StepTestPhase::IDLE) {
        g_step_test_phase = StepTestPhase::IDLE;
        std::printf("Step response test aborted.\n");
    }
    if (was_enabled || reason != nullptr) {
        std::printf("Compensation disabled%s%s.\n",
                    reason != nullptr ? ": " : "",
                    reason != nullptr ? reason : "");
    }

    if (stop_motion && g_walk_running.load()) {
        stopWalkMotion(reason != nullptr ? reason : "compensation safety", true);
    } else if (stop_motion &&
        g_rock_automation.load() == RockAutomation::ALTERNATE) {
        stopRockAutomation(reason != nullptr ? reason : "compensation safety", true);
    } else if (g_stand_ready.load() && g_state.load() == SafetyState::ARMED) {
        g_interpolator.abort();
        applyStandTarget(0.0f, g_stand_base_height_mm);
    } else if (g_rock_ready.load() &&
               g_rock_automation.load() == RockAutomation::NONE &&
               g_state.load() == SafetyState::ARMED) {
        g_interpolator.abort();
        applyRockTarget(g_rock_delta_mm, g_rock_base_height_mm,
                        tilt_pose_test::kMinMoveDurationMs, false);
    }
    resetCompensationOutputs();
}

bool compensationAttitudeReady() {
    if (!g_imu_available || !g_roll_valid || !g_pitch_valid ||
        !g_roll_zero_valid || !g_pitch_zero_valid ||
        g_imu_consecutive_failures >= 5) {
        std::printf("Compensation unavailable: valid roll/pitch and zero reference "
                    "are required.\n");
        return false;
    }
    return true;
}

void togglePitchCompensation() {
    if (!g_pitch_comp_enabled && !compensationAttitudeReady()) {
        return;
    }
    g_pitch_comp_enabled = !g_pitch_comp_enabled;
    g_comp_body_x_mm = 0.0f;
    std::printf("Pitch compensation %s (Kp=%.1f, lpf=%.2f).\n",
                g_pitch_comp_enabled ? "ON" : "OFF",
                g_comp_kp_pitch, g_comp_lpf_alpha);
    if (!g_pitch_comp_enabled && g_stand_ready.load()) {
        g_interpolator.abort();
        applyStandTarget(0.0f, g_stand_base_height_mm);
    }
}

void adjustPitchGain(float change) {
    g_comp_kp_pitch = std::fmax(
        0.0f, std::fmin(tilt_pose_test::kCompKpPitchMax,
                        g_comp_kp_pitch + change));
    std::printf("Pitch compensation Kp=%.1f mm/deg.\n", g_comp_kp_pitch);
}

void adjustCompLpf(float change) {
    g_comp_lpf_alpha = std::fmax(
        0.05f, std::fmin(1.0f, g_comp_lpf_alpha + change));
    std::printf("Pitch compensation LPF alpha=%.2f.\n", g_comp_lpf_alpha);
}

void toggleRockCenterCompensation() {
    if (!g_rock_center_comp_enabled && !compensationAttitudeReady()) {
        return;
    }
    g_rock_center_comp_enabled = !g_rock_center_comp_enabled;
    if (!g_rock_center_comp_enabled) {
        g_rock_delta_bias_mm = 0.0f;
    }
    std::printf("Rock roll-centering compensation %s (Kr=%.2f).\n",
                g_rock_center_comp_enabled ? "ON" : "OFF",
                tilt_pose_test::kCompKrCenterDefault);
}

void toggleRockAmplitudeCompensation() {
    if (!g_rock_amp_comp_enabled && !compensationAttitudeReady()) {
        return;
    }
    g_rock_amp_comp_enabled = !g_rock_amp_comp_enabled;
    if (!g_rock_amp_comp_enabled) {
        g_rock_delta_scale = 1.0f;
    }
    std::printf("Rock roll-amplitude compensation %s (Ka=%.2f).\n",
                g_rock_amp_comp_enabled ? "ON" : "OFF",
                tilt_pose_test::kCompKaAmpDefault);
}

void updatePitchCompensation() {
    if (!g_pitch_comp_enabled) {
        g_comp_body_x_mm = 0.0f;
        return;
    }
    float relative_pitch_deg = 0.0f;
    if (!currentRelativePitch(relative_pitch_deg)) {
        return;
    }
    const float raw_command_mm = std::fmax(
        -tilt_pose_test::kCompBodyXMaxMm,
        std::fmin(tilt_pose_test::kCompBodyXMaxMm,
                  g_comp_kp_pitch * relative_pitch_deg));
    g_comp_body_x_mm +=
        g_comp_lpf_alpha * (raw_command_mm - g_comp_body_x_mm);
    g_comp_body_x_min_mm = std::fmin(g_comp_body_x_min_mm, g_comp_body_x_mm);
    g_comp_body_x_max_mm = std::fmax(g_comp_body_x_max_mm, g_comp_body_x_mm);
}

void resetRockCyclePeaks() {
    g_rock_cycle_roll_valid = false;
    float relative_roll_deg = 0.0f;
    if (currentRelativeRoll(relative_roll_deg)) {
        g_rock_cycle_roll_valid = true;
        g_rock_cycle_roll_min_deg = relative_roll_deg;
        g_rock_cycle_roll_max_deg = relative_roll_deg;
    }
}

void finishRockCycle(float next_commanded_delta_mm) {
    if (!g_rock_cycle_roll_valid) {
        std::printf("cyc#%lu roll unavailable; compensation unchanged.\n",
                    static_cast<unsigned long>(g_alternate_stats.cycles));
        resetRockCyclePeaks();
        return;
    }

    const float roll_mid_deg =
        (g_rock_cycle_roll_min_deg + g_rock_cycle_roll_max_deg) * 0.5f;
    const float roll_amp_deg =
        (g_rock_cycle_roll_max_deg - g_rock_cycle_roll_min_deg) * 0.5f;
    if (!g_rock_first_mid_valid) {
        g_rock_first_mid_valid = true;
        g_rock_first_mid_deg = roll_mid_deg;
    }
    g_rock_last_mid_deg = roll_mid_deg;

    float candidate_bias_mm = g_rock_delta_bias_mm;
    float candidate_scale = g_rock_delta_scale;
    if (g_rock_center_comp_enabled) {
        candidate_bias_mm +=
            -tilt_pose_test::kCompKrCenterDefault * roll_mid_deg;
        candidate_bias_mm = std::fmax(
            -tilt_pose_test::kCompDeltaBiasMaxMm,
            std::fmin(tilt_pose_test::kCompDeltaBiasMaxMm,
                      candidate_bias_mm));
    }
    if (g_rock_amp_comp_enabled) {
        candidate_scale += tilt_pose_test::kCompKaAmpDefault *
                           (tilt_pose_test::kCompRollTargetAmpDeg -
                            roll_amp_deg);
        candidate_scale = std::fmax(
            1.0f, std::fmin(tilt_pose_test::kCompDeltaScaleMax,
                            candidate_scale));
    }

    const float candidate_applied_delta_mm =
        next_commanded_delta_mm * candidate_scale + candidate_bias_mm;
    float candidate_target[tilt::NUM_JOINTS]{};
    const bool candidate_valid =
        std::fabs(candidate_applied_delta_mm) <=
            tilt_pose_test::kRockDeltaMaxMm &&
        buildRockTargetForMethod(
            g_rock_method, candidate_applied_delta_mm,
            g_rock_base_height_mm,
            g_pitch_comp_enabled ? g_comp_body_x_mm : 0.0f,
            candidate_target) &&
        targetWithinLogicalLimits(candidate_target);
    if (candidate_valid) {
        g_rock_delta_bias_mm = candidate_bias_mm;
        g_rock_delta_scale = candidate_scale;
    } else {
        std::printf("Rock compensation update cancelled: next target exceeds "
                    "workspace or joint limits.\n");
    }

    float relative_pitch_deg = 0.0f;
    const bool pitch_valid = currentRelativePitch(relative_pitch_deg);
    std::printf("cyc#%lu delta=%+.1f(bias%+.1f scale%.2f→applied%+.1f) "
                "roll=%+.1f~%+.1f(mid%+.1f amp%.1f) pitch=",
                static_cast<unsigned long>(g_alternate_stats.cycles),
                next_commanded_delta_mm, g_rock_delta_bias_mm,
                g_rock_delta_scale,
                next_commanded_delta_mm * g_rock_delta_scale +
                    g_rock_delta_bias_mm,
                g_rock_cycle_roll_min_deg, g_rock_cycle_roll_max_deg,
                roll_mid_deg, roll_amp_deg);
    if (pitch_valid) {
        std::printf("%+.1f", relative_pitch_deg);
    } else {
        std::printf("--");
    }
    std::printf(" body_x=%+.1f\n", g_comp_body_x_mm);
    resetRockCyclePeaks();
}

void printStandState() {
    float relative_pitch_deg = 0.0f;
    float relative_roll_deg = 0.0f;
    const bool pitch_relative_valid = currentRelativePitch(relative_pitch_deg);
    const bool roll_relative_valid = currentRelativeRoll(relative_roll_deg);
    std::printf("[STAND comp=%s Kp=%.1f lpf=%.2f] pitch=",
                g_pitch_comp_enabled ? "ON" : "OFF",
                g_comp_kp_pitch, g_comp_lpf_alpha);
    if (g_pitch_valid) {
        std::printf("%+.1f(rel ", g_pitch_deg);
        if (pitch_relative_valid) std::printf("%+.1f)", relative_pitch_deg);
        else std::printf("--)");
    } else {
        std::printf("--(rel --)");
    }
    const float displayed_body_x =
        g_step_test_phase == StepTestPhase::DISTURBANCE
            ? tilt_pose_test::kStepTestBodyXMm
            : g_comp_body_x_mm;
    std::printf(" body_x=%+.1fmm roll=", displayed_body_x);
    if (g_roll_valid) {
        std::printf("%+.1f(rel ", g_roll_deg);
        if (roll_relative_valid) std::printf("%+.1f)\n", relative_roll_deg);
        else std::printf("--)\n");
    } else {
        std::printf("--(rel --)\n");
    }
}

void finishStepResponseTest() {
    float residual_pitch_deg = 0.0f;
    const bool residual_valid = currentRelativePitch(residual_pitch_deg);
    std::printf("\n── 스텝 응답 (Kp=%.1f, lpf=%.2f) ──\n"
                "  외란 후 최대 pitch:   %+.1f deg\n",
                g_comp_kp_pitch, g_comp_lpf_alpha,
                g_step_test_peak_pitch_deg);
    if (g_step_test_settled) {
        std::printf("  |pitch| < %.1fdeg 도달: %lu ms\n",
                    tilt_pose_test::kStepTestSettleDeg,
                    static_cast<unsigned long>(g_step_test_settle_ms));
    } else {
        std::printf("  |pitch| < %.1fdeg 도달: 못 함\n",
                    tilt_pose_test::kStepTestSettleDeg);
    }
    if (residual_valid) {
        std::printf("  2초 시점 잔차:        %+.1f deg\n", residual_pitch_deg);
    } else {
        std::printf("  2초 시점 잔차:        --\n");
    }
    std::printf("  진동 횟수(부호 반전): %lu회\n",
                static_cast<unsigned long>(g_step_test_sign_reversals));
    if (g_step_test_sign_reversals >= 3) {
        std::printf("  안내: 진동이 많습니다. Kp를 낮춰 보세요.\n");
    } else if (!g_step_test_settled) {
        std::printf("  안내: Kp가 작거나 발판 지지 길이가 부족할 수 있습니다.\n");
    }
    g_step_test_phase = StepTestPhase::IDLE;
}

void sampleStepResponse(std::uint32_t now) {
    if (g_step_test_phase != StepTestPhase::RECORDING) {
        return;
    }
    float pitch_deg = 0.0f;
    if (!currentRelativePitch(pitch_deg)) {
        return;
    }
    if (std::fabs(pitch_deg) > std::fabs(g_step_test_peak_pitch_deg)) {
        g_step_test_peak_pitch_deg = pitch_deg;
    }
    if (std::fabs(pitch_deg) >= tilt_pose_test::kStepTestSettleDeg) {
        g_step_test_was_unsettled = true;
    } else if (g_step_test_was_unsettled && !g_step_test_settled) {
        g_step_test_settled = true;
        g_step_test_settle_ms = now - g_step_test_phase_started_ms;
    }

    const int sign = pitch_deg > 0.1f ? 1 : pitch_deg < -0.1f ? -1 : 0;
    if (sign != 0) {
        if (g_step_test_last_sign != 0 && sign != g_step_test_last_sign) {
            ++g_step_test_sign_reversals;
        }
        g_step_test_last_sign = sign;
    }
    g_step_test_last_pitch_deg = pitch_deg;
}

void startStepResponseTest() {
    if (!g_stand_ready.load() || g_step_test_phase != StepTestPhase::IDLE) {
        std::printf("Step response requires idle stand mode.\n");
        return;
    }
    if (!compensationAttitudeReady()) {
        return;
    }
    g_pitch_comp_enabled = false;
    g_comp_body_x_mm = 0.0f;
    g_step_test_phase = StepTestPhase::DISTURBANCE;
    g_step_test_disturbance_arrived = false;
    g_step_test_phase_started_ms = nowMs();
    g_interpolator.abort();
    if (!applyStandTarget(tilt_pose_test::kStepTestBodyXMm,
                          g_stand_base_height_mm)) {
        g_step_test_phase = StepTestPhase::IDLE;
        return;
    }
    std::printf("Step response: applying body_x=%+.1fmm for %lums.\n",
                tilt_pose_test::kStepTestBodyXMm,
                static_cast<unsigned long>(tilt_pose_test::kStepTestHoldMs));
}

void serviceStepResponse(std::uint32_t now) {
    if (g_step_test_phase == StepTestPhase::DISTURBANCE &&
        !g_step_test_disturbance_arrived) {
        if (g_interpolator.isBusy()) {
            return;
        }
        g_step_test_disturbance_arrived = true;
        g_step_test_phase_started_ms = now;
        std::printf("Step response: disturbance target reached; holding for %lums.\n",
                    static_cast<unsigned long>(tilt_pose_test::kStepTestHoldMs));
        return;
    }
    if (g_step_test_phase == StepTestPhase::DISTURBANCE &&
        g_step_test_disturbance_arrived &&
        now - g_step_test_phase_started_ms >= tilt_pose_test::kStepTestHoldMs) {
        g_step_test_phase = StepTestPhase::RECORDING;
        g_step_test_phase_started_ms = now;
        g_step_test_was_unsettled = false;
        g_step_test_settled = false;
        g_step_test_settle_ms = 0;
        g_step_test_peak_pitch_deg = 0.0f;
        g_step_test_last_sign = 0;
        g_step_test_sign_reversals = 0;
        g_pitch_comp_enabled = true;
        g_comp_body_x_mm = 0.0f;
        g_interpolator.abort();
        applyStandTarget(0.0f, g_stand_base_height_mm);
        std::printf("Step response: disturbance released, recording for %lums.\n",
                    static_cast<unsigned long>(tilt_pose_test::kStepTestRecordMs));
    }
    sampleStepResponse(now);
    if (g_step_test_phase == StepTestPhase::RECORDING &&
        now - g_step_test_phase_started_ms >=
            tilt_pose_test::kStepTestRecordMs) {
        finishStepResponseTest();
    }
}

void serviceCompensation() {
    if (g_imu_comp_fault_requested) {
        g_imu_comp_fault_requested = false;
        disableAllCompensation("five consecutive IMU read failures", false);
    }

    const bool compensation_active = g_pitch_comp_enabled ||
                                     g_rock_center_comp_enabled ||
                                     g_rock_amp_comp_enabled;
    float relative_pitch_deg = 0.0f;
    float relative_roll_deg = 0.0f;
    const bool pitch_valid = currentRelativePitch(relative_pitch_deg);
    const bool roll_valid = currentRelativeRoll(relative_roll_deg);
    if ((compensation_active || g_walk_running.load()) &&
        ((pitch_valid && std::fabs(relative_pitch_deg) >
                             tilt_pose_test::kCompAbortPitchDeg) ||
         (roll_valid && std::fabs(relative_roll_deg) >
                            tilt_pose_test::kCompAbortRollDeg))) {
        disableAllCompensation("attitude exceeded compensation safety limit", true);
        return;
    }

    updatePitchCompensation();
    const std::uint32_t now = nowMs();
    if (g_stand_ready.load()) {
        serviceStepResponse(now);
        const float desired_body_x =
            g_step_test_phase == StepTestPhase::DISTURBANCE
                ? tilt_pose_test::kStepTestBodyXMm
                : (g_pitch_comp_enabled ? g_comp_body_x_mm : 0.0f);
        if (!g_interpolator.isBusy() &&
            (std::fabs(desired_body_x - g_stand_last_sent_body_x_mm) > 0.05f ||
             std::fabs(g_stand_base_height_mm -
                       g_stand_last_sent_height_mm) > 0.01f)) {
            if (!applyStandTarget(desired_body_x, g_stand_base_height_mm)) {
                g_pitch_comp_enabled = false;
                g_comp_body_x_mm = 0.0f;
                std::printf("Stand compensation disabled: target rejected.\n");
            }
        }
        if (now - g_last_status_print_ms >=
            tilt_pose_test::kStatusPrintPeriodMs) {
            g_last_status_print_ms = now;
            printStandState();
        }
    } else if (g_rock_ready.load() && g_pitch_comp_enabled &&
               g_rock_automation.load() == RockAutomation::NONE &&
               !g_interpolator.isBusy() &&
               std::fabs(g_comp_body_x_mm -
                         g_rock_last_applied_body_x_mm) > 0.05f) {
        if (!applyRockTarget(g_rock_delta_mm, g_rock_base_height_mm,
                             tilt_pose_test::kMinMoveDurationMs, false)) {
            g_pitch_comp_enabled = false;
            g_comp_body_x_mm = 0.0f;
            std::printf("Rock pitch compensation disabled: target rejected.\n");
        }
    }
}

void adjustStandHeight(float change_mm) {
    if (!g_stand_ready.load() || g_step_test_phase != StepTestPhase::IDLE) {
        return;
    }
    const float candidate = std::fmax(
        tilt_pose_test::kBodyHeightMinMm,
        std::fmin(tilt_pose_test::kBodyHeightMaxMm,
                  g_stand_base_height_mm + change_mm));
    if (candidate == g_stand_base_height_mm) {
        std::printf("Stand height limit reached: %.2fmm.\n", candidate);
        return;
    }
    g_stand_base_height_mm = candidate;
    std::printf("Stand base height=%.2fmm.\n", g_stand_base_height_mm);
}

void zeroStandCompensation() {
    if (!g_stand_ready.load()) {
        return;
    }
    g_step_test_phase = StepTestPhase::IDLE;
    g_pitch_comp_enabled = false;
    g_comp_body_x_mm = 0.0f;
    g_interpolator.abort();
    applyStandTarget(0.0f, g_stand_base_height_mm);
    std::printf("Stand compensation OFF; body_x returning to zero.\n");
}

void quitStandMode() {
    if (!g_stand_mode.exchange(false)) {
        return;
    }
    g_stand_ready.store(false);
    g_step_test_phase = StepTestPhase::IDLE;
    g_pitch_comp_enabled = false;
    g_comp_body_x_mm = 0.0f;
    g_interpolator.abort();
    beginMove(tilt::ZERO_POSE_RAD, tilt_pose_test::kPoseDurationMs);
    std::printf("Leaving stand mode and returning home.\n");
}

void requestRockMode() {
    if (!isArmed("rock")) {
        return;
    }
    std::printf("\nROCK SAFETY: place the robot on the floor and be ready to catch it.\n"
                "Is the robot on the floor and supported by your hands? (y/n): ");
    std::fflush(stdout);
    g_rock_confirmation_pending.store(true);
}

void confirmRockMode() {
    if (!isArmed("rock")) {
        g_rock_confirmation_pending.store(false);
        return;
    }
    g_rock_confirmation_pending.store(false);
    g_rock_mode.store(true);
    g_rock_ready.store(false);
    g_rock_automation.store(RockAutomation::NONE);
    g_rock_delta_mm = 0.0f;
    g_rock_base_height_mm = tilt::ZERO_POSE_HEIGHT_MM;
    g_rock_max_abs_delta_mm = 0.0f;
    g_rock_coarse = false;
    g_rock_method = RockMethod::FLAT;
    g_pitch_comp_enabled = false;
    g_rock_center_comp_enabled = false;
    g_rock_amp_comp_enabled = false;
    resetCompensationOutputs();
    g_rock_applied_delta_mm = 0.0f;
    std::memcpy(g_rock_target_rad, tilt::ZERO_POSE_RAD,
                sizeof(g_rock_target_rad));
    g_rock_target_valid = true;
    g_roll_zero_valid = false;
    g_pitch_zero_valid = false;
    g_warn_rock_zero_on_arrival = false;
    g_rock_alternate_period_ms = tilt_pose_test::kRockAlternatePeriodMs;
    g_left_lift = {};
    g_right_lift = {};
    g_alternate_stats = {};
    if (beginMove(tilt::ZERO_POSE_RAD, tilt_pose_test::kPoseDurationMs,
                  false, true)) {
        std::printf("Returning home before rocking mode starts.\n");
    } else {
        g_rock_mode.store(false);
    }
}

void handleRockMove(float delta_change_mm, float height_change_mm) {
    if (!g_rock_ready.load()) {
        std::printf("Rock mode is not ready yet.\n");
        return;
    }
    if (g_rock_automation.load() != RockAutomation::NONE) {
        std::printf("Stop the active rock automation with any key first.\n");
        return;
    }
    applyRockTarget(g_rock_delta_mm + delta_change_mm,
                    g_rock_base_height_mm + height_change_mm,
                    tilt_pose_test::kRockStepDurationMs);
}

void recordFootLift() {
    if (!g_rock_ready.load() || g_interpolator.isBusy()) {
        std::printf("Wait for the current rocking move to finish before recording.\n");
        return;
    }
    if (g_rock_delta_mm == 0.0f) {
        std::printf("Cannot record a foot-lift point at delta=0.\n");
        return;
    }
    FootLiftRecord& record = g_rock_delta_mm > 0.0f ? g_left_lift : g_right_lift;
    record.valid = true;
    record.delta_mm = g_rock_delta_mm;
    record.roll_valid = g_roll_valid;
    record.roll_deg = g_roll_deg;
    std::printf("Recorded %s foot lift at delta=%+.1fmm",
                g_rock_delta_mm > 0.0f ? "left" : "right", g_rock_delta_mm);
    if (g_roll_valid) {
        std::printf(", roll=%+.1fdeg.\n", g_roll_deg);
    } else {
        std::printf(", roll=--.\n");
    }
}

void startRockSweep() {
    if (!g_rock_ready.load() || g_interpolator.isBusy()) {
        std::printf("Rock sweep requires an idle, ready rocking mode.\n");
        return;
    }
    g_rock_sweep_max_delta_mm = 0.0f;
    g_rock_sweep_roll_valid = false;
    g_rock_sweep_roll_deg = 0.0f;
    g_rock_sweep_phase = RockSweepPhase::MOVING;
    g_rock_automation.store(RockAutomation::SWEEP);
    std::printf("Automatic rock sweep started. Press any key to stop and return delta to 0.\n");
    if (!applyRockTarget(tilt_pose_test::kRockSweepStartMm,
                         g_rock_base_height_mm,
                         tilt_pose_test::kRockStepDurationMs, false)) {
        stopRockAutomation("start target rejected", true);
    }
}

void startRockAlternate() {
    if (!g_rock_ready.load() || g_interpolator.isBusy()) {
        std::printf("Alternating rock requires an idle, ready rocking mode.\n");
        return;
    }
    const float amplitude = std::fabs(g_rock_delta_mm);
    if (amplitude <= 0.0f) {
        std::printf("Set a non-zero delta first, then press 2.\n");
        return;
    }
    g_rock_alternate_amplitude_mm = amplitude;
    if (!calculateAlternateMoveDuration(g_rock_alternate_move_duration_ms)) {
        std::printf("Alternating rock rejected: could not build both targets.\n");
        return;
    }
    ensureAlternatePeriod(g_rock_alternate_move_duration_ms);
    g_alternate_stats = {};
    g_alternate_stats.active = true;
    g_rock_cycle_started = false;
    g_rock_cycle_roll_valid = false;
    g_rock_first_mid_valid = false;
    g_comp_body_x_min_mm = g_comp_body_x_mm;
    g_comp_body_x_max_mm = g_comp_body_x_mm;
    sampleAlternateAttitude();
    g_rock_next_action_ms = nowMs() + g_rock_alternate_period_ms;
    g_rock_automation.store(RockAutomation::ALTERNATE);
    std::printf("Alternating rock started: d=%.1fmm, period=%lums. "
                "Use [/] to adjust; any other key stops.\n",
                amplitude, static_cast<unsigned long>(g_rock_alternate_period_ms));
}

void adjustRockAlternatePeriod(int change_ms) {
    if (g_rock_automation.load() != RockAutomation::ALTERNATE) {
        return;
    }
    const int requested = static_cast<int>(g_rock_alternate_period_ms) + change_ms;
    const int bounded = requested < static_cast<int>(tilt_pose_test::kRockAlternatePeriodMinMs)
                            ? static_cast<int>(tilt_pose_test::kRockAlternatePeriodMinMs)
                            : requested > static_cast<int>(tilt_pose_test::kRockAlternatePeriodMaxMs)
                                  ? static_cast<int>(tilt_pose_test::kRockAlternatePeriodMaxMs)
                                  : requested;
    g_rock_alternate_period_ms = static_cast<std::uint32_t>(bounded);
    ensureAlternatePeriod(g_rock_alternate_move_duration_ms);
    g_rock_next_action_ms = nowMs() + g_rock_alternate_period_ms;
    std::printf("Alternating period=%lums.\n",
                static_cast<unsigned long>(g_rock_alternate_period_ms));
}

void quitRockMode() {
    if (!g_rock_mode.load()) {
        return;
    }
    const RockAutomation stopped =
        g_rock_automation.exchange(RockAutomation::NONE);
    g_interpolator.abort();
    if (stopped == RockAutomation::ALTERNATE) {
        printAlternateSummary();
    }
    printRockSummary();
    g_rock_delta_mm = 0.0f;
    g_rock_base_height_mm = tilt::ZERO_POSE_HEIGHT_MM;
    g_rock_ready.store(false);
    g_rock_mode.store(false);
    g_pitch_comp_enabled = false;
    g_rock_center_comp_enabled = false;
    g_rock_amp_comp_enabled = false;
    g_roll_zero_valid = false;
    g_pitch_zero_valid = false;
    g_rock_target_valid = false;
    g_warn_rock_zero_on_arrival = false;
    beginMove(tilt::ZERO_POSE_RAD, tilt_pose_test::kPoseDurationMs);
    std::printf("Leaving rocking mode and returning home.\n");
}

void setWalkLegPhase(WalkLegState& leg, std::uint8_t phase) {
    leg.phase = phase % 4;
    switch (leg.phase) {
        case 0: leg.short_leg = true; break;
        case 1: leg.x_mm = +g_walk_legx_mm; break;
        case 2: leg.short_leg = false; break;
        case 3: leg.x_mm = -g_walk_legx_mm; break;
    }
}

bool walkLegWithinLimits(tilt::Leg leg, float theta[3]) {
    const float original[3] = {theta[0], theta[1], theta[2]};
    if (tilt::clamp_to_limits(leg, theta)) {
        return true;
    }
    const int offset = leg == tilt::Leg::LEFT ? 0 : 3;
    for (int joint = 0; joint < 3; ++joint) {
        if (theta[joint] != original[joint]) {
            std::printf("Walk target rejected: %s reached its joint limit.\n",
                        tilt::JOINT_NAME[joint + offset]);
        }
    }
    return false;
}

bool buildWalkTarget(float delta_bias_mm,
                     float target[tilt::NUM_JOINTS]) {
    const float signed_delta_mm =
        g_walk_left.short_leg ? +g_walk_delta_mm : -g_walk_delta_mm;
    const float applied_delta_mm = signed_delta_mm + delta_bias_mm;
    if (std::fabs(applied_delta_mm) > tilt_pose_test::kRockDeltaMaxMm) {
        std::printf("Walk target rejected: applied delta %+.1fmm exceeds limit "
                    "±%.1fmm.\n",
                    applied_delta_mm, tilt_pose_test::kRockDeltaMaxMm);
        return false;
    }

    const float left_height = g_walk_base_height_mm - applied_delta_mm * 0.5f;
    const float right_height = g_walk_base_height_mm + applied_delta_mm * 0.5f;
    const float common_foot_x =
        g_pitch_comp_enabled ? -g_comp_body_x_mm : 0.0f;
    const float left_x = g_walk_left.x_mm + common_foot_x;
    const float right_x = g_walk_right.x_mm + common_foot_x;
    const tilt::Vec3 left_target{left_x, +tilt::Y_HIP_MM, -left_height};
    const tilt::Vec3 right_target{right_x, -tilt::Y_HIP_MM, -right_height};
    tilt::IkResult left = tilt::ik_foot(tilt::Leg::LEFT, left_target, 0.0f);
    tilt::IkResult right = tilt::ik_foot(tilt::Leg::RIGHT, right_target, 0.0f);
    if (!left.reachable || !right.reachable) {
        std::printf("Walk target rejected: unreachable (%s%s%s).\n",
                    !left.reachable ? "LEFT" : "",
                    !left.reachable && !right.reachable ? "+" : "",
                    !right.reachable ? "RIGHT" : "");
        return false;
    }
    if (!walkLegWithinLimits(tilt::Leg::LEFT, left.theta) ||
        !walkLegWithinLimits(tilt::Leg::RIGHT, right.theta)) {
        return false;
    }
    for (int joint = 0; joint < 3; ++joint) {
        target[joint] = left.theta[joint];
        target[joint + 3] = right.theta[joint];
    }
    if (!targetWithinLogicalLimits(target)) {
        return false;
    }

    g_walk_applied_delta_mm = applied_delta_mm;
    g_walk_last_left_x_mm = left_x;
    g_walk_last_right_x_mm = right_x;
    g_walk_last_left_height_mm = left_height;
    g_walk_last_right_height_mm = right_height;
    return true;
}

bool applyWalkTarget() {
    float target[tilt::NUM_JOINTS]{};
    const float bias_mm = g_rock_center_comp_enabled
                              ? g_rock_delta_bias_mm
                              : 0.0f;
    if (!buildWalkTarget(bias_mm, target) ||
        !beginMove(target, g_walk_leg_time_ms)) {
        return false;
    }
    return true;
}

void sampleWalkAttitude() {
    if (!g_walk_running.load()) {
        return;
    }
    float value_deg = 0.0f;
    if (currentRelativeRoll(value_deg)) {
        if (!g_walk_stats.roll_valid) {
            g_walk_stats.roll_valid = true;
            g_walk_stats.roll_min_deg = value_deg;
            g_walk_stats.roll_max_deg = value_deg;
        } else {
            g_walk_stats.roll_min_deg = std::fmin(g_walk_stats.roll_min_deg, value_deg);
            g_walk_stats.roll_max_deg = std::fmax(g_walk_stats.roll_max_deg, value_deg);
        }
        if (!g_walk_stats.cycle_roll_valid) {
            g_walk_stats.cycle_roll_valid = true;
            g_walk_stats.cycle_roll_min_deg = value_deg;
            g_walk_stats.cycle_roll_max_deg = value_deg;
        } else {
            g_walk_stats.cycle_roll_min_deg =
                std::fmin(g_walk_stats.cycle_roll_min_deg, value_deg);
            g_walk_stats.cycle_roll_max_deg =
                std::fmax(g_walk_stats.cycle_roll_max_deg, value_deg);
        }
    }
    if (currentRelativePitch(value_deg)) {
        if (!g_walk_stats.pitch_valid) {
            g_walk_stats.pitch_valid = true;
            g_walk_stats.pitch_min_deg = value_deg;
            g_walk_stats.pitch_max_deg = value_deg;
        } else {
            g_walk_stats.pitch_min_deg =
                std::fmin(g_walk_stats.pitch_min_deg, value_deg);
            g_walk_stats.pitch_max_deg =
                std::fmax(g_walk_stats.pitch_max_deg, value_deg);
        }
    }
}

void printWalkState() {
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    const bool roll_valid = currentRelativeRoll(roll_deg);
    const bool pitch_valid = currentRelativePitch(pitch_deg);
    std::printf("[WALK %s] legx=%+.1f delta=%.1f base=%.2f leg_t=%lums "
                "pitch_comp=%s center=%s bias=%+.1f | roll=",
                g_walk_running.load() ? "RUN" : "STOP",
                g_walk_legx_mm, g_walk_delta_mm, g_walk_base_height_mm,
                static_cast<unsigned long>(g_walk_leg_time_ms),
                g_pitch_comp_enabled ? "ON" : "OFF",
                g_rock_center_comp_enabled ? "ON" : "OFF",
                g_rock_delta_bias_mm);
    if (roll_valid) std::printf("%+.1f", roll_deg);
    else std::printf("--");
    std::printf(" pitch=");
    if (pitch_valid) std::printf("%+.1f", pitch_deg);
    else std::printf("--");
    std::printf(" body_x=%+.1f\n", g_comp_body_x_mm);
}

void printWalkCycle(bool cycle_roll_valid, float cycle_roll_min_deg,
                    float cycle_roll_max_deg) {
    float relative_pitch_deg = 0.0f;
    const bool pitch_valid = currentRelativePitch(relative_pitch_deg);
    std::printf("walk#%lu legx=%+.1f delta=%.1f base=%.2f leg_t=%lums | "
                "L(x%+.1f z%.2f) R(x%+.1f z%.2f) | roll=",
                static_cast<unsigned long>(g_walk_stats.cycles),
                g_walk_legx_mm, g_walk_delta_mm, g_walk_base_height_mm,
                static_cast<unsigned long>(g_walk_leg_time_ms),
                g_walk_last_left_x_mm, g_walk_last_left_height_mm,
                g_walk_last_right_x_mm, g_walk_last_right_height_mm);
    if (cycle_roll_valid) {
        const float mid_deg = (cycle_roll_min_deg + cycle_roll_max_deg) * 0.5f;
        const float amp_deg = (cycle_roll_max_deg - cycle_roll_min_deg) * 0.5f;
        std::printf("%+.1f(amp%.1f)", mid_deg, amp_deg);
    } else {
        std::printf("--(amp--)");
    }
    std::printf(" pitch=");
    if (pitch_valid) std::printf("%+.1f", relative_pitch_deg);
    else std::printf("--");
    std::printf(" body_x=%+.1f\n", g_comp_body_x_mm);
}

void finishWalkCycle() {
    const bool roll_valid = g_walk_stats.cycle_roll_valid;
    const float roll_min_deg = g_walk_stats.cycle_roll_min_deg;
    const float roll_max_deg = g_walk_stats.cycle_roll_max_deg;
    if (roll_valid && g_rock_center_comp_enabled) {
        const float roll_mid_deg = (roll_min_deg + roll_max_deg) * 0.5f;
        const float candidate_bias_mm = std::fmax(
            -tilt_pose_test::kCompDeltaBiasMaxMm,
            std::fmin(tilt_pose_test::kCompDeltaBiasMaxMm,
                      g_rock_delta_bias_mm -
                          tilt_pose_test::kCompKrCenterDefault * roll_mid_deg));
        float candidate_target[tilt::NUM_JOINTS]{};
        if (buildWalkTarget(candidate_bias_mm, candidate_target)) {
            g_rock_delta_bias_mm = candidate_bias_mm;
        } else {
            std::printf("Walk center compensation update cancelled: target rejected.\n");
        }
    }
    ++g_walk_stats.cycles;
    g_walk_stats.cycle_roll_valid = false;
    g_walk_last_cycle_roll_valid = roll_valid;
    g_walk_last_cycle_roll_min_deg = roll_min_deg;
    g_walk_last_cycle_roll_max_deg = roll_max_deg;
}

void printWalkSummary() {
    sampleWalkAttitude();
    std::printf("\n── walk 결과 ──\n"
                "  legx %.1fmm, delta %.1fmm, leg_time %lums, %lu 사이클\n"
                "  보상: pitch=%s(Kp%.1f) roll중심=%s\n\n",
                g_walk_summary_legx_mm, g_walk_summary_delta_mm,
                static_cast<unsigned long>(g_walk_summary_leg_time_ms),
                static_cast<unsigned long>(g_walk_stats.cycles),
                g_pitch_comp_enabled ? "ON" : "OFF", g_comp_kp_pitch,
                g_rock_center_comp_enabled ? "ON" : "OFF");
    float roll_peak_to_peak = 0.0f;
    if (g_walk_stats.roll_valid) {
        roll_peak_to_peak =
            g_walk_stats.roll_max_deg - g_walk_stats.roll_min_deg;
        std::printf("  roll  진동폭: %+.1f ~ %+.1f deg (p-p %.1f)\n",
                    g_walk_stats.roll_min_deg, g_walk_stats.roll_max_deg,
                    roll_peak_to_peak);
    } else {
        std::printf("  roll  진동폭: --\n");
    }
    if (g_walk_stats.pitch_valid) {
        std::printf("  pitch 진동폭: %+.1f ~ %+.1f deg (p-p %.1f)\n",
                    g_walk_stats.pitch_min_deg, g_walk_stats.pitch_max_deg,
                    g_walk_stats.pitch_max_deg - g_walk_stats.pitch_min_deg);
    } else {
        std::printf("  pitch 진동폭: --\n");
    }
    float pitch_residual_deg = 0.0f;
    if (currentRelativePitch(pitch_residual_deg)) {
        std::printf("  pitch 잔차: %+.1f deg\n", pitch_residual_deg);
    } else {
        std::printf("  pitch 잔차: --\n");
    }
    std::printf("  거부된 스텝: %lu회\n"
                "  body_x 사용 범위: %+.1f ~ %+.1f mm (한계 ±%.1f)\n",
                static_cast<unsigned long>(g_walk_stats.rejected_steps),
                g_comp_body_x_min_mm, g_comp_body_x_max_mm,
                tilt_pose_test::kCompBodyXMaxMm);
    if (g_walk_stats.roll_valid && roll_peak_to_peak * 0.5f < 9.5f) {
        std::printf("\n  참고: 발이 뜨지 않고 끌리는 경우\n"
                    "    - 현재 발판 안쪽 가장자리 y = 18.2mm, 필요 실제 roll ≈ 9.5deg\n"
                    "    - 실측 roll 진폭이 그보다 작으면 발은 뜨지 않는다\n"
                    "    - 발판을 안쪽으로 10mm 넓히면 필요 roll 이 약 4.3deg로 낮아진다\n");
    }
}

void stopWalkMotion(const char* reason, bool return_home) {
    if (!g_walk_running.exchange(false)) {
        return;
    }
    g_walk_summary_legx_mm = g_walk_legx_mm;
    g_walk_summary_delta_mm = g_walk_delta_mm;
    g_walk_summary_leg_time_ms = g_walk_leg_time_ms;
    g_interpolator.abort();
    g_walk_legx_mm = 0.0f;
    g_walk_delta_mm = 0.0f;
    g_walk_left.x_mm = 0.0f;
    g_walk_right.x_mm = 0.0f;
    std::printf("Walk stopped: %s.\n", reason);
    if (return_home && g_state.load() == SafetyState::ARMED) {
        beginMove(tilt::ZERO_POSE_RAD, tilt_pose_test::kPoseDurationMs);
    }
}

void startWalkMotion() {
    if (!g_walk_ready.load() || g_walk_running.load()) {
        return;
    }
    g_walk_stats.transitions = 0;
    g_walk_stats.consecutive_rejects = 0;
    g_walk_stats.cycle_roll_valid = false;
    g_walk_left = {};
    g_walk_right = {};
    setWalkLegPhase(g_walk_left, 0);
    setWalkLegPhase(g_walk_right, 2);
    g_walk_running.store(true);
    std::printf("전진량은 바닥에 표시를 두고 눈으로 재세요. 사이클 수가 출력되므로\n"
                "(이동거리 / 사이클 수) 로 스텝당 전진량을 계산할 수 있습니다.\n");
    if (!applyWalkTarget()) {
        ++g_walk_stats.rejected_steps;
        ++g_walk_stats.consecutive_rejects;
        std::printf("Walk initial step skipped (1/%lu consecutive rejects).\n",
                    static_cast<unsigned long>(
                        tilt_pose_test::kWalkMaxConsecutiveRejects));
    }
    g_walk_next_action_ms = nowMs() + g_walk_leg_time_ms;
    std::printf("Walk started.\n");
}

void requestWalkMode() {
    if (!isArmed("walk")) {
        return;
    }
    std::printf("\nWALK SAFETY: 바닥에 세우고 손을 받칠 준비를 하세요.\n"
                "앞으로 걸어갈 공간이 확보되어 있습니까? (y/n): ");
    std::fflush(stdout);
    g_walk_confirmation_pending.store(true);
}

void confirmWalkMode() {
    if (!isArmed("walk")) {
        g_walk_confirmation_pending.store(false);
        return;
    }
    g_walk_confirmation_pending.store(false);
    g_walk_mode.store(true);
    g_walk_ready.store(false);
    g_walk_running.store(false);
    g_walk_legx_mm = 0.0f;
    g_walk_delta_mm = tilt_pose_test::kWalkDeltaDefaultMm;
    g_walk_base_height_mm = tilt::ZERO_POSE_HEIGHT_MM;
    g_walk_leg_time_ms = tilt_pose_test::kWalkLegTimeDefaultMs;
    g_walk_summary_legx_mm = 0.0f;
    g_walk_summary_delta_mm = g_walk_delta_mm;
    g_walk_summary_leg_time_ms = g_walk_leg_time_ms;
    g_walk_stats = {};
    g_pitch_comp_enabled = false;
    g_rock_center_comp_enabled = false;
    g_rock_amp_comp_enabled = false;
    resetCompensationOutputs();
    g_roll_zero_valid = false;
    g_pitch_zero_valid = false;
    if (beginMove(tilt::ZERO_POSE_RAD, tilt_pose_test::kPoseDurationMs,
                  false, false, false, true)) {
        std::printf("Returning home before walk mode starts.\n");
    } else {
        g_walk_mode.store(false);
    }
}

void quitWalkMode() {
    if (!g_walk_mode.load()) {
        return;
    }
    if (g_walk_running.load()) {
        stopWalkMotion("user exit", false);
    }
    printWalkSummary();
    g_walk_ready.store(false);
    g_walk_mode.store(false);
    g_pitch_comp_enabled = false;
    g_rock_center_comp_enabled = false;
    g_comp_body_x_mm = 0.0f;
    g_roll_zero_valid = false;
    g_pitch_zero_valid = false;
    g_interpolator.abort();
    beginMove(tilt::ZERO_POSE_RAD, tilt_pose_test::kPoseDurationMs);
    std::printf("Leaving walk mode and returning home.\n");
}

void serviceWalk() {
    if (!g_walk_running.load() || !g_walk_ready.load() ||
        g_state.load() != SafetyState::ARMED || g_interpolator.isBusy()) {
        return;
    }
    const std::uint32_t now = nowMs();
    if (static_cast<std::int32_t>(now - g_walk_next_action_ms) < 0) {
        return;
    }

    setWalkLegPhase(g_walk_left, g_walk_left.phase + 1);
    setWalkLegPhase(g_walk_right, g_walk_right.phase + 1);
    ++g_walk_stats.transitions;
    const bool cycle_complete = (g_walk_stats.transitions % 4) == 0;
    if (cycle_complete) {
        finishWalkCycle();
    }
    if (!applyWalkTarget()) {
        ++g_walk_stats.rejected_steps;
        ++g_walk_stats.consecutive_rejects;
        std::printf("Walk step skipped (%lu/%lu consecutive rejects).\n",
                    static_cast<unsigned long>(g_walk_stats.consecutive_rejects),
                    static_cast<unsigned long>(
                        tilt_pose_test::kWalkMaxConsecutiveRejects));
        if (g_walk_stats.consecutive_rejects >=
            tilt_pose_test::kWalkMaxConsecutiveRejects) {
            stopWalkMotion("three consecutive targets rejected", true);
            return;
        }
    } else {
        g_walk_stats.consecutive_rejects = 0;
    }
    if (cycle_complete) {
        printWalkCycle(g_walk_last_cycle_roll_valid,
                       g_walk_last_cycle_roll_min_deg,
                       g_walk_last_cycle_roll_max_deg);
    }
    g_walk_next_action_ms = now + g_walk_leg_time_ms;
}

void serviceRockAutomation() {
    const RockAutomation automation = g_rock_automation.load();
    if (automation == RockAutomation::NONE || !g_rock_ready.load() ||
        g_state.load() != SafetyState::ARMED) {
        return;
    }
    float safety_roll_deg = g_roll_deg;
    float relative_roll_deg = 0.0f;
    if (currentRelativeRoll(relative_roll_deg)) {
        safety_roll_deg = relative_roll_deg;
    }
    if (g_roll_valid &&
        std::fabs(safety_roll_deg) > tilt_pose_test::kRockRollAbortDeg) {
        stopRockAutomation("IMU roll exceeded 20 degrees", true);
        return;
    }

    const std::uint32_t now = nowMs();
    if (automation == RockAutomation::SWEEP) {
        if (g_rock_sweep_phase == RockSweepPhase::MOVING) {
            if (g_interpolator.isBusy()) {
                return;
            }
            g_rock_sweep_max_delta_mm = std::fmax(g_rock_sweep_max_delta_mm,
                                                  std::fabs(g_rock_delta_mm));
            if (g_roll_valid) {
                g_rock_sweep_roll_valid = true;
                g_rock_sweep_roll_deg = g_roll_deg;
            }
            printRockState();
            g_rock_sweep_phase = RockSweepPhase::HOLDING;
            g_rock_next_action_ms = now + tilt_pose_test::kRockSweepHoldMs;
            return;
        }
        if (static_cast<std::int32_t>(now - g_rock_next_action_ms) < 0) {
            return;
        }
        const float next_delta = g_rock_delta_mm + tilt_pose_test::kRockSweepStepMm;
        if (next_delta > tilt_pose_test::kRockSweepEndMm + 0.001f) {
            g_rock_automation.store(RockAutomation::NONE);
            std::printf("Rock sweep complete: max delta=%.1fmm, ",
                        g_rock_sweep_max_delta_mm);
            if (g_rock_sweep_roll_valid) {
                std::printf("roll=%+.1fdeg.\n", g_rock_sweep_roll_deg);
            } else {
                std::printf("roll=--.\n");
            }
            printLiftRecord("recorded left lift ", g_left_lift);
            printLiftRecord("recorded right lift", g_right_lift);
            return;
        }
        g_rock_sweep_phase = RockSweepPhase::MOVING;
        if (!applyRockTarget(next_delta, g_rock_base_height_mm,
                             tilt_pose_test::kRockStepDurationMs, false)) {
            stopRockAutomation("sweep target rejected", true);
        }
        return;
    }

    if (g_interpolator.isBusy() ||
        static_cast<std::int32_t>(now - g_rock_next_action_ms) < 0) {
        return;
    }
    const float next_delta = g_rock_delta_mm >= 0.0f
                                 ? -g_rock_alternate_amplitude_mm
                                 : +g_rock_alternate_amplitude_mm;
    if (g_rock_cycle_started) {
        finishRockCycle(next_delta);
        if (!calculateAlternateMoveDuration(g_rock_alternate_move_duration_ms)) {
            stopRockAutomation("compensated target rejected", true);
            return;
        }
        ensureAlternatePeriod(g_rock_alternate_move_duration_ms);
    } else {
        g_rock_cycle_started = true;
        resetRockCyclePeaks();
    }
    if (!applyRockTarget(next_delta, g_rock_base_height_mm,
                         g_rock_alternate_move_duration_ms, false)) {
        stopRockAutomation("alternating target rejected", true);
        return;
    }
    g_alternate_stats.move_pending = true;
    g_rock_next_action_ms = now + g_rock_alternate_period_ms;
}

void serviceMotion() {
    if (g_estop_requested.exchange(false)) {
        g_interpolator.abort();
        g_start_ik_when_idle = false;
        g_start_stand_when_idle = false;
        g_start_rock_when_idle = false;
        g_start_walk_when_idle = false;
        g_walk_running.store(false);
        g_rock_automation.store(RockAutomation::NONE);
        g_pitch_comp_enabled = false;
        g_rock_center_comp_enabled = false;
        g_rock_amp_comp_enabled = false;
        resetCompensationOutputs();
        g_warn_rock_zero_on_arrival = false;
        return;
    }
    if (!g_interpolator.isBusy() || g_state.load() != SafetyState::ARMED) {
        return;
    }

    float next_goal[tilt::NUM_JOINTS]{};
    const bool still_moving = g_interpolator.update(nowMs(), next_goal);
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        if (std::fabs(next_goal[joint] - g_goal_rad[joint]) >
            tilt_pose_test::kMaxJointStepRad) {
            g_interpolator.abort();
            g_start_ik_when_idle = false;
            g_start_stand_when_idle = false;
            g_start_rock_when_idle = false;
            g_start_walk_when_idle = false;
            g_warn_rock_zero_on_arrival = false;
            const RockAutomation stopped =
                g_rock_automation.exchange(RockAutomation::NONE);
            if (stopped == RockAutomation::ALTERNATE) {
                printAlternateSummary();
            }
            if (stopped != RockAutomation::NONE) {
                std::printf("Rock automation stopped: joint speed guard.\n");
            }
            if (g_walk_running.exchange(false)) {
                g_walk_summary_legx_mm = g_walk_legx_mm;
                g_walk_summary_delta_mm = g_walk_delta_mm;
                g_walk_summary_leg_time_ms = g_walk_leg_time_ms;
                g_walk_legx_mm = 0.0f;
                g_walk_delta_mm = 0.0f;
                std::printf("Walk stopped: joint speed guard.\n");
            }

            // g_goal_rad is the last command that was actually transmitted.
            // Aborting leaves both the interpolation origin and held goal there.
            if (!g_speed_guard_active || g_speed_guard_joint != joint) {
                ESP_LOGW(kTag, "Speed guard aborted move at %s (%+.3f deg step).",
                         tilt::JOINT_NAME[joint],
                         (next_goal[joint] - g_goal_rad[joint]) * kRadToDeg);
            }
            g_speed_guard_active = true;
            g_speed_guard_joint = joint;
            return;
        }
    }
    if (!sendSixJointGoal(next_goal)) {
        emergencyStopNow();
        return;
    }
    std::memcpy(g_goal_rad, next_goal, sizeof(g_goal_rad));
    g_speed_guard_active = false;
    g_speed_guard_joint = -1;
    if (!still_moving) {
        if (g_rock_automation.load() == RockAutomation::ALTERNATE &&
            g_alternate_stats.move_pending) {
            ++g_alternate_stats.cycles;
            g_alternate_stats.move_pending = false;
        }
        warnRockRollResidualIfNeeded();
        if (g_rock_automation.load() == RockAutomation::NONE &&
            !g_walk_running.load()) {
            std::printf("Move complete.\n");
        }
        if (g_start_ik_when_idle) {
            g_start_ik_when_idle = false;
            g_ik_mode.store(true);
            std::printf("IK mode active: arrows/WASD move body, q exits, ! E-STOP.\n");
        }
        if (g_start_stand_when_idle) {
            g_start_stand_when_idle = false;
            g_roll_zero_valid = g_roll_valid;
            if (g_roll_zero_valid) g_roll_zero_deg = g_roll_deg;
            g_pitch_zero_valid = g_pitch_valid;
            if (g_pitch_zero_valid) g_pitch_zero_deg = g_pitch_deg;
            g_stand_last_sent_body_x_mm = 0.0f;
            g_stand_last_sent_height_mm = g_stand_base_height_mm;
            g_last_status_print_ms = 0;
            g_stand_ready.store(true);
            std::printf("Stand mode active: compensation starts OFF. "
                        "Press c to enable, t for step response, q to exit.\n");
            printStandState();
        }
        if (g_start_rock_when_idle) {
            g_start_rock_when_idle = false;
            g_roll_zero_valid = g_roll_valid;
            if (g_roll_zero_valid) {
                g_roll_zero_deg = g_roll_deg;
            }
            g_pitch_zero_valid = g_pitch_valid;
            if (g_pitch_zero_valid) {
                g_pitch_zero_deg = g_pitch_deg;
            }
            g_rock_ready.store(true);
            std::printf("Rocking mode active on floor: arrows/WASD adjust, "
                        "f toggles IK/FLAT, 1 sweep, 2 alternate, k records lift, "
                        "r resets attitude, "
                        "q exits, ! E-STOP.\n");
            printRockState();
        }
        if (g_start_walk_when_idle) {
            g_start_walk_when_idle = false;
            g_roll_zero_valid = g_roll_valid;
            if (g_roll_zero_valid) g_roll_zero_deg = g_roll_deg;
            g_pitch_zero_valid = g_pitch_valid;
            if (g_pitch_zero_valid) g_pitch_zero_deg = g_pitch_deg;
            g_walk_ready.store(true);
            std::printf("Walk mode ready and stopped: press space to start. "
                        "legx begins at 0.0mm; q exits, ! E-STOP.\n");
            printWalkState();
        }
    }
}

int jointIndex(const char* name) {
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        if (strcasecmp(name, tilt::JOINT_NAME[joint]) == 0) {
            return joint;
        }
    }
    return -1;
}

void handleCommand(const Command& command) {
    switch (command.type) {
        case CommandType::HELP: printHelp(); return;
        case CommandType::STATUS: printStatus(); return;
        case CommandType::CHECK: printCheck(); return;
        case CommandType::ARM: arm(); return;
        case CommandType::DISARM: disarm(); return;
        case CommandType::RECOVER: recover(); return;
        case CommandType::FK: printFk(); return;
        case CommandType::HOME:
            if (isArmed("home")) {
                beginMove(tilt::ZERO_POSE_RAD, tilt_pose_test::kPoseDurationMs);
            }
            return;
        case CommandType::POSE: {
            if (!isArmed("pose")) {
                return;
            }
            for (const auto& pose : tilt_pose_test::kPoses) {
                if (strcasecmp(command.name, pose.name) == 0) {
                    float target[tilt::NUM_JOINTS]{};
                    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
                        target[joint] = pose.deg[joint] * tilt::DEG2RAD;
                    }
                    beginMove(target, tilt_pose_test::kPoseDurationMs);
                    return;
                }
            }
            std::printf("Unknown pose '%s'.\n", command.name);
            return;
        }
        case CommandType::JOINT: {
            if (!isArmed("joint")) {
                return;
            }
            const int joint = jointIndex(command.name);
            if (joint < 0 || !std::isfinite(command.value) || command.value == 0.0f ||
                std::fabs(command.value) > tilt_pose_test::kMaxJogStepDeg) {
                std::printf("Usage: joint <LHY|LHP|LKP|RHY|RHP|RKP> <-5..+5>.\n");
                return;
            }
            float target[tilt::NUM_JOINTS]{};
            std::memcpy(target, g_goal_rad, sizeof(target));
            target[joint] += command.value * tilt::DEG2RAD;
            beginMove(target, tilt_pose_test::kJointDurationMs);
            return;
        }
        case CommandType::IK_ENTER: enterIkMode(); return;
        case CommandType::IK_TOGGLE_STEP:
            if (g_ik_mode.load()) {
                g_ik_coarse = !g_ik_coarse;
                std::printf("IK step: %s (%.1f mm).\n", g_ik_coarse ? "COARSE" : "FINE",
                            g_ik_coarse ? tilt_pose_test::kIkCoarseStepMm :
                                          tilt_pose_test::kIkFineStepMm);
            }
            return;
        case CommandType::IK_HOME:
            if (g_ik_mode.load()) {
                handleIkMove(-g_body_x_mm,
                             tilt::ZERO_POSE_HEIGHT_MM - g_body_height_mm);
            }
            return;
        case CommandType::IK_VERIFY:
            if (g_ik_mode.load()) {
                printIkVerification();
            }
            return;
        case CommandType::IK_QUIT:
            if (g_ik_mode.exchange(false)) {
                beginMove(tilt::ZERO_POSE_RAD, tilt_pose_test::kPoseDurationMs);
                std::printf("Leaving IK mode and returning home.\n");
            }
            return;
        case CommandType::IK_UP:
        case CommandType::IK_DOWN:
        case CommandType::IK_RIGHT:
        case CommandType::IK_LEFT: {
            const float step = g_ik_coarse ? tilt_pose_test::kIkCoarseStepMm :
                                             tilt_pose_test::kIkFineStepMm;
            if (command.type == CommandType::IK_UP) handleIkMove(0.0f, step);
            if (command.type == CommandType::IK_DOWN) handleIkMove(0.0f, -step);
            if (command.type == CommandType::IK_RIGHT) handleIkMove(step, 0.0f);
            if (command.type == CommandType::IK_LEFT) handleIkMove(-step, 0.0f);
            return;
        }
        case CommandType::STAND_ENTER: enterStandMode(); return;
        case CommandType::STAND_UP:
            adjustStandHeight(tilt_pose_test::kRockFineStepMm);
            return;
        case CommandType::STAND_DOWN:
            adjustStandHeight(-tilt_pose_test::kRockFineStepMm);
            return;
        case CommandType::STAND_TOGGLE_COMP:
            if (g_stand_ready.load()) togglePitchCompensation();
            return;
        case CommandType::STAND_KP_UP:
            adjustPitchGain(+tilt_pose_test::kCompKpPitchStep);
            return;
        case CommandType::STAND_KP_DOWN:
            adjustPitchGain(-tilt_pose_test::kCompKpPitchStep);
            return;
        case CommandType::STAND_LPF_DOWN:
            adjustCompLpf(-tilt_pose_test::kCompLpfAlphaStep);
            return;
        case CommandType::STAND_LPF_UP:
            adjustCompLpf(+tilt_pose_test::kCompLpfAlphaStep);
            return;
        case CommandType::STAND_RESET_ATTITUDE:
            if (g_stand_ready.load()) resetRockAttitudeZero();
            return;
        case CommandType::STAND_ZERO: zeroStandCompensation(); return;
        case CommandType::STAND_STEP_TEST: startStepResponseTest(); return;
        case CommandType::STAND_VERIFY:
            if (g_stand_ready.load()) printStandState();
            return;
        case CommandType::STAND_QUIT: quitStandMode(); return;
        case CommandType::ROCK_ENTER: requestRockMode(); return;
        case CommandType::ROCK_CONFIRM: confirmRockMode(); return;
        case CommandType::ROCK_CANCEL_ENTRY:
            g_rock_confirmation_pending.store(false);
            std::printf("Rocking mode entry cancelled.\n");
            return;
        case CommandType::ROCK_TOGGLE_STEP:
            if (g_rock_ready.load()) {
                g_rock_coarse = !g_rock_coarse;
                std::printf("Rock step: %s (%.1fmm).\n",
                            g_rock_coarse ? "COARSE" : "FINE",
                            g_rock_coarse ? tilt_pose_test::kRockCoarseStepMm :
                                            tilt_pose_test::kRockFineStepMm);
            }
            return;
        case CommandType::ROCK_TOGGLE_METHOD: toggleRockMethod(); return;
        case CommandType::ROCK_TOGGLE_PITCH_COMP:
            if (g_rock_ready.load()) togglePitchCompensation();
            return;
        case CommandType::ROCK_TOGGLE_CENTER_COMP:
            if (g_rock_ready.load()) toggleRockCenterCompensation();
            return;
        case CommandType::ROCK_TOGGLE_AMP_COMP:
            if (g_rock_ready.load()) toggleRockAmplitudeCompensation();
            return;
        case CommandType::ROCK_KP_UP:
            adjustPitchGain(+tilt_pose_test::kCompKpPitchStep);
            return;
        case CommandType::ROCK_KP_DOWN:
            adjustPitchGain(-tilt_pose_test::kCompKpPitchStep);
            return;
        case CommandType::ROCK_ZERO:
            if (g_rock_ready.load()) {
                applyRockTarget(0.0f, g_rock_base_height_mm,
                                tilt_pose_test::kRockStepDurationMs);
            }
            return;
        case CommandType::ROCK_SWEEP: startRockSweep(); return;
        case CommandType::ROCK_ALTERNATE: startRockAlternate(); return;
        case CommandType::ROCK_RECORD: recordFootLift(); return;
        case CommandType::ROCK_RESET_ROLL:
            if (g_rock_ready.load()) {
                resetRockAttitudeZero();
            }
            return;
        case CommandType::ROCK_VERIFY:
            if (g_rock_ready.load()) {
                printRockState();
            }
            return;
        case CommandType::ROCK_QUIT: quitRockMode(); return;
        case CommandType::ROCK_PERIOD_DOWN:
            adjustRockAlternatePeriod(
                -static_cast<int>(tilt_pose_test::kRockAlternatePeriodStepMs));
            return;
        case CommandType::ROCK_PERIOD_UP:
            adjustRockAlternatePeriod(
                static_cast<int>(tilt_pose_test::kRockAlternatePeriodStepMs));
            return;
        case CommandType::ROCK_CANCEL_AUTOMATION:
            stopRockAutomation("user input", true);
            return;
        case CommandType::ROCK_CANCEL_AUTOMATION_KEEP_POSITION:
            stopRockAutomation("method toggle", false);
            return;
        case CommandType::ROCK_UP:
        case CommandType::ROCK_DOWN:
        case CommandType::ROCK_RIGHT:
        case CommandType::ROCK_LEFT: {
            const float step = g_rock_coarse ? tilt_pose_test::kRockCoarseStepMm :
                                               tilt_pose_test::kRockFineStepMm;
            if (command.type == CommandType::ROCK_UP) handleRockMove(0.0f, step);
            if (command.type == CommandType::ROCK_DOWN) handleRockMove(0.0f, -step);
            if (command.type == CommandType::ROCK_RIGHT) handleRockMove(step, 0.0f);
            if (command.type == CommandType::ROCK_LEFT) handleRockMove(-step, 0.0f);
            return;
        }
        case CommandType::WALK_ENTER: requestWalkMode(); return;
        case CommandType::WALK_CONFIRM: confirmWalkMode(); return;
        case CommandType::WALK_CANCEL_ENTRY:
            g_walk_confirmation_pending.store(false);
            std::printf("Walk mode entry cancelled.\n");
            return;
        case CommandType::WALK_TOGGLE:
            if (!g_walk_ready.load()) return;
            if (g_walk_running.load()) stopWalkMotion("user input", true);
            else startWalkMotion();
            return;
        case CommandType::WALK_RIGHT:
        case CommandType::WALK_LEFT:
            if (g_walk_ready.load()) {
                const float change = command.type == CommandType::WALK_RIGHT
                                         ? tilt_pose_test::kWalkLegxStepMm
                                         : -tilt_pose_test::kWalkLegxStepMm;
                g_walk_legx_mm = std::fmax(
                    -tilt_pose_test::kWalkLegxMaxMm,
                    std::fmin(tilt_pose_test::kWalkLegxMaxMm,
                              g_walk_legx_mm + change));
                std::printf("Walk legx=%+.1fmm.\n", g_walk_legx_mm);
            }
            return;
        case CommandType::WALK_UP:
        case CommandType::WALK_DOWN:
            if (g_walk_ready.load()) {
                const float change = command.type == CommandType::WALK_UP
                                         ? tilt_pose_test::kRockFineStepMm
                                         : -tilt_pose_test::kRockFineStepMm;
                g_walk_base_height_mm = std::fmax(
                    tilt_pose_test::kBodyHeightMinMm,
                    std::fmin(tilt_pose_test::kBodyHeightMaxMm,
                              g_walk_base_height_mm + change));
                std::printf("Walk base height=%.2fmm.\n", g_walk_base_height_mm);
            }
            return;
        case CommandType::WALK_DELTA_UP:
        case CommandType::WALK_DELTA_DOWN:
            if (g_walk_ready.load()) {
                const float change = command.type == CommandType::WALK_DELTA_UP
                                         ? 1.0f : -1.0f;
                g_walk_delta_mm = std::fmax(
                    0.0f, std::fmin(tilt_pose_test::kRockDeltaMaxMm,
                                    g_walk_delta_mm + change));
                std::printf("Walk delta=%.1fmm.\n", g_walk_delta_mm);
            }
            return;
        case CommandType::WALK_PERIOD_DOWN:
        case CommandType::WALK_PERIOD_UP:
            if (g_walk_ready.load()) {
                const int change = command.type == CommandType::WALK_PERIOD_UP
                                       ? static_cast<int>(tilt_pose_test::kWalkLegTimeStepMs)
                                       : -static_cast<int>(tilt_pose_test::kWalkLegTimeStepMs);
                const int requested = static_cast<int>(g_walk_leg_time_ms) + change;
                const int bounded = std::max(
                    static_cast<int>(tilt_pose_test::kWalkLegTimeMinMs),
                    std::min(static_cast<int>(tilt_pose_test::kWalkLegTimeMaxMs),
                             requested));
                g_walk_leg_time_ms = static_cast<std::uint32_t>(bounded);
                std::printf("Walk leg_time=%lums.\n",
                            static_cast<unsigned long>(g_walk_leg_time_ms));
            }
            return;
        case CommandType::WALK_TOGGLE_PITCH_COMP:
            if (g_walk_ready.load()) togglePitchCompensation();
            return;
        case CommandType::WALK_TOGGLE_CENTER_COMP:
            if (g_walk_ready.load()) toggleRockCenterCompensation();
            return;
        case CommandType::WALK_ZERO_X:
            if (g_walk_ready.load()) {
                g_walk_legx_mm = 0.0f;
                std::printf("Walk legx reset to 0.0mm.\n");
            }
            return;
        case CommandType::WALK_RESET_ATTITUDE:
            if (g_walk_ready.load()) resetRockAttitudeZero();
            return;
        case CommandType::WALK_VERIFY:
            if (g_walk_ready.load()) printWalkState();
            return;
        case CommandType::WALK_QUIT: quitWalkMode(); return;
    }
}

void enqueue(Command command) {
    if (xQueueSend(g_command_queue, &command, 0) != pdTRUE) {
        std::printf("Command queue full; command ignored.\n");
    }
}

void parseLine(char* line) {
    char operation[20]{};
    char first[24]{};
    char second[24]{};
    const int count = std::sscanf(line, "%19s %23s %23s", operation, first, second);
    if (count <= 0) {
        return;
    }
    if (strcasecmp(operation, "help") == 0) enqueue({CommandType::HELP});
    else if (strcasecmp(operation, "status") == 0) enqueue({CommandType::STATUS});
    else if (strcasecmp(operation, "check") == 0) enqueue({CommandType::CHECK});
    else if (strcasecmp(operation, "arm") == 0) enqueue({CommandType::ARM});
    else if (strcasecmp(operation, "disarm") == 0) enqueue({CommandType::DISARM});
    else if (strcasecmp(operation, "recover") == 0) enqueue({CommandType::RECOVER});
    else if (strcasecmp(operation, "home") == 0) enqueue({CommandType::HOME});
    else if (strcasecmp(operation, "fk") == 0) enqueue({CommandType::FK});
    else if (strcasecmp(operation, "ik") == 0) enqueue({CommandType::IK_ENTER});
    else if (strcasecmp(operation, "stand") == 0) enqueue({CommandType::STAND_ENTER});
    else if (strcasecmp(operation, "rock") == 0) enqueue({CommandType::ROCK_ENTER});
    else if (strcasecmp(operation, "walk") == 0) enqueue({CommandType::WALK_ENTER});
    else if (strcasecmp(operation, "pose") == 0 && count == 2) {
        Command command{CommandType::POSE};
        std::strncpy(command.name, first, sizeof(command.name) - 1);
        enqueue(command);
    } else if (strcasecmp(operation, "joint") == 0 && count == 3) {
        char* end = nullptr;
        const float degrees = std::strtof(second, &end);
        if (end == second || *end != '\0') {
            std::printf("Joint delta must be a number.\n");
            return;
        }
        Command command{CommandType::JOINT};
        std::strncpy(command.name, first, sizeof(command.name) - 1);
        command.value = degrees;
        enqueue(command);
    } else {
        std::printf("Unknown command. Type help.\n");
    }
}

void enqueueIkKey(std::uint8_t key) {
    if (key == '!') {
        emergencyStopNow();
        return;
    }
    CommandType type;
    switch (key) {
        case 'w': type = CommandType::IK_UP; break;
        case 's': type = CommandType::IK_DOWN; break;
        case 'd': type = CommandType::IK_RIGHT; break;
        case 'a': type = CommandType::IK_LEFT; break;
        case 'm': type = CommandType::IK_TOGGLE_STEP; break;
        case '0': type = CommandType::IK_HOME; break;
        case 'v': type = CommandType::IK_VERIFY; break;
        case 'q': type = CommandType::IK_QUIT; break;
        default: return;
    }
    enqueue({type});
}

void enqueueStandKey(std::uint8_t key) {
    CommandType type;
    switch (key) {
        case 'c': case 'C': type = CommandType::STAND_TOGGLE_COMP; break;
        case '+': type = CommandType::STAND_KP_UP; break;
        case '-': type = CommandType::STAND_KP_DOWN; break;
        case '[': type = CommandType::STAND_LPF_DOWN; break;
        case ']': type = CommandType::STAND_LPF_UP; break;
        case 'r': case 'R': type = CommandType::STAND_RESET_ATTITUDE; break;
        case '0': type = CommandType::STAND_ZERO; break;
        case 't': case 'T': type = CommandType::STAND_STEP_TEST; break;
        case 'v': case 'V': type = CommandType::STAND_VERIFY; break;
        case 'q': case 'Q': type = CommandType::STAND_QUIT; break;
        default: return;
    }
    enqueue({type});
}

void enqueueRockKey(std::uint8_t key) {
    CommandType type;
    switch (key) {
        case 'w': case 'W': type = CommandType::ROCK_UP; break;
        case 's': case 'S': type = CommandType::ROCK_DOWN; break;
        case 'd': case 'D': type = CommandType::ROCK_RIGHT; break;
        case 'a': case 'A': type = CommandType::ROCK_LEFT; break;
        case 'm': case 'M': type = CommandType::ROCK_TOGGLE_STEP; break;
        case 'f': case 'F': type = CommandType::ROCK_TOGGLE_METHOD; break;
        case 'c': case 'C': type = CommandType::ROCK_TOGGLE_PITCH_COMP; break;
        case 'x': case 'X': type = CommandType::ROCK_TOGGLE_CENTER_COMP; break;
        case 'z': case 'Z': type = CommandType::ROCK_TOGGLE_AMP_COMP; break;
        case '+': type = CommandType::ROCK_KP_UP; break;
        case '-': type = CommandType::ROCK_KP_DOWN; break;
        case '0': type = CommandType::ROCK_ZERO; break;
        case '1': type = CommandType::ROCK_SWEEP; break;
        case '2': type = CommandType::ROCK_ALTERNATE; break;
        case 'k': case 'K': type = CommandType::ROCK_RECORD; break;
        case 'r': case 'R': type = CommandType::ROCK_RESET_ROLL; break;
        case 'v': case 'V': type = CommandType::ROCK_VERIFY; break;
        case 'q': case 'Q': type = CommandType::ROCK_QUIT; break;
        default: return;
    }
    enqueue({type});
}

void enqueueWalkKey(std::uint8_t key) {
    CommandType type;
    switch (key) {
        case ' ': type = CommandType::WALK_TOGGLE; break;
        case 'w': case 'W': type = CommandType::WALK_UP; break;
        case 's': case 'S': type = CommandType::WALK_DOWN; break;
        case 'd': case 'D': type = CommandType::WALK_RIGHT; break;
        case 'a': case 'A': type = CommandType::WALK_LEFT; break;
        case '+': type = CommandType::WALK_DELTA_UP; break;
        case '-': type = CommandType::WALK_DELTA_DOWN; break;
        case '[': type = CommandType::WALK_PERIOD_DOWN; break;
        case ']': type = CommandType::WALK_PERIOD_UP; break;
        case 'c': case 'C': type = CommandType::WALK_TOGGLE_PITCH_COMP; break;
        case 'x': case 'X': type = CommandType::WALK_TOGGLE_CENTER_COMP; break;
        case '0': type = CommandType::WALK_ZERO_X; break;
        case 'r': case 'R': type = CommandType::WALK_RESET_ATTITUDE; break;
        case 'v': case 'V': type = CommandType::WALK_VERIFY; break;
        case 'q': case 'Q': type = CommandType::WALK_QUIT; break;
        default: return;
    }
    enqueue({type});
}

void consoleTask(void*) {
    enum class EscapeState : std::uint8_t { NONE, ESC, BRACKET };
    EscapeState escape_state = EscapeState::NONE;
    char line[kConsoleLineCapacity]{};
    std::size_t length = 0;
    printHelp();
    std::printf("> ");
    std::fflush(stdout);

    while (true) {
        std::uint8_t byte = 0;
        if (usb_serial_jtag_read_bytes(&byte, 1, pdMS_TO_TICKS(50)) <= 0) {
            continue;
        }
        if (byte == '!') {
            emergencyStopNow();
            length = 0;
            escape_state = EscapeState::NONE;
            continue;
        }

        if (g_rock_confirmation_pending.load()) {
            g_rock_confirmation_pending.store(false);
            enqueue({byte == 'y' || byte == 'Y' ? CommandType::ROCK_CONFIRM
                                                : CommandType::ROCK_CANCEL_ENTRY});
            continue;
        }

        if (g_walk_confirmation_pending.load()) {
            g_walk_confirmation_pending.store(false);
            enqueue({byte == 'y' || byte == 'Y' ? CommandType::WALK_CONFIRM
                                                : CommandType::WALK_CANCEL_ENTRY});
            continue;
        }

        if (g_walk_mode.load()) {
            if (escape_state == EscapeState::ESC) {
                escape_state = byte == '[' ? EscapeState::BRACKET
                                            : EscapeState::NONE;
                continue;
            }
            if (escape_state == EscapeState::BRACKET) {
                switch (byte) {
                    case 'A': enqueue({CommandType::WALK_UP}); break;
                    case 'B': enqueue({CommandType::WALK_DOWN}); break;
                    case 'C': enqueue({CommandType::WALK_RIGHT}); break;
                    case 'D': enqueue({CommandType::WALK_LEFT}); break;
                    default: break;
                }
                escape_state = EscapeState::NONE;
                continue;
            }
            if (byte == 0x1B) {
                escape_state = EscapeState::ESC;
            } else {
                enqueueWalkKey(byte);
            }
            continue;
        }

        if (g_stand_mode.load()) {
            if (escape_state == EscapeState::ESC) {
                escape_state = byte == '[' ? EscapeState::BRACKET
                                            : EscapeState::NONE;
                continue;
            }
            if (escape_state == EscapeState::BRACKET) {
                if (byte == 'A') enqueue({CommandType::STAND_UP});
                if (byte == 'B') enqueue({CommandType::STAND_DOWN});
                escape_state = EscapeState::NONE;
                continue;
            }
            if (byte == 0x1B) {
                escape_state = EscapeState::ESC;
            } else {
                enqueueStandKey(byte);
            }
            continue;
        }

        if (g_rock_mode.load()) {
            const RockAutomation automation = g_rock_automation.load();
            if (automation == RockAutomation::SWEEP) {
                if (byte == 'c' || byte == 'C' || byte == 'x' || byte == 'X' ||
                    byte == 'z' || byte == 'Z' || byte == '+' || byte == '-' ||
                    byte == 'v' || byte == 'V') {
                    enqueueRockKey(byte);
                    escape_state = EscapeState::NONE;
                    continue;
                }
                if (byte == 'f' || byte == 'F') {
                    enqueue({CommandType::ROCK_CANCEL_AUTOMATION_KEEP_POSITION});
                    enqueue({CommandType::ROCK_TOGGLE_METHOD});
                    escape_state = EscapeState::NONE;
                    continue;
                }
                if (byte == 'k' || byte == 'K') {
                    enqueue({CommandType::ROCK_RECORD});
                }
                enqueue({CommandType::ROCK_CANCEL_AUTOMATION});
                if (byte == 'r' || byte == 'R') {
                    enqueue({CommandType::ROCK_RESET_ROLL});
                }
                if (byte == 'q' || byte == 'Q') {
                    enqueue({CommandType::ROCK_QUIT});
                }
                escape_state = EscapeState::NONE;
                continue;
            }
            if (automation == RockAutomation::ALTERNATE) {
                if (byte == '[') {
                    enqueue({CommandType::ROCK_PERIOD_DOWN});
                } else if (byte == ']') {
                    enqueue({CommandType::ROCK_PERIOD_UP});
                } else if (byte == 'c' || byte == 'C' ||
                           byte == 'x' || byte == 'X' ||
                           byte == 'z' || byte == 'Z' ||
                           byte == '+' || byte == '-' ||
                           byte == 'v' || byte == 'V') {
                    enqueueRockKey(byte);
                } else if (byte == 'f' || byte == 'F') {
                    enqueue({CommandType::ROCK_CANCEL_AUTOMATION_KEEP_POSITION});
                    enqueue({CommandType::ROCK_TOGGLE_METHOD});
                } else {
                    if (byte == 'k' || byte == 'K') {
                        enqueue({CommandType::ROCK_RECORD});
                    }
                    enqueue({CommandType::ROCK_CANCEL_AUTOMATION});
                    if (byte == 'r' || byte == 'R') {
                        enqueue({CommandType::ROCK_RESET_ROLL});
                    }
                    if (byte == 'q' || byte == 'Q') {
                        enqueue({CommandType::ROCK_QUIT});
                    }
                }
                escape_state = EscapeState::NONE;
                continue;
            }
            if (escape_state == EscapeState::ESC) {
                escape_state = byte == '[' ? EscapeState::BRACKET : EscapeState::NONE;
                continue;
            }
            if (escape_state == EscapeState::BRACKET) {
                switch (byte) {
                    case 'A': enqueue({CommandType::ROCK_UP}); break;
                    case 'B': enqueue({CommandType::ROCK_DOWN}); break;
                    case 'C': enqueue({CommandType::ROCK_RIGHT}); break;
                    case 'D': enqueue({CommandType::ROCK_LEFT}); break;
                    default: break;
                }
                escape_state = EscapeState::NONE;
                continue;
            }
            if (byte == 0x1B) {
                escape_state = EscapeState::ESC;
            } else {
                enqueueRockKey(byte);
            }
            continue;
        }

        if (g_ik_mode.load()) {
            if (escape_state == EscapeState::ESC) {
                escape_state = byte == '[' ? EscapeState::BRACKET : EscapeState::NONE;
                continue;
            }
            if (escape_state == EscapeState::BRACKET) {
                switch (byte) {
                    case 'A': enqueue({CommandType::IK_UP}); break;
                    case 'B': enqueue({CommandType::IK_DOWN}); break;
                    case 'C': enqueue({CommandType::IK_RIGHT}); break;
                    case 'D': enqueue({CommandType::IK_LEFT}); break;
                    default: break;
                }
                escape_state = EscapeState::NONE;
                continue;
            }
            if (byte == 0x1B) {
                escape_state = EscapeState::ESC;
            } else {
                enqueueIkKey(byte);
            }
            continue;
        }

        if (byte == '\r' || byte == '\n') {
            if (length > 0) {
                line[length] = '\0';
                parseLine(line);
                length = 0;
            }
            std::printf("> ");
            std::fflush(stdout);
        } else if ((byte == 0x08 || byte == 0x7F) && length > 0) {
            --length;
        } else if (byte >= 0x20 && byte <= 0x7E && length + 1 < sizeof(line)) {
            line[length++] = static_cast<char>(byte);
        }
    }
}

}  // namespace

extern "C" void app_main() {
    usb_serial_jtag_driver_config_t console_config =
        USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    const esp_err_t console_result =
        usb_serial_jtag_driver_install(&console_config);
    if (console_result != ESP_OK && console_result != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(kTag, "USB Serial/JTAG console initialization failed: %s",
                 esp_err_to_name(console_result));
        return;
    }
    ESP_ERROR_CHECK(g_bus.initialize());
    g_imu_available = tilt::imu_init();
    if (!g_imu_available) {
        ESP_LOGW(kTag, "MPU6050 unavailable; rocking continues with roll/pitch=--.");
    }

    // Boot invariant: never move automatically and always request torque OFF.
    torqueOffBestEffort();
    std::printf("\nBoot complete: state=DISARMED, torque OFF, no automatic home move.\n");
    std::printf("ROCK WARNING: use rocking mode on the floor, with hands ready to catch "
                "the robot if it tips.\n");

    g_command_queue = xQueueCreate(kCommandQueueDepth, sizeof(Command));
    if (g_command_queue == nullptr) {
        ESP_LOGE(kTag, "Could not create console command queue.");
        return;
    }
    xTaskCreate(consoleTask, "pose_console", 4096, nullptr, 5, nullptr);

    TickType_t wake_time = xTaskGetTickCount();
    while (true) {
        serviceImu();
        sampleAlternateAttitude();
        sampleWalkAttitude();
        Command command{};
        while (xQueueReceive(g_command_queue, &command, 0) == pdTRUE) {
            handleCommand(command);
        }
        serviceCompensation();
        serviceMotion();
        serviceRockAutomation();
        serviceWalk();
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(tilt::CONTROL_PERIOD_MS));
    }
}
