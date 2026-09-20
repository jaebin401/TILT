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

namespace {

constexpr char kTag[] = "pose_test";
constexpr std::size_t kConsoleLineCapacity = 96;
constexpr UBaseType_t kCommandQueueDepth = 20;
constexpr float kRadToDeg = 57.2957795130823208768f;

enum class SafetyState : std::uint8_t { DISARMED, ARMED, ESTOP };
enum class RockAutomation : std::uint8_t { NONE, SWEEP, ALTERNATE };
enum class RockSweepPhase : std::uint8_t { MOVING, HOLDING };
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
    ROCK_ENTER,
    ROCK_CONFIRM,
    ROCK_CANCEL_ENTRY,
    ROCK_UP,
    ROCK_DOWN,
    ROCK_RIGHT,
    ROCK_LEFT,
    ROCK_TOGGLE_STEP,
    ROCK_ZERO,
    ROCK_SWEEP,
    ROCK_ALTERNATE,
    ROCK_RECORD,
    ROCK_VERIFY,
    ROCK_QUIT,
    ROCK_PERIOD_DOWN,
    ROCK_PERIOD_UP,
    ROCK_CANCEL_AUTOMATION,
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
std::atomic<bool> g_rock_mode{false};
std::atomic<bool> g_rock_ready{false};
std::atomic<bool> g_rock_confirmation_pending{false};
std::atomic<RockAutomation> g_rock_automation{RockAutomation::NONE};
float g_goal_rad[tilt::NUM_JOINTS]{};
float g_body_x_mm = 0.0f;
float g_body_height_mm = tilt::ZERO_POSE_HEIGHT_MM;
bool g_ik_coarse = false;
bool g_start_ik_when_idle = false;
bool g_start_rock_when_idle = false;

bool g_imu_available = false;
bool g_roll_valid = false;
float g_roll_deg = 0.0f;
std::int64_t g_last_imu_sample_us = 0;

float g_rock_delta_mm = 0.0f;
float g_rock_base_height_mm = tilt::ZERO_POSE_HEIGHT_MM;
float g_rock_max_abs_delta_mm = 0.0f;
bool g_rock_coarse = false;
std::uint32_t g_rock_alternate_period_ms =
    tilt_pose_test::kRockAlternatePeriodMs;
std::uint32_t g_rock_next_action_ms = 0;
float g_rock_alternate_amplitude_mm = 0.0f;
RockSweepPhase g_rock_sweep_phase = RockSweepPhase::MOVING;
float g_rock_sweep_max_delta_mm = 0.0f;
bool g_rock_sweep_roll_valid = false;
float g_rock_sweep_roll_deg = 0.0f;

struct FootLiftRecord {
    bool valid = false;
    float delta_mm = 0.0f;
    bool roll_valid = false;
    float roll_deg = 0.0f;
};

FootLiftRecord g_left_lift{};
FootLiftRecord g_right_lift{};

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

void serviceImu() {
    if (!g_imu_available) {
        return;
    }
    tilt::ImuRaw raw{};
    if (!tilt::imu_read_raw(raw)) {
        return;
    }
    const std::int64_t now_us = esp_timer_get_time();
    const float dt_s = g_last_imu_sample_us == 0
                           ? 0.0f
                           : static_cast<float>(now_us - g_last_imu_sample_us) /
                                 1'000'000.0f;
    g_last_imu_sample_us = now_us;
    const tilt::Attitude attitude = g_imu_filter.update(raw, dt_s);
    if (g_imu_filter.initialized() && std::isfinite(attitude.roll_rad)) {
        g_roll_deg = attitude.roll_rad * kRadToDeg;
        g_roll_valid = true;
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
    g_rock_mode.store(false);
    g_rock_ready.store(false);
    g_rock_confirmation_pending.store(false);
    g_rock_automation.store(RockAutomation::NONE);
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
    std::printf("state=%s  IK=%s  ROCK=%s\n", stateName(g_state.load()),
                g_ik_mode.load() ? "active" : "off",
                g_rock_mode.load() ? "active" : "off");
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
        "  joint <LHY|LHP|LKP|RHY|RHP|RKP> <-5..+5> | fk | ik | rock\n"
        "  In IK: arrows/WASD move, m fine/coarse, 0 home, v FK check, q exit.\n"
        "  In ROCK: arrows/WASD move, 1 sweep, 2 alternate, k record, v status, q exit.\n\n");
}

bool beginMove(const float target[tilt::NUM_JOINTS], std::uint32_t duration_ms,
               bool begin_ik_after_arrival = false,
               bool begin_rock_after_arrival = false) {
    if (!isArmed("move") || !targetWithinLogicalLimits(target)) {
        return false;
    }
    g_start_ik_when_idle = begin_ik_after_arrival;
    g_start_rock_when_idle = begin_rock_after_arrival;
    g_interpolator.start(g_goal_rad, target, duration_ms, nowMs());
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
    g_start_rock_when_idle = false;
    g_ik_mode.store(false);
    g_rock_mode.store(false);
    g_rock_ready.store(false);
    g_rock_confirmation_pending.store(false);
    g_rock_automation.store(RockAutomation::NONE);
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

void printRockState() {
    const float left_height = g_rock_base_height_mm - g_rock_delta_mm * 0.5f;
    const float right_height = g_rock_base_height_mm + g_rock_delta_mm * 0.5f;
    if (g_roll_valid) {
        std::printf("delta=%+.1f  base=%.2f  L=%.2f R=%.2f  roll=%+.1fdeg [%s]\n",
                    g_rock_delta_mm, g_rock_base_height_mm, left_height,
                    right_height, g_roll_deg, g_rock_coarse ? "COARSE" : "FINE");
    } else {
        std::printf("delta=%+.1f  base=%.2f  L=%.2f R=%.2f  roll=-- [%s]\n",
                    g_rock_delta_mm, g_rock_base_height_mm, left_height,
                    right_height, g_rock_coarse ? "COARSE" : "FINE");
    }
}

bool buildRockTarget(float delta_mm, float base_height_mm,
                     float target[tilt::NUM_JOINTS]) {
    const float left_height = base_height_mm - delta_mm * 0.5f;
    const float right_height = base_height_mm + delta_mm * 0.5f;
    const tilt::Vec3 left_target{0.0f, +tilt::Y_HIP_MM, -left_height};
    const tilt::Vec3 right_target{0.0f, -tilt::Y_HIP_MM, -right_height};
    tilt::IkResult left = tilt::ik_foot(tilt::Leg::LEFT, left_target, 0.0f);
    tilt::IkResult right = tilt::ik_foot(tilt::Leg::RIGHT, right_target, 0.0f);

    if (!left.reachable || !right.reachable) {
        std::printf("Rock target rejected: unreachable (%s%s%s).\n",
                    !left.reachable ? "LEFT" : "",
                    !left.reachable && !right.reachable ? "+" : "",
                    !right.reachable ? "RIGHT" : "");
        return false;
    }

    const float original_left[3] = {left.theta[0], left.theta[1], left.theta[2]};
    const float original_right[3] = {right.theta[0], right.theta[1], right.theta[2]};
    const bool left_unchanged = tilt::clamp_to_limits(tilt::Leg::LEFT, left.theta);
    const bool right_unchanged = tilt::clamp_to_limits(tilt::Leg::RIGHT, right.theta);
    if (!left_unchanged || !right_unchanged) {
        for (int joint = 0; joint < 3; ++joint) {
            if (left.theta[joint] != original_left[joint]) {
                std::printf("Rock target rejected: %s reached its joint limit.\n",
                            tilt::JOINT_NAME[joint]);
            }
            if (right.theta[joint] != original_right[joint]) {
                std::printf("Rock target rejected: %s reached its joint limit.\n",
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

bool applyRockTarget(float requested_delta_mm, float requested_base_height_mm,
                     std::uint32_t duration_ms, bool print_state = true) {
    const float delta_mm = std::fmax(-tilt_pose_test::kRockDeltaMaxMm,
                                     std::fmin(tilt_pose_test::kRockDeltaMaxMm,
                                               requested_delta_mm));
    const float base_height_mm = std::fmax(
        tilt_pose_test::kBodyHeightMinMm,
        std::fmin(tilt_pose_test::kBodyHeightMaxMm, requested_base_height_mm));
    if (delta_mm != requested_delta_mm || base_height_mm != requested_base_height_mm) {
        std::printf("Rock input clamped: delta=%+.1f base=%.2f\n",
                    delta_mm, base_height_mm);
    }

    float target[tilt::NUM_JOINTS]{};
    if (!buildRockTarget(delta_mm, base_height_mm, target) ||
        !beginMove(target, duration_ms)) {
        return false;
    }
    g_rock_delta_mm = delta_mm;
    g_rock_base_height_mm = base_height_mm;
    g_rock_max_abs_delta_mm = std::fmax(g_rock_max_abs_delta_mm,
                                        std::fabs(delta_mm));
    if (print_state) {
        printRockState();
    }
    return true;
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

void returnRockDeltaToZero() {
    if (g_rock_mode.load() && g_state.load() == SafetyState::ARMED) {
        applyRockTarget(0.0f, g_rock_base_height_mm,
                        tilt_pose_test::kRockStepDurationMs);
    }
}

void stopRockAutomation(const char* reason, bool return_to_zero) {
    if (g_rock_automation.exchange(RockAutomation::NONE) == RockAutomation::NONE) {
        return;
    }
    g_interpolator.abort();
    std::printf("Rock automation stopped: %s.\n", reason);
    if (return_to_zero) {
        returnRockDeltaToZero();
    }
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
    g_rock_alternate_period_ms = tilt_pose_test::kRockAlternatePeriodMs;
    g_left_lift = {};
    g_right_lift = {};
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
    g_rock_next_action_ms = nowMs() + g_rock_alternate_period_ms;
    std::printf("Alternating period=%lums.\n",
                static_cast<unsigned long>(g_rock_alternate_period_ms));
}

void quitRockMode() {
    if (!g_rock_mode.load()) {
        return;
    }
    g_rock_automation.store(RockAutomation::NONE);
    g_interpolator.abort();
    printRockSummary();
    g_rock_delta_mm = 0.0f;
    g_rock_base_height_mm = tilt::ZERO_POSE_HEIGHT_MM;
    g_rock_ready.store(false);
    g_rock_mode.store(false);
    beginMove(tilt::ZERO_POSE_RAD, tilt_pose_test::kPoseDurationMs);
    std::printf("Leaving rocking mode and returning home.\n");
}

void serviceRockAutomation() {
    const RockAutomation automation = g_rock_automation.load();
    if (automation == RockAutomation::NONE || !g_rock_ready.load() ||
        g_state.load() != SafetyState::ARMED) {
        return;
    }
    if (g_roll_valid && std::fabs(g_roll_deg) > tilt_pose_test::kRockRollAbortDeg) {
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
    if (!applyRockTarget(next_delta, g_rock_base_height_mm,
                         tilt_pose_test::kRockStepDurationMs)) {
        stopRockAutomation("alternating target rejected", true);
        return;
    }
    g_rock_next_action_ms = now + g_rock_alternate_period_ms;
}

void serviceMotion() {
    if (g_estop_requested.exchange(false)) {
        g_interpolator.abort();
        g_start_ik_when_idle = false;
        g_start_rock_when_idle = false;
        g_rock_automation.store(RockAutomation::NONE);
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
            ESP_LOGW(kTag, "Speed guard aborted move at %s (%+.3f deg step).",
                     tilt::JOINT_NAME[joint],
                     (next_goal[joint] - g_goal_rad[joint]) * kRadToDeg);
            g_interpolator.abort();
            g_start_ik_when_idle = false;
            g_start_rock_when_idle = false;
            if (g_rock_automation.load() != RockAutomation::NONE) {
                stopRockAutomation("joint speed guard", true);
            }
            return;
        }
    }
    if (!sendSixJointGoal(next_goal)) {
        emergencyStopNow();
        return;
    }
    std::memcpy(g_goal_rad, next_goal, sizeof(g_goal_rad));
    if (!still_moving) {
        if (g_rock_automation.load() == RockAutomation::NONE) {
            std::printf("Move complete.\n");
        }
        if (g_start_ik_when_idle) {
            g_start_ik_when_idle = false;
            g_ik_mode.store(true);
            std::printf("IK mode active: arrows/WASD move body, q exits, ! E-STOP.\n");
        }
        if (g_start_rock_when_idle) {
            g_start_rock_when_idle = false;
            g_rock_ready.store(true);
            std::printf("Rocking mode active on floor: arrows/WASD adjust, "
                        "1 sweep, 2 alternate, k records lift, q exits, ! E-STOP.\n");
            printRockState();
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
        case CommandType::ROCK_ZERO:
            if (g_rock_ready.load()) {
                applyRockTarget(0.0f, g_rock_base_height_mm,
                                tilt_pose_test::kRockStepDurationMs);
            }
            return;
        case CommandType::ROCK_SWEEP: startRockSweep(); return;
        case CommandType::ROCK_ALTERNATE: startRockAlternate(); return;
        case CommandType::ROCK_RECORD: recordFootLift(); return;
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
    else if (strcasecmp(operation, "rock") == 0) enqueue({CommandType::ROCK_ENTER});
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

void enqueueRockKey(std::uint8_t key) {
    CommandType type;
    switch (key) {
        case 'w': case 'W': type = CommandType::ROCK_UP; break;
        case 's': case 'S': type = CommandType::ROCK_DOWN; break;
        case 'd': case 'D': type = CommandType::ROCK_RIGHT; break;
        case 'a': case 'A': type = CommandType::ROCK_LEFT; break;
        case 'm': case 'M': type = CommandType::ROCK_TOGGLE_STEP; break;
        case '0': type = CommandType::ROCK_ZERO; break;
        case '1': type = CommandType::ROCK_SWEEP; break;
        case '2': type = CommandType::ROCK_ALTERNATE; break;
        case 'k': case 'K': type = CommandType::ROCK_RECORD; break;
        case 'v': case 'V': type = CommandType::ROCK_VERIFY; break;
        case 'q': case 'Q': type = CommandType::ROCK_QUIT; break;
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

        if (g_rock_mode.load()) {
            const RockAutomation automation = g_rock_automation.load();
            if (automation == RockAutomation::SWEEP) {
                if (byte == 'k' || byte == 'K') {
                    enqueue({CommandType::ROCK_RECORD});
                }
                enqueue({CommandType::ROCK_CANCEL_AUTOMATION});
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
                } else {
                    if (byte == 'k' || byte == 'K') {
                        enqueue({CommandType::ROCK_RECORD});
                    }
                    enqueue({CommandType::ROCK_CANCEL_AUTOMATION});
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
        ESP_LOGW(kTag, "MPU6050 unavailable; rocking continues with roll=--.");
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
        Command command{};
        while (xQueueReceive(g_command_queue, &command, 0) == pdTRUE) {
            handleCommand(command);
        }
        serviceMotion();
        serviceRockAutomation();
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(tilt::CONTROL_PERIOD_MS));
    }
}
