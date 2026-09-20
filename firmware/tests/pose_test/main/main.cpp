#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <strings.h>

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "tilt/sts3215/Sts3215Bus.h"
#include "tilt_config.h"
#include "tilt_kinematics.h"

#include "Interpolator.h"
#include "JointMapper.h"
#include "PoseTestConfig.h"

namespace {

constexpr char kTag[] = "pose_test";
constexpr std::size_t kConsoleLineCapacity = 96;
constexpr UBaseType_t kCommandQueueDepth = 20;
constexpr float kRadToDeg = 57.2957795130823208768f;

enum class SafetyState : std::uint8_t { DISARMED, ARMED, ESTOP };
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
QueueHandle_t g_command_queue = nullptr;
std::atomic<SafetyState> g_state{SafetyState::DISARMED};
std::atomic<bool> g_estop_requested{false};
std::atomic<bool> g_ik_mode{false};
float g_goal_rad[tilt::NUM_JOINTS]{};
float g_body_x_mm = 0.0f;
float g_body_height_mm = tilt::ZERO_POSE_HEIGHT_MM;
bool g_ik_coarse = false;
bool g_start_ik_when_idle = false;

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
    std::printf("state=%s  IK=%s\n", stateName(g_state.load()),
                g_ik_mode.load() ? "active" : "off");
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
        "  joint <LHY|LHP|LKP|RHY|RHP|RKP> <-5..+5> | fk | ik\n"
        "  In IK: arrows/WASD move, m fine/coarse, 0 home, v FK check, q exit.\n\n");
}

bool beginMove(const float target[tilt::NUM_JOINTS], std::uint32_t duration_ms,
               bool begin_ik_after_arrival = false) {
    if (!isArmed("move") || !targetWithinLogicalLimits(target)) {
        return false;
    }
    g_start_ik_when_idle = begin_ik_after_arrival;
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
    g_ik_mode.store(false);
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

void serviceMotion() {
    if (g_estop_requested.exchange(false)) {
        g_interpolator.abort();
        g_start_ik_when_idle = false;
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
            return;
        }
    }
    if (!sendSixJointGoal(next_goal)) {
        emergencyStopNow();
        return;
    }
    std::memcpy(g_goal_rad, next_goal, sizeof(g_goal_rad));
    if (!still_moving) {
        std::printf("Move complete.\n");
        if (g_start_ik_when_idle) {
            g_start_ik_when_idle = false;
            g_ik_mode.store(true);
            std::printf("IK mode active: arrows/WASD move body, q exits, ! E-STOP.\n");
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

    // Boot invariant: never move automatically and always request torque OFF.
    torqueOffBestEffort();
    std::printf("\nBoot complete: state=DISARMED, torque OFF, no automatic home move.\n");

    g_command_queue = xQueueCreate(kCommandQueueDepth, sizeof(Command));
    if (g_command_queue == nullptr) {
        ESP_LOGE(kTag, "Could not create console command queue.");
        return;
    }
    xTaskCreate(consoleTask, "pose_console", 4096, nullptr, 5, nullptr);

    TickType_t wake_time = xTaskGetTickCount();
    while (true) {
        Command command{};
        while (xQueueReceive(g_command_queue, &command, 0) == pdTRUE) {
            handleCommand(command);
        }
        serviceMotion();
        vTaskDelayUntil(&wake_time, pdMS_TO_TICKS(tilt::CONTROL_PERIOD_MS));
    }
}
