#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
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
#include "tilt_mpu6050.h"

#include "Interpolation.h"
#include "WalkConfig.h"

namespace {

constexpr char kTag[] = "walk_test";
constexpr float kRadToDeg = 57.2957795131f;
constexpr std::size_t kLineCapacity = 96;
constexpr UBaseType_t kQueueDepth = 20;
constexpr int kLeft = 0;
constexpr int kRight = 1;

enum class SafetyState : std::uint8_t { DISARMED, ARMED, ESTOP };
enum class CommandType : std::uint8_t {
    HELP,
    STATUS,
    ARM,
    DISARM,
    RECOVER,
    HOME,
    STAND,
    JOINT,
    WALK_ENTER,
    WALK_TOGGLE,
    WALK_LEGX_UP,
    WALK_LEGX_DOWN,
    WALK_HEIGHT_UP,
    WALK_HEIGHT_DOWN,
    WALK_DIFF_UP,
    WALK_DIFF_DOWN,
    WALK_STEP_DOWN,
    WALK_STEP_UP,
    WALK_LEG_DOWN,
    WALK_LEG_UP,
    WALK_IMU_TOGGLE,
    WALK_ZERO_LEGX,
    WALK_RESET_ATTITUDE,
    WALK_STATUS,
    WALK_QUIT,
};

enum class MoveCompletion : std::uint8_t { NONE, WALK_READY };

struct Command {
    CommandType type;
    char name[16]{};
    float value = 0.0f;
};

struct JointMove {
    float from[tilt::NUM_JOINTS]{};
    float to[tilt::NUM_JOINTS]{};
    std::uint32_t started_ms = 0;
    std::uint32_t duration_ms = 1;
    MoveCompletion completion = MoveCompletion::NONE;
    bool active = false;
};

struct WalkStats {
    std::uint32_t cycles = 0;
    std::uint32_t rejected_targets = 0;
    std::uint32_t speed_guard_count = 0;
    bool started = false;
    bool roll_valid = false;
    float roll_min_deg = 0.0f;
    float roll_max_deg = 0.0f;
    bool pitch_valid = false;
    float pitch_min_deg = 0.0f;
    float pitch_max_deg = 0.0f;
    float last_legx_mm = tilt_walk_test::kLegxDefaultMm;
    float last_long_mm = tilt_walk_test::kLongLegDefaultMm;
    float last_short_mm = tilt_walk_test::kShortLegDefaultMm;
    std::uint32_t last_step_ms = tilt_walk_test::kStepTimeDefaultMs;
    std::uint32_t last_leg_ms = tilt_walk_test::kLegTimeDefaultMs;
    bool last_imu_modulation = false;
};

constexpr tilt::sts3215::BusConfig kBusConfig{
    static_cast<uart_port_t>(tilt::SERVO_UART_PORT),
    static_cast<gpio_num_t>(tilt::SERVO_UART_TX_PIN),
    static_cast<gpio_num_t>(tilt::SERVO_UART_RX_PIN),
    tilt::SERVO_UART_BAUD,
    50,
};

tilt::sts3215::Sts3215Bus g_bus(kBusConfig);
tilt::ComplementaryFilter g_imu_filter;
QueueHandle_t g_command_queue = nullptr;
std::atomic<SafetyState> g_safety{SafetyState::DISARMED};
std::atomic<bool> g_walk_mode{false};
std::atomic<bool> g_walk_ready{false};
std::atomic<bool> g_walk_running{false};

float g_goal_rad[tilt::NUM_JOINTS]{};
bool g_goal_valid = false;
JointMove g_joint_move{};
bool g_speed_guard_latched = false;

bool g_imu_available = false;
bool g_imu_modulation = false;
bool g_roll_valid = false;
bool g_pitch_valid = false;
float g_roll_deg = 0.0f;
float g_pitch_deg = 0.0f;
bool g_attitude_zero_valid = false;
float g_roll_zero_deg = 0.0f;
float g_pitch_zero_deg = 0.0f;
std::uint32_t g_last_imu_ms = 0;
std::uint32_t g_imu_failures = 0;

float g_long_leg_mm = tilt_walk_test::kLongLegDefaultMm;
float g_short_leg_mm = tilt_walk_test::kShortLegDefaultMm;
float g_legx_mm = tilt_walk_test::kLegxDefaultMm;
std::uint32_t g_step_time_ms = tilt_walk_test::kStepTimeDefaultMs;
std::uint32_t g_leg_time_ms = tilt_walk_test::kLegTimeDefaultMs;

bool g_leg_trigger[2]{};
std::uint8_t g_leg_state[2]{};
float g_leg_x_mm[2]{};
float g_leg_z_mm[2]{tilt_walk_test::kLongLegDefaultMm,
                    tilt_walk_test::kLongLegDefaultMm};
std::uint32_t g_prev_step_ms = 0;
std::uint32_t g_prev_leg_ms[2]{};
std::uint8_t g_step_count = 0;
bool g_leg_completed[2]{};
tilt_walk_test::Interpolation g_x_interpolation[2];
tilt_walk_test::Interpolation g_z_interpolation[2];
float g_interpolated_x_mm[2]{};
float g_interpolated_z_mm[2]{tilt_walk_test::kLongLegDefaultMm,
                             tilt_walk_test::kLongLegDefaultMm};
bool g_rejection_latched = false;
WalkStats g_stats{};

void emergencyStopNow();

std::uint32_t nowMs() {
    return static_cast<std::uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

const char* safetyName(SafetyState state) {
    switch (state) {
        case SafetyState::DISARMED: return "DISARMED";
        case SafetyState::ARMED: return "ARMED";
        case SafetyState::ESTOP: return "ESTOP";
    }
    return "UNKNOWN";
}

bool busOk(esp_err_t result, const char* operation) {
    if (result == ESP_OK) return true;
    ESP_LOGE(kTag, "%s failed: %s", operation, esp_err_to_name(result));
    return false;
}

std::uint16_t radToTick(int joint, float rad) {
    const float signed_ticks = rad / tilt::RAD_PER_TICK *
                               static_cast<float>(tilt::JOINT_SIGN[joint]);
    const int tick = static_cast<int>(std::lround(signed_ticks)) +
                     static_cast<int>(tilt::ZERO_TICK[joint]);
    return static_cast<std::uint16_t>(std::clamp(
        tick, tilt::SERVO_POS_MIN, tilt::SERVO_POS_MAX));
}

float tickToRad(int joint, std::uint16_t tick) {
    return static_cast<float>(static_cast<int>(tick) -
                              static_cast<int>(tilt::ZERO_TICK[joint])) *
           tilt::RAD_PER_TICK * static_cast<float>(tilt::JOINT_SIGN[joint]);
}

bool readPositions(std::uint16_t raw[tilt::NUM_JOINTS]) {
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const esp_err_t result =
            g_bus.readPosition(tilt::SERVO_ID[joint], raw[joint]);
        if (result != ESP_OK) {
            ESP_LOGE(kTag, "read %s failed: %s", tilt::JOINT_NAME[joint],
                     esp_err_to_name(result));
            return false;
        }
    }
    return true;
}

bool targetWithinLimits(const float target[tilt::NUM_JOINTS], bool verbose) {
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const auto& limit = tilt::JOINT_LIMIT[joint / 3][joint % 3];
        if (!std::isfinite(target[joint]) || target[joint] < limit.minimum_rad ||
            target[joint] > limit.maximum_rad) {
            if (verbose) {
                std::printf("%s target %+.2fdeg rejected (limit %+.2f..%+.2fdeg).\n",
                            tilt::JOINT_NAME[joint], target[joint] * kRadToDeg,
                            limit.minimum_rad * kRadToDeg,
                            limit.maximum_rad * kRadToDeg);
            }
            return false;
        }
    }
    return true;
}

void stopWalkForSpeedGuard() {
    g_walk_running.store(false);
    g_legx_mm = 0.0f;
    g_leg_trigger[kLeft] = g_leg_trigger[kRight] = false;
    g_leg_state[kLeft] = g_leg_state[kRight] = 0;
    std::printf("Walk stopped: joint speed guard. Last transmitted goal is held.\n");
}

bool transmitGoal(const float target[tilt::NUM_JOINTS]) {
    if (g_safety.load() != SafetyState::ARMED) return false;

    if (g_goal_valid) {
        for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
            const float step = target[joint] - g_goal_rad[joint];
            if (std::fabs(step) > tilt_walk_test::kMaxJointStepRad) {
                const bool first_guard_event = !g_speed_guard_latched;
                if (first_guard_event) {
                    ESP_LOGW(kTag, "Speed guard blocked %s (%+.3fdeg/10ms).",
                             tilt::JOINT_NAME[joint], step * kRadToDeg);
                    ++g_stats.speed_guard_count;
                }
                g_speed_guard_latched = true;
                g_joint_move.active = false;
                if (g_walk_mode.load() && first_guard_event) {
                    stopWalkForSpeedGuard();
                }
                return false;
            }
        }
    }

    std::uint16_t ticks[tilt::NUM_JOINTS]{};
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        ticks[joint] = radToTick(joint, target[joint]);
    }
    if (!busOk(g_bus.syncWritePositions(tilt::SERVO_ID, ticks,
                                         tilt::NUM_JOINTS,
                                         tilt_walk_test::kServoSpeedRaw),
               "syncWritePositions")) {
        emergencyStopNow();
        return false;
    }
    std::memcpy(g_goal_rad, target, sizeof(g_goal_rad));
    g_goal_valid = true;
    g_speed_guard_latched = false;
    return true;
}

void torqueOffBestEffort() {
    const esp_err_t result =
        g_bus.setTorqueAll(tilt::SERVO_ID, tilt::NUM_JOINTS, false);
    if (result != ESP_OK) {
        ESP_LOGE(kTag, "torque OFF failed: %s; cut servo power now",
                 esp_err_to_name(result));
    }
}

void emergencyStopNow() {
    g_walk_running.store(false);
    g_walk_ready.store(false);
    g_walk_mode.store(false);
    g_joint_move.active = false;
    g_imu_modulation = false;
    g_safety.store(SafetyState::ESTOP);
    const esp_err_t result =
        g_bus.emergencyStop(tilt::SERVO_ID, tilt::NUM_JOINTS);
    if (result != ESP_OK) {
        ESP_LOGE(kTag, "E-STOP torque OFF failed: %s; cut servo power now",
                 esp_err_to_name(result));
    }
    std::printf("\n!! E-STOP: torque OFF requested. Cut servo power if needed. !!\n");
}

bool isArmed(const char* command) {
    if (g_safety.load() == SafetyState::ARMED) return true;
    std::printf("%s requires ARMED (current %s).\n", command,
                safetyName(g_safety.load()));
    return false;
}

void arm() {
    if (g_safety.load() != SafetyState::DISARMED) {
        std::printf("arm requires DISARMED (current %s).\n",
                    safetyName(g_safety.load()));
        return;
    }
    std::uint16_t ticks[tilt::NUM_JOINTS]{};
    if (!readPositions(ticks)) {
        std::printf("arm rejected: could not read all six present positions.\n");
        return;
    }
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        g_goal_rad[joint] = tickToRad(joint, ticks[joint]);
    }
    g_goal_valid = true;

    // Capture and write the actual pose before torque ON. This prevents an old
    // register goal from pulling the robot when torque is restored.
    if (!busOk(g_bus.syncWritePositions(tilt::SERVO_ID, ticks,
                                         tilt::NUM_JOINTS,
                                         tilt_walk_test::kServoSpeedRaw),
               "arm hold target")) {
        return;
    }
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        if (!busOk(g_bus.setAcceleration(
                       tilt::SERVO_ID[joint],
                       tilt_walk_test::kServoAcceleration),
                   "setAcceleration")) {
            torqueOffBestEffort();
            return;
        }
    }
    if (!busOk(g_bus.setTorqueAll(tilt::SERVO_ID, tilt::NUM_JOINTS, true),
               "torque ON")) {
        torqueOffBestEffort();
        return;
    }
    g_speed_guard_latched = false;
    g_safety.store(SafetyState::ARMED);
    std::printf("ARMED: present pose held. '!' is E-STOP.\n");
}

void disarm() {
    if (g_safety.load() == SafetyState::ESTOP) {
        std::printf("Use recover first; ESTOP remains latched.\n");
        return;
    }
    g_walk_running.store(false);
    g_walk_ready.store(false);
    g_walk_mode.store(false);
    g_joint_move.active = false;
    g_imu_modulation = false;
    torqueOffBestEffort();
    g_safety.store(SafetyState::DISARMED);
    std::printf("DISARMED: torque OFF requested.\n");
}

void recover() {
    if (g_safety.load() != SafetyState::ESTOP) {
        std::printf("recover requires ESTOP.\n");
        return;
    }
    torqueOffBestEffort();
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        if (g_bus.ping(tilt::SERVO_ID[joint]) != ESP_OK) {
            std::printf("recover failed: %s did not respond.\n",
                        tilt::JOINT_NAME[joint]);
            return;
        }
    }
    g_safety.store(SafetyState::DISARMED);
    g_speed_guard_latched = false;
    std::printf("Recovered to DISARMED. Inspect the robot, then run arm.\n");
}

void serviceImu(std::uint32_t now) {
    if (!g_imu_available || now - g_last_imu_ms < tilt::IMU_SAMPLE_PERIOD_MS) {
        return;
    }
    const float dt_s = g_last_imu_ms == 0
                           ? tilt::IMU_SAMPLE_PERIOD_MS / 1000.0f
                           : (now - g_last_imu_ms) / 1000.0f;
    g_last_imu_ms = now;
    tilt::ImuRaw raw{};
    if (!tilt::imu_read_raw(raw)) {
        ++g_imu_failures;
        if (g_imu_failures >= 5) {
            g_imu_available = false;
            g_imu_modulation = false;
            ESP_LOGW(kTag, "MPU6050 reads failed; timing modulation OFF, walk continues.");
        }
        return;
    }
    g_imu_failures = 0;
    const tilt::Attitude attitude = g_imu_filter.update(raw, dt_s);
    if (!g_imu_filter.initialized()) return;
    if (std::isfinite(attitude.roll_rad)) {
        g_roll_deg = attitude.roll_rad * kRadToDeg;
        g_roll_valid = true;
    }
    if (std::isfinite(attitude.pitch_rad)) {
        g_pitch_deg = attitude.pitch_rad * kRadToDeg;
        g_pitch_valid = true;
    }
}

void effectiveTimes(std::uint32_t& step_ms, std::uint32_t& leg_ms) {
    step_ms = g_step_time_ms;
    leg_ms = g_leg_time_ms;
    if (!g_imu_modulation || !g_imu_available || !g_pitch_valid) return;
    const float magnitude = std::fabs(g_pitch_deg);
    step_ms = static_cast<std::uint32_t>(std::lround(std::clamp(
        tilt_walk_test::kStepTimeDefaultMs +
            magnitude * tilt_walk_test::kStepTimePitchGain,
        static_cast<float>(tilt_walk_test::kStepTimeModMinMs),
        static_cast<float>(tilt_walk_test::kStepTimeModMaxMs))));
    leg_ms = static_cast<std::uint32_t>(std::lround(std::clamp(
        tilt_walk_test::kLegTimeDefaultMs -
            magnitude * tilt_walk_test::kLegTimePitchGain,
        static_cast<float>(tilt_walk_test::kLegTimeModMinMs),
        static_cast<float>(tilt_walk_test::kLegTimeModMaxMs))));
}

bool solveFeet(float left_x, float left_z, float right_x, float right_z,
               float target[tilt::NUM_JOINTS]) {
    const float xs[2] = {left_x, right_x};
    const float zs[2] = {left_z, right_z};
    for (int leg = 0; leg < 2; ++leg) {
        const tilt::Leg side = leg == kLeft ? tilt::Leg::LEFT : tilt::Leg::RIGHT;
        const float hip_y = leg == kLeft ? tilt::Y_HIP_MM : -tilt::Y_HIP_MM;
        const tilt::IkResult result =
            tilt::ik_foot(side, {xs[leg], hip_y, -zs[leg]}, 0.0f);
        if (!result.reachable) return false;
        float checked[3] = {result.theta[0], result.theta[1], result.theta[2]};
        if (!tilt::clamp_to_limits(side, checked)) return false;
        for (int joint = 0; joint < 3; ++joint) {
            target[leg * 3 + joint] = result.theta[joint];
        }
    }
    return targetWithinLimits(target, false);
}

bool beginJointMove(const float target[tilt::NUM_JOINTS],
                    std::uint32_t duration_ms,
                    MoveCompletion completion = MoveCompletion::NONE) {
    if (!isArmed("move") || !targetWithinLimits(target, true)) return false;
    std::memcpy(g_joint_move.from, g_goal_rad, sizeof(g_joint_move.from));
    std::memcpy(g_joint_move.to, target, sizeof(g_joint_move.to));
    g_joint_move.started_ms = nowMs();
    g_joint_move.duration_ms = std::max<std::uint32_t>(duration_ms, 1);
    g_joint_move.completion = completion;
    g_joint_move.active = true;
    return true;
}

void resetAttitudeZero() {
    if (!g_roll_valid || !g_pitch_valid) {
        g_attitude_zero_valid = false;
        std::printf("IMU attitude is not available; display zero not changed.\n");
        return;
    }
    g_roll_zero_deg = g_roll_deg;
    g_pitch_zero_deg = g_pitch_deg;
    g_attitude_zero_valid = true;
    std::printf("IMU display zero reset: roll=%+.1f pitch=%+.1fdeg.\n",
                g_roll_zero_deg, g_pitch_zero_deg);
}

void resetWalkInterpolation(std::uint32_t now) {
    for (int leg = 0; leg < 2; ++leg) {
        g_leg_trigger[leg] = false;
        g_leg_state[leg] = 0;
        g_leg_x_mm[leg] = 0.0f;
        g_leg_z_mm[leg] = g_long_leg_mm;
        g_prev_leg_ms[leg] = now;
        g_leg_completed[leg] = false;
        g_x_interpolation[leg].reset(0.0f, now);
        g_z_interpolation[leg].reset(g_long_leg_mm, now);
        g_interpolated_x_mm[leg] = 0.0f;
        g_interpolated_z_mm[leg] = g_long_leg_mm;
    }
    g_prev_step_ms = now;
    g_step_count = 0;
    g_rejection_latched = false;
}

void finishJointMove() {
    const MoveCompletion completion = g_joint_move.completion;
    g_joint_move.active = false;
    g_joint_move.completion = MoveCompletion::NONE;
    std::printf("Move complete.\n");
    if (completion == MoveCompletion::WALK_READY) {
        resetWalkInterpolation(nowMs());
        if (g_roll_valid && g_pitch_valid) {
            g_roll_zero_deg = g_roll_deg;
            g_pitch_zero_deg = g_pitch_deg;
            g_attitude_zero_valid = true;
        }
        g_walk_ready.store(true);
        std::printf("Walk mode ready and STOPPED. Press space to start; ! E-STOPS.\n");
    }
}

void serviceJointMove(std::uint32_t now) {
    if (!g_joint_move.active || g_safety.load() != SafetyState::ARMED) return;
    const std::uint32_t elapsed = now - g_joint_move.started_ms;
    const float t = std::min(1.0f, static_cast<float>(elapsed) /
                                      g_joint_move.duration_ms);
    float next[tilt::NUM_JOINTS]{};
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        next[joint] = g_joint_move.from[joint] +
                      (g_joint_move.to[joint] - g_joint_move.from[joint]) * t;
    }
    if (!transmitGoal(next)) return;
    if (t >= 1.0f) finishJointMove();
}

void sampleWalkStats() {
    if (!g_walk_running.load()) return;
    g_stats.started = true;
    g_stats.last_legx_mm = g_legx_mm;
    g_stats.last_long_mm = g_long_leg_mm;
    g_stats.last_short_mm = g_short_leg_mm;
    g_stats.last_step_ms = g_step_time_ms;
    g_stats.last_leg_ms = g_leg_time_ms;
    g_stats.last_imu_modulation = g_imu_modulation;
    if (g_roll_valid) {
        if (!g_stats.roll_valid) {
            g_stats.roll_min_deg = g_stats.roll_max_deg = g_roll_deg;
            g_stats.roll_valid = true;
        } else {
            g_stats.roll_min_deg = std::min(g_stats.roll_min_deg, g_roll_deg);
            g_stats.roll_max_deg = std::max(g_stats.roll_max_deg, g_roll_deg);
        }
    }
    if (g_pitch_valid) {
        if (!g_stats.pitch_valid) {
            g_stats.pitch_min_deg = g_stats.pitch_max_deg = g_pitch_deg;
            g_stats.pitch_valid = true;
        } else {
            g_stats.pitch_min_deg = std::min(g_stats.pitch_min_deg, g_pitch_deg);
            g_stats.pitch_max_deg = std::max(g_stats.pitch_max_deg, g_pitch_deg);
        }
    }
}

void printCycle(std::uint32_t step_ms, std::uint32_t leg_ms) {
    std::printf("cyc#%lu legx=%+.1f long=%.1f short=%.1f step=%lu leg=%lu | "
                "L(x%+.1f z%.1f s%u) R(x%+.1f z%.1f s%u) | ",
                static_cast<unsigned long>(g_stats.cycles), g_legx_mm,
                g_long_leg_mm, g_short_leg_mm,
                static_cast<unsigned long>(step_ms),
                static_cast<unsigned long>(leg_ms),
                g_interpolated_x_mm[kLeft], g_interpolated_z_mm[kLeft],
                g_leg_state[kLeft], g_interpolated_x_mm[kRight],
                g_interpolated_z_mm[kRight], g_leg_state[kRight]);
    if (g_roll_valid) std::printf("roll=%+.1f ", g_roll_deg);
    else std::printf("roll=-- ");
    if (g_pitch_valid) std::printf("pitch=%+.1f\n", g_pitch_deg);
    else std::printf("pitch=--\n");
}

void updateGaitState(std::uint32_t now, std::uint32_t step_ms,
                     std::uint32_t leg_ms) {
    if (now - g_prev_step_ms >= step_ms) {
        if (g_step_count == 0) {
            g_step_count = 1;
            g_leg_trigger[kLeft] = true;
        } else {
            g_step_count = 0;
            g_leg_trigger[kRight] = true;
        }
        g_prev_step_ms = now;
    }

    for (int leg = 0; leg < 2; ++leg) {
        if (g_leg_trigger[leg] && g_leg_state[leg] == 0 &&
            now - g_prev_leg_ms[leg] >= leg_ms) {
            g_leg_z_mm[leg] = g_short_leg_mm;  // z only
            g_leg_trigger[leg] = false;
            g_leg_state[leg] = 1;
            g_prev_leg_ms[leg] = now;
        } else if (g_leg_state[leg] == 1 &&
                   now - g_prev_leg_ms[leg] >= leg_ms) {
            g_leg_x_mm[leg] = -g_legx_mm;      // x only
            g_leg_state[leg] = 2;
            g_prev_leg_ms[leg] = now;
        } else if (g_leg_state[leg] == 2 &&
                   now - g_prev_leg_ms[leg] >= leg_ms) {
            g_leg_z_mm[leg] = g_long_leg_mm;   // z only
            g_leg_state[leg] = 3;
            g_prev_leg_ms[leg] = now;
        } else if (g_leg_state[leg] == 3 &&
                   now - g_prev_leg_ms[leg] >= leg_ms) {
            g_leg_x_mm[leg] = +g_legx_mm;      // x only
            g_leg_state[leg] = 0;
            g_prev_leg_ms[leg] = now;
            g_leg_completed[leg] = true;
        }
    }
}

void holdLastGoalAfterGuard(std::uint32_t now) {
    for (int leg = 0; leg < 2; ++leg) {
        float joints[3] = {g_goal_rad[leg * 3], g_goal_rad[leg * 3 + 1],
                           g_goal_rad[leg * 3 + 2]};
        const tilt::Vec3 foot = tilt::fk_foot(
            leg == kLeft ? tilt::Leg::LEFT : tilt::Leg::RIGHT, joints);
        g_leg_x_mm[leg] = foot.x;
        g_leg_z_mm[leg] = -foot.z;
        g_x_interpolation[leg].reset(foot.x, now);
        g_z_interpolation[leg].reset(-foot.z, now);
    }
}

void serviceWalk(std::uint32_t now) {
    if (!g_walk_mode.load() || !g_walk_ready.load() ||
        g_safety.load() != SafetyState::ARMED || g_joint_move.active) {
        return;
    }

    std::uint32_t step_ms = 0;
    std::uint32_t leg_ms = 0;
    effectiveTimes(step_ms, leg_ms);
    if (g_walk_running.load()) updateGaitState(now, step_ms, leg_ms);

    for (int leg = 0; leg < 2; ++leg) {
        g_interpolated_x_mm[leg] =
            g_x_interpolation[leg].go(g_leg_x_mm[leg], leg_ms, now);
        g_interpolated_z_mm[leg] =
            g_z_interpolation[leg].go(g_leg_z_mm[leg], leg_ms, now);
    }

    const float bias = g_legx_mm * tilt_walk_test::kStanceBiasRatio;
    float target[tilt::NUM_JOINTS]{};
    if (!solveFeet(g_interpolated_x_mm[kLeft] + bias,
                   g_interpolated_z_mm[kLeft],
                   g_interpolated_x_mm[kRight] + bias,
                   g_interpolated_z_mm[kRight], target)) {
        ++g_stats.rejected_targets;
        if (!g_rejection_latched) {
            ESP_LOGW(kTag, "Unreachable or joint-limit gait target rejected; holding last goal.");
        }
        g_rejection_latched = true;
        sampleWalkStats();
        return;
    }
    g_rejection_latched = false;
    if (!transmitGoal(target)) {
        if (g_speed_guard_latched) holdLastGoalAfterGuard(now);
        return;
    }

    sampleWalkStats();
    if (g_leg_completed[kLeft] && g_leg_completed[kRight]) {
        g_leg_completed[kLeft] = g_leg_completed[kRight] = false;
        ++g_stats.cycles;
        printCycle(step_ms, leg_ms);
    }
}

void startWalk() {
    if (!g_walk_ready.load() || !isArmed("walk start")) return;
    if (g_walk_running.exchange(true)) return;
    const std::uint32_t now = nowMs();
    g_prev_step_ms = now;
    g_prev_leg_ms[kLeft] = g_prev_leg_ms[kRight] = now;
    g_step_count = 0;
    g_leg_trigger[kLeft] = g_leg_trigger[kRight] = false;
    g_leg_state[kLeft] = g_leg_state[kRight] = 0;
    g_leg_completed[kLeft] = g_leg_completed[kRight] = false;
    std::printf("Walk RUNNING. Space stops; ! E-STOPS.\n");
}

void stopWalk() {
    if (!g_walk_running.exchange(false)) return;
    g_stats.last_legx_mm = g_legx_mm;
    g_stats.last_long_mm = g_long_leg_mm;
    g_stats.last_short_mm = g_short_leg_mm;
    g_stats.last_step_ms = g_step_time_ms;
    g_stats.last_leg_ms = g_leg_time_ms;
    g_stats.last_imu_modulation = g_imu_modulation;
    g_legx_mm = 0.0f;
    for (int leg = 0; leg < 2; ++leg) {
        g_leg_trigger[leg] = false;
        g_leg_state[leg] = 0;
        g_leg_x_mm[leg] = 0.0f;
        g_leg_z_mm[leg] = g_long_leg_mm;
        g_leg_completed[leg] = false;
    }
    std::printf("Walk STOPPED: legx=0, both feet returning to long_leg.\n");
}

void printWalkStatus() {
    std::uint32_t effective_step = 0;
    std::uint32_t effective_leg = 0;
    effectiveTimes(effective_step, effective_leg);
    const char* mode = g_walk_running.load() ? "running" : "stopped";
    std::printf("[WALK %s] legx=%+.1f  long=%.1f short=%.1f (diff %.1f)  base~%.1f\n",
                mode, g_legx_mm, g_long_leg_mm, g_short_leg_mm,
                g_long_leg_mm - g_short_leg_mm,
                (g_long_leg_mm + g_short_leg_mm) * 0.5f);
    std::printf("  stepTime=%lums  legTime=%lums  effective=%lu/%lums  IMU modulation=%s\n",
                static_cast<unsigned long>(g_step_time_ms),
                static_cast<unsigned long>(g_leg_time_ms),
                static_cast<unsigned long>(effective_step),
                static_cast<unsigned long>(effective_leg),
                g_imu_modulation ? "ON" : "OFF");
    std::printf("  extension: long %.1f%%, short %.1f%% (maximum reach %.1fmm)\n",
                g_long_leg_mm / tilt_walk_test::kMaximumReachMm * 100.0f,
                g_short_leg_mm / tilt_walk_test::kMaximumReachMm * 100.0f,
                tilt_walk_test::kMaximumReachMm);
    std::printf("  IMU ");
    if (g_roll_valid) {
        std::printf("roll=%+.1f", g_roll_deg);
        if (g_attitude_zero_valid) {
            std::printf(" (rel %+.1f)", g_roll_deg - g_roll_zero_deg);
        }
    } else {
        std::printf("roll=--");
    }
    if (g_pitch_valid) {
        std::printf("  pitch=%+.1f", g_pitch_deg);
        if (g_attitude_zero_valid) {
            std::printf(" (rel %+.1f)", g_pitch_deg - g_pitch_zero_deg);
        }
    } else {
        std::printf("  pitch=--");
    }
    std::printf("\n  rejected targets: %lu\n",
                static_cast<unsigned long>(g_stats.rejected_targets));
}

void printWalkSummary() {
    const float legx = g_stats.started ? g_stats.last_legx_mm : g_legx_mm;
    const float long_mm = g_stats.started ? g_stats.last_long_mm : g_long_leg_mm;
    const float short_mm = g_stats.started ? g_stats.last_short_mm : g_short_leg_mm;
    const std::uint32_t step_ms =
        g_stats.started ? g_stats.last_step_ms : g_step_time_ms;
    const std::uint32_t leg_ms =
        g_stats.started ? g_stats.last_leg_ms : g_leg_time_ms;
    const bool modulation =
        g_stats.started ? g_stats.last_imu_modulation : g_imu_modulation;
    std::printf("\n-- walk result --\n");
    std::printf("  legx %.1fmm, long %.1f short %.1f, stepTime %lu legTime %lu, %lu cycles\n",
                legx, long_mm, short_mm,
                static_cast<unsigned long>(step_ms),
                static_cast<unsigned long>(leg_ms),
                static_cast<unsigned long>(g_stats.cycles));
    std::printf("  IMU modulation: %s\n", modulation ? "ON" : "OFF");
    if (g_stats.roll_valid) {
        std::printf("  roll range:  %+.1f ~ %+.1f deg (p-p %.1f)\n",
                    g_stats.roll_min_deg, g_stats.roll_max_deg,
                    g_stats.roll_max_deg - g_stats.roll_min_deg);
    } else {
        std::printf("  roll range:  --\n");
    }
    if (g_stats.pitch_valid) {
        std::printf("  pitch range: %+.1f ~ %+.1f deg (p-p %.1f)\n",
                    g_stats.pitch_min_deg, g_stats.pitch_max_deg,
                    g_stats.pitch_max_deg - g_stats.pitch_min_deg);
    } else {
        std::printf("  pitch range: --\n");
    }
    std::printf("  rejected targets: %lu\n  speed guard events: %lu\n\n",
                static_cast<unsigned long>(g_stats.rejected_targets),
                static_cast<unsigned long>(g_stats.speed_guard_count));
    std::printf("  Measure forward travel on the floor and divide it by %lu for distance per cycle.\n\n",
                static_cast<unsigned long>(g_stats.cycles));
}

void enterWalkMode() {
    if (!isArmed("walk") || g_walk_mode.load()) return;
    float stand_target[tilt::NUM_JOINTS]{};
    if (!solveFeet(0.0f, g_long_leg_mm, 0.0f, g_long_leg_mm, stand_target)) {
        std::printf("walk rejected: long_leg stand target is invalid.\n");
        return;
    }
    g_stats = {};
    g_walk_mode.store(true);
    g_walk_ready.store(false);
    g_walk_running.store(false);
    g_imu_modulation = false;
    if (!beginJointMove(stand_target, tilt_walk_test::kPoseDurationMs,
                        MoveCompletion::WALK_READY)) {
        g_walk_mode.store(false);
        return;
    }
    std::printf("Entering walk mode: moving to long_leg stand pose first.\n");
}

void quitWalkMode() {
    if (!g_walk_mode.load()) return;
    if (g_walk_running.load()) stopWalk();
    printWalkSummary();
    g_walk_ready.store(false);
    g_walk_mode.store(false);
    g_imu_modulation = false;
    beginJointMove(tilt::ZERO_POSE_RAD, tilt_walk_test::kPoseDurationMs);
    std::printf("Leaving walk mode and returning home.\n");
}

void printHelp() {
    std::printf(
        "\nTILT walk_test commands (press Enter)\n"
        "  help | status | arm | disarm | recover | !\n"
        "  home | stand | joint <LHY|LHP|LKP|RHY|RHP|RKP> <-5..+5> | walk\n"
        "Walk keys: space start/stop, arrows/WASD tune length/legx, +/- diff,\n"
        "  [/] stepTime, {/} legTime, i IMU timing, 0 legx zero,\n"
        "  r attitude display zero, v full status, q exit, ! E-STOP.\n\n");
}

void printStatus() {
    std::printf("state=%s walk=%s\n", safetyName(g_safety.load()),
                g_walk_mode.load()
                    ? (g_walk_running.load() ? "running" : "stopped")
                    : "off");
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const esp_err_t ping = g_bus.ping(tilt::SERVO_ID[joint]);
        std::uint16_t raw = 0;
        const esp_err_t read = g_bus.readPosition(tilt::SERVO_ID[joint], raw);
        if (read == ESP_OK) {
            std::printf("  %-3s ID %u: ping=%s raw=%4u logical=%+7.2fdeg\n",
                        tilt::JOINT_NAME[joint], tilt::SERVO_ID[joint],
                        ping == ESP_OK ? "OK" : "FAIL", raw,
                        tickToRad(joint, raw) * kRadToDeg);
        } else {
            std::printf("  %-3s ID %u: ping=%s read=%s\n",
                        tilt::JOINT_NAME[joint], tilt::SERVO_ID[joint],
                        ping == ESP_OK ? "OK" : "FAIL", esp_err_to_name(read));
        }
    }
}

int jointIndex(const char* name) {
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        if (strcasecmp(name, tilt::JOINT_NAME[joint]) == 0) return joint;
    }
    return -1;
}

void adjustHeight(float requested) {
    const float minimum_delta = std::max(
        tilt_walk_test::kLegLengthMinMm - g_short_leg_mm,
        tilt_walk_test::kLegLengthMinMm - g_long_leg_mm);
    const float maximum_delta = std::min(
        tilt_walk_test::kLegLengthMaxMm - g_short_leg_mm,
        tilt_walk_test::kLegLengthMaxMm - g_long_leg_mm);
    const float delta = std::clamp(requested, minimum_delta, maximum_delta);
    g_long_leg_mm += delta;
    g_short_leg_mm += delta;
    for (int leg = 0; leg < 2; ++leg) g_leg_z_mm[leg] += delta;
    printWalkStatus();
}

void adjustDifference(float delta) {
    const float proposed_short = g_short_leg_mm - delta;
    if (proposed_short < tilt_walk_test::kLegLengthMinMm ||
        proposed_short > tilt_walk_test::kLegLengthMaxMm ||
        proposed_short > g_long_leg_mm) {
        std::printf("length difference limit reached.\n");
        return;
    }
    g_short_leg_mm = proposed_short;
    for (int leg = 0; leg < 2; ++leg) {
        if (g_leg_state[leg] == 1 || g_leg_state[leg] == 2) {
            g_leg_z_mm[leg] = g_short_leg_mm;
        }
    }
    printWalkStatus();
}

void handleCommand(const Command& command) {
    switch (command.type) {
        case CommandType::HELP: printHelp(); return;
        case CommandType::STATUS: printStatus(); return;
        case CommandType::ARM: arm(); return;
        case CommandType::DISARM: disarm(); return;
        case CommandType::RECOVER: recover(); return;
        case CommandType::HOME:
            beginJointMove(tilt::ZERO_POSE_RAD,
                           tilt_walk_test::kPoseDurationMs);
            return;
        case CommandType::STAND: {
            float target[tilt::NUM_JOINTS]{};
            if (!solveFeet(0.0f, g_long_leg_mm, 0.0f, g_long_leg_mm, target)) {
                std::printf("stand target rejected.\n");
                return;
            }
            beginJointMove(target, tilt_walk_test::kPoseDurationMs);
            return;
        }
        case CommandType::JOINT: {
            if (!isArmed("joint")) return;
            const int joint = jointIndex(command.name);
            if (joint < 0 || !std::isfinite(command.value) ||
                command.value == 0.0f ||
                std::fabs(command.value) > tilt_walk_test::kMaxJogStepDeg) {
                std::printf("Usage: joint <LHY|LHP|LKP|RHY|RHP|RKP> <-5..+5>.\n");
                return;
            }
            float target[tilt::NUM_JOINTS]{};
            std::memcpy(target, g_goal_rad, sizeof(target));
            target[joint] += command.value * tilt::DEG2RAD;
            beginJointMove(target, 500);
            return;
        }
        case CommandType::WALK_ENTER: enterWalkMode(); return;
        case CommandType::WALK_TOGGLE:
            if (g_walk_running.load()) stopWalk(); else startWalk();
            return;
        case CommandType::WALK_LEGX_UP:
            g_legx_mm = std::min(g_legx_mm + tilt_walk_test::kLegxStepMm,
                                 tilt_walk_test::kLegxMaxMm);
            printWalkStatus();
            return;
        case CommandType::WALK_LEGX_DOWN:
            g_legx_mm = std::max(g_legx_mm - tilt_walk_test::kLegxStepMm,
                                 -tilt_walk_test::kLegxMaxMm);
            printWalkStatus();
            return;
        case CommandType::WALK_HEIGHT_UP:
            adjustHeight(+tilt_walk_test::kLegLengthStepMm); return;
        case CommandType::WALK_HEIGHT_DOWN:
            adjustHeight(-tilt_walk_test::kLegLengthStepMm); return;
        case CommandType::WALK_DIFF_UP:
            adjustDifference(+tilt_walk_test::kLegLengthStepMm); return;
        case CommandType::WALK_DIFF_DOWN:
            adjustDifference(-tilt_walk_test::kLegLengthStepMm); return;
        case CommandType::WALK_STEP_DOWN:
            g_step_time_ms = std::max(
                tilt_walk_test::kTimeMinMs,
                g_step_time_ms - std::min(g_step_time_ms,
                                          tilt_walk_test::kTimeStepMs));
            printWalkStatus(); return;
        case CommandType::WALK_STEP_UP:
            g_step_time_ms = std::min(
                tilt_walk_test::kTimeMaxMs,
                g_step_time_ms + tilt_walk_test::kTimeStepMs);
            printWalkStatus(); return;
        case CommandType::WALK_LEG_DOWN:
            g_leg_time_ms = std::max(
                tilt_walk_test::kTimeMinMs,
                g_leg_time_ms - std::min(g_leg_time_ms,
                                         tilt_walk_test::kTimeStepMs));
            printWalkStatus(); return;
        case CommandType::WALK_LEG_UP:
            g_leg_time_ms = std::min(
                tilt_walk_test::kTimeMaxMs,
                g_leg_time_ms + tilt_walk_test::kTimeStepMs);
            printWalkStatus(); return;
        case CommandType::WALK_IMU_TOGGLE:
            if (!g_imu_available || !g_pitch_valid) {
                g_imu_modulation = false;
                std::printf("IMU unavailable: timing modulation remains OFF.\n");
            } else {
                g_imu_modulation = !g_imu_modulation;
                std::printf("IMU timing modulation %s.\n",
                            g_imu_modulation ? "ON" : "OFF");
            }
            return;
        case CommandType::WALK_ZERO_LEGX:
            g_legx_mm = 0.0f; printWalkStatus(); return;
        case CommandType::WALK_RESET_ATTITUDE: resetAttitudeZero(); return;
        case CommandType::WALK_STATUS: printWalkStatus(); return;
        case CommandType::WALK_QUIT: quitWalkMode(); return;
    }
}

void enqueue(Command command) {
    if (g_command_queue == nullptr ||
        xQueueSend(g_command_queue, &command, 0) != pdTRUE) {
        ESP_LOGW(kTag, "console command queue full");
    }
}

void parseLine(char* line) {
    while (*line == ' ' || *line == '\t') ++line;
    if (strcasecmp(line, "help") == 0) enqueue({CommandType::HELP});
    else if (strcasecmp(line, "status") == 0) enqueue({CommandType::STATUS});
    else if (strcasecmp(line, "arm") == 0) enqueue({CommandType::ARM});
    else if (strcasecmp(line, "disarm") == 0) enqueue({CommandType::DISARM});
    else if (strcasecmp(line, "recover") == 0) enqueue({CommandType::RECOVER});
    else if (strcasecmp(line, "home") == 0) enqueue({CommandType::HOME});
    else if (strcasecmp(line, "stand") == 0) enqueue({CommandType::STAND});
    else if (strcasecmp(line, "walk") == 0) enqueue({CommandType::WALK_ENTER});
    else {
        Command command{CommandType::JOINT};
        char extra = '\0';
        if (std::sscanf(line, "joint %15s %f %c", command.name,
                        &command.value, &extra) == 2) {
            enqueue(command);
        } else {
            std::printf("Unknown command. Type help.\n");
        }
    }
}

void enqueueWalkKey(std::uint8_t key) {
    switch (key) {
        case ' ': enqueue({CommandType::WALK_TOGGLE}); break;
        case 'd': case 'D': enqueue({CommandType::WALK_LEGX_UP}); break;
        case 'a': case 'A': enqueue({CommandType::WALK_LEGX_DOWN}); break;
        case 'w': case 'W': enqueue({CommandType::WALK_HEIGHT_UP}); break;
        case 's': case 'S': enqueue({CommandType::WALK_HEIGHT_DOWN}); break;
        case '+': enqueue({CommandType::WALK_DIFF_UP}); break;
        case '-': enqueue({CommandType::WALK_DIFF_DOWN}); break;
        case '[': enqueue({CommandType::WALK_STEP_DOWN}); break;
        case ']': enqueue({CommandType::WALK_STEP_UP}); break;
        case '{': enqueue({CommandType::WALK_LEG_DOWN}); break;
        case '}': enqueue({CommandType::WALK_LEG_UP}); break;
        case 'i': case 'I': enqueue({CommandType::WALK_IMU_TOGGLE}); break;
        case '0': enqueue({CommandType::WALK_ZERO_LEGX}); break;
        case 'r': case 'R': enqueue({CommandType::WALK_RESET_ATTITUDE}); break;
        case 'v': case 'V': enqueue({CommandType::WALK_STATUS}); break;
        case 'q': case 'Q': enqueue({CommandType::WALK_QUIT}); break;
        default: break;
    }
}

void consoleTask(void*) {
    enum class EscapeState : std::uint8_t { NONE, ESC, BRACKET };
    EscapeState escape = EscapeState::NONE;
    char line[kLineCapacity]{};
    std::size_t length = 0;
    printHelp();
    std::printf("> ");
    std::fflush(stdout);

    while (true) {
        std::uint8_t byte = 0;
        if (usb_serial_jtag_read_bytes(&byte, 1, pdMS_TO_TICKS(50)) <= 0) continue;
        if (byte == '!') {
            emergencyStopNow();
            length = 0;
            escape = EscapeState::NONE;
            continue;
        }
        if (g_walk_mode.load()) {
            if (escape == EscapeState::ESC) {
                escape = byte == '[' ? EscapeState::BRACKET : EscapeState::NONE;
                continue;
            }
            if (escape == EscapeState::BRACKET) {
                if (byte == 'A') enqueue({CommandType::WALK_HEIGHT_UP});
                else if (byte == 'B') enqueue({CommandType::WALK_HEIGHT_DOWN});
                else if (byte == 'C') enqueue({CommandType::WALK_LEGX_UP});
                else if (byte == 'D') enqueue({CommandType::WALK_LEGX_DOWN});
                escape = EscapeState::NONE;
                continue;
            }
            if (byte == 0x1B) escape = EscapeState::ESC;
            else enqueueWalkKey(byte);
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
        } else if (byte >= 0x20 && byte <= 0x7E &&
                   length + 1 < sizeof(line)) {
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
        ESP_LOGE(kTag, "USB console init failed: %s",
                 esp_err_to_name(console_result));
        return;
    }
    ESP_ERROR_CHECK(g_bus.initialize());
    g_imu_available = tilt::imu_init();
    if (!g_imu_available) {
        ESP_LOGW(kTag, "MPU6050 unavailable; timing modulation OFF, walk remains usable.");
    }

    // Boot invariant: no automatic move and torque requested OFF.
    torqueOffBestEffort();
    std::printf("\nBoot complete: DISARMED, torque OFF, no automatic motion.\n");
    std::printf("Keep a physical servo-power disconnect within reach.\n");

    g_command_queue = xQueueCreate(kQueueDepth, sizeof(Command));
    if (g_command_queue == nullptr) {
        ESP_LOGE(kTag, "could not create console command queue");
        return;
    }
    xTaskCreate(consoleTask, "walk_console", 4096, nullptr, 5, nullptr);

    TickType_t wake = xTaskGetTickCount();
    while (true) {
        const std::uint32_t now = nowMs();
        serviceImu(now);
        Command command{};
        while (xQueueReceive(g_command_queue, &command, 0) == pdTRUE) {
            handleCommand(command);
        }
        serviceJointMove(now);
        serviceWalk(now);
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(tilt_walk_test::kLoopPeriodMs));
    }
}
