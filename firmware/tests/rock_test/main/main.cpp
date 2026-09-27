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
#include "tilt_mpu6050.h"

#include "ParallelLeg.h"
#include "RockConfig.h"

namespace {

constexpr char kTag[] = "rock_test";
constexpr float kRadToDeg = 57.2957795131f;
constexpr std::size_t kLineCapacity = 96;
constexpr UBaseType_t kQueueDepth = 24;
constexpr int kLeft = 0;
constexpr int kRight = 1;
constexpr int kPhaseCount = 6;
constexpr int kMaxLiftRecords = 64;
constexpr float kPoseRecognitionToleranceRad = 2.0f * tilt::DEG2RAD;

enum class SafetyState : std::uint8_t { DISARMED, ARMED, ESTOP };
enum class CommandType : std::uint8_t {
    HELP, STATUS, ARM, DISARM, RECOVER, STAND, ROCK_ENTER,
    ROCK_TOGGLE, ROCK_NEXT, BASE_UP, BASE_DOWN, SHIFT_UP, SHIFT_DOWN,
    LIFT_UP, LIFT_DOWN, SHIFT_TIME_DOWN, SHIFT_TIME_UP,
    LIFT_TIME_DOWN, LIFT_TIME_UP, PLANT_TIME_DOWN, PLANT_TIME_UP,
    LIFT_HOLD_DOWN, LIFT_HOLD_UP, PLANT_HOLD_DOWN, PLANT_HOLD_UP,
    MODE_TOGGLE, IMU_TIMING_TOGGLE,
    LEAN_DOWN, LEAN_UP, RECORD_LIFT, RESET_IMU, ROCK_STATUS,
    ROCK_ZERO, ROCK_QUIT,
};
enum class RampKind : std::uint8_t { NONE, STAND, ENTER, PHASE, ADJUST, ZERO, EXIT };
enum class RockPattern : std::uint8_t { PHASE6, BRUTON };

struct Command {
    CommandType type;
    float height_mm = 0.0f;
    float lean_deg = 0.0f;
};

struct HeightRamp {
    float from_height[2]{};
    float to_height[2]{};
    float from_lean_rad = 0.0f;
    float to_lean_rad = 0.0f;
    std::uint32_t started_ms = 0;
    std::uint32_t duration_ms = 1;
    std::uint32_t hold_ms = 0;
    std::uint32_t reached_ms = 0;
    RampKind kind = RampKind::NONE;
    int phase = -1;
    bool reached = false;
    bool active = false;
};

struct LegInterp {
    float from_mm = 0.0f;
    float to_mm = 0.0f;
    float current_mm = 0.0f;
    std::uint32_t started_ms = 0;
    std::uint32_t duration_ms = 1;

    void reset(float value_mm, std::uint32_t now) {
        from_mm = to_mm = current_mm = value_mm;
        started_ms = now;
        duration_ms = 1;
    }

    float go(float target_mm, std::uint32_t duration, std::uint32_t now) {
        const float elapsed = std::min(
            1.0f, static_cast<float>(now - started_ms) / duration_ms);
        current_mm = from_mm + (to_mm - from_mm) * elapsed;
        if (std::fabs(target_mm - to_mm) > 0.0001f) {
            from_mm = current_mm;
            to_mm = target_mm;
            started_ms = now;
            duration_ms = std::max<std::uint32_t>(duration, 1);
        }
        const float t = std::min(
            1.0f, static_cast<float>(now - started_ms) / duration_ms);
        current_mm = from_mm + (to_mm - from_mm) * t;
        return current_mm;
    }
};

struct LiftTracking {
    float recent[10]{};
    std::uint8_t next = 0;
    std::uint8_t count = 0;
    std::uint32_t total_count = 0;
    float total_sum = 0.0f;
    float last_ratio = 0.0f;

    void add(float ratio) {
        recent[next] = ratio;
        next = static_cast<std::uint8_t>((next + 1) % 10);
        if (count < 10) ++count;
        ++total_count;
        total_sum += ratio;
        last_ratio = ratio;
    }

    float recentMean() const {
        if (count == 0) return 0.0f;
        float sum = 0.0f;
        for (std::uint8_t i = 0; i < count; ++i) sum += recent[i];
        return sum / count;
    }

    float recentMinimum() const {
        if (count == 0) return 0.0f;
        float result = recent[0];
        for (std::uint8_t i = 1; i < count; ++i) {
            result = std::min(result, recent[i]);
        }
        return result;
    }

    float totalMean() const {
        return total_count == 0 ? 0.0f : total_sum / total_count;
    }
};

struct Range {
    bool valid = false;
    float minimum = 0.0f;
    float maximum = 0.0f;

    void add(float value) {
        if (!std::isfinite(value)) return;
        if (!valid) {
            minimum = maximum = value;
            valid = true;
        } else {
            minimum = std::min(minimum, value);
            maximum = std::max(maximum, value);
        }
    }
};

struct LiftRecord {
    int phase = -1;
    float left_mm = 0.0f;
    float right_mm = 0.0f;
    bool roll_valid = false;
    float roll_deg = 0.0f;
};

struct RockStats {
    std::uint32_t cycles = 0;
    std::uint32_t bruton_cycles = 0;
    std::uint32_t rejected_targets = 0;
    std::uint32_t speed_guard_events = 0;
    float max_support_dx_mm = 0.0f;
    Range roll;
    Range pitch;
    std::uint32_t lift_count = 0;
    std::uint32_t left_lift_count = 0;
    std::uint32_t right_lift_count = 0;
    LiftRecord records[kMaxLiftRecords]{};
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
QueueHandle_t g_queue = nullptr;
std::atomic<SafetyState> g_safety{SafetyState::DISARMED};
std::atomic<bool> g_rock_mode{false};
std::atomic<bool> g_rock_ready{false};
std::atomic<bool> g_continuous{false};
RockPattern g_pattern = RockPattern::PHASE6;
std::atomic<bool> g_bruton_running{false};

float g_goal_rad[tilt::NUM_JOINTS]{};
bool g_goal_valid = false;
bool g_parallel_pose_valid = false;
float g_sent_height_mm[2]{tilt::ZERO_POSE_HEIGHT_MM,
                           tilt::ZERO_POSE_HEIGHT_MM};
float g_sent_lean_rad = 0.0f;
HeightRamp g_ramp{};
bool g_speed_guard_latched = false;

float g_base_mm = tilt_rock_test::kBaseDefaultMm;
float g_shift_mm = tilt_rock_test::kShiftDefaultMm;
float g_lift_mm = tilt_rock_test::kLiftDefaultMm;
float g_lean_deg = tilt_rock_test::kLeanDefaultDeg;
std::uint32_t g_shift_duration_ms =
    tilt_rock_test::kShiftDurationDefaultMs;
std::uint32_t g_lift_duration_ms =
    tilt_rock_test::kLiftDurationDefaultMs;
std::uint32_t g_plant_duration_ms =
    tilt_rock_test::kPlantDurationDefaultMs;
std::uint32_t g_lift_hold_ms = tilt_rock_test::kLiftHoldDefaultMs;
std::uint32_t g_plant_hold_ms = tilt_rock_test::kPlantHoldDefaultMs;
float g_bruton_lift_mm = tilt_rock_test::kBrutonLiftDefaultMm;
std::uint32_t g_step_time_base_ms =
    tilt_rock_test::kBrutonStepTimeDefaultMs;
std::uint32_t g_leg_time_base_ms =
    tilt_rock_test::kBrutonLegTimeDefaultMs;
bool g_imu_timing_enabled = false;

LegInterp g_leg_interp[2];
bool g_leg_trigger[2]{};
std::uint8_t g_leg_count[2]{};
float g_leg_z_target_mm[2]{};
float g_bruton_lift_start_mm[2]{};
float g_bruton_lift_target_mm[2]{};
bool g_leg_completed[2]{};
int g_last_triggered_leg = kRight;
std::uint8_t g_step_count = 0;
std::uint32_t g_prev_step_ms = 0;
std::uint32_t g_prev_leg_ms[2]{};
bool g_bruton_rejection_latched = false;
bool g_measure_read_failure_latched = false;
float g_phase_lift_start_mm[2]{};
LiftTracking g_lift_tracking[2];

int g_current_phase = -1;
int g_next_phase = 0;
float g_phase_dx_mm[2]{};
float g_phase_support_dx_mm = 0.0f;
float g_cycle_support_dx_mm = 0.0f;
std::uint8_t g_completed_phases_in_cycle = 0;
Range g_cycle_roll;
Range g_cycle_pitch;
std::uint32_t g_retry_after_ms = 0;
bool g_phase_reject_warned[kPhaseCount]{};
RockStats g_stats{};

bool g_imu_available = false;
bool g_roll_valid = false;
bool g_pitch_valid = false;
float g_roll_deg = 0.0f;
float g_pitch_deg = 0.0f;
bool g_imu_zero_valid = false;
float g_roll_zero_deg = 0.0f;
float g_pitch_zero_deg = 0.0f;
std::uint32_t g_last_imu_ms = 0;
std::uint32_t g_imu_failures = 0;

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

const char* phaseName(int phase) {
    constexpr const char* names[kPhaseCount] = {
        "SHIFT_R", "LIFT_L", "PLANT_L",
        "SHIFT_L", "LIFT_R", "PLANT_R",
    };
    return phase >= 0 && phase < kPhaseCount ? names[phase] : "STAND";
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

bool inferCurrentParallelPose() {
    if (!g_goal_valid) return false;
    float heights[2]{};
    float lean[2]{};
    for (int leg = 0; leg < 2; ++leg) {
        const int offset = leg * 3;
        const float yaw = g_goal_rad[offset];
        const float hip = g_goal_rad[offset + 1];
        const float knee = g_goal_rad[offset + 2];
        const float a = knee + tilt::ANKLE_FIXED_RAD +
                        tilt::KNEE_OFFSET_RAD;
        lean[leg] = hip + a;
        heights[leg] = tilt_rock_test::calfVerticalMm() +
                       tilt::THIGH_LENGTH_MM * std::cos(a);
        if (!std::isfinite(a) || a < 0.0f || a > 1.57f ||
            std::fabs(yaw) > kPoseRecognitionToleranceRad) {
            return false;
        }
    }
    if (std::fabs(lean[kLeft] - lean[kRight]) >
        kPoseRecognitionToleranceRad) {
        return false;
    }
    const float common_lean = (lean[kLeft] + lean[kRight]) * 0.5f;
    float reconstructed[tilt::NUM_JOINTS]{};
    if (!tilt_rock_test::parallelStance(
            heights[kLeft], heights[kRight], common_lean, reconstructed)) {
        return false;
    }
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        if (std::fabs(reconstructed[joint] - g_goal_rad[joint]) >
            kPoseRecognitionToleranceRad) {
            return false;
        }
    }
    g_sent_height_mm[kLeft] = heights[kLeft];
    g_sent_height_mm[kRight] = heights[kRight];
    g_sent_lean_rad = common_lean;
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
    g_safety.store(SafetyState::ESTOP);
    g_continuous.store(false);
    g_bruton_running = false;
    g_rock_ready.store(false);
    g_rock_mode.store(false);
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
        std::printf("arm rejected: could not read all six positions.\n");
        return;
    }
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        g_goal_rad[joint] = tickToRad(joint, ticks[joint]);
    }
    g_goal_valid = true;
    g_parallel_pose_valid = inferCurrentParallelPose();

    // Never enable torque against a stale servo-register goal.
    if (!busOk(g_bus.syncWritePositions(tilt::SERVO_ID, ticks,
                                         tilt::NUM_JOINTS,
                                         tilt_rock_test::kServoSpeedRaw),
               "arm hold target")) {
        return;
    }
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        if (!busOk(g_bus.setAcceleration(
                       tilt::SERVO_ID[joint],
                       tilt_rock_test::kServoAcceleration),
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
    std::printf("ARMED: present six-joint pose held.\n");
    if (!g_parallel_pose_valid) {
        std::printf("Current pose is not a parallel-leg pose (within 2deg). "
                    "stand/rock are blocked; disarm and position the robot "
                    "near the zero parallel pose first.\n");
    }
}

void disarm() {
    if (g_safety.load() == SafetyState::ESTOP) {
        std::printf("Use recover first; ESTOP remains latched.\n");
        return;
    }
    g_continuous.store(false);
    g_bruton_running = false;
    g_rock_ready.store(false);
    g_rock_mode.store(false);
    g_ramp.active = false;
    g_parallel_pose_valid = false;
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
    g_ramp.active = false;
    g_bruton_running = false;
    g_parallel_pose_valid = false;
    g_speed_guard_latched = false;
    g_safety.store(SafetyState::DISARMED);
    std::printf("Recovered to DISARMED. Inspect the robot, then arm again.\n");
}

void serviceImu(std::uint32_t now) {
    if (!g_imu_available ||
        now - g_last_imu_ms < tilt::IMU_SAMPLE_PERIOD_MS) return;
    const float dt_s = g_last_imu_ms == 0
                           ? tilt::IMU_SAMPLE_PERIOD_MS / 1000.0f
                           : (now - g_last_imu_ms) / 1000.0f;
    g_last_imu_ms = now;
    tilt::ImuRaw raw{};
    if (!tilt::imu_read_raw(raw)) {
        ++g_imu_failures;
        if (g_imu_failures >= 5) {
            g_imu_available = false;
            ESP_LOGW(kTag, "MPU6050 unavailable; rocking continues without attitude display.");
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

void sampleStats() {
    if (!g_rock_mode.load()) return;
    if (g_roll_valid) g_stats.roll.add(g_roll_deg);
    if (g_pitch_valid) g_stats.pitch.add(g_pitch_deg);
    if (g_continuous.load() || g_bruton_running) {
        if (g_roll_valid) g_cycle_roll.add(g_roll_deg);
        if (g_pitch_valid) g_cycle_pitch.add(g_pitch_deg);
    }
}

void phaseTargets(int phase, float& left_mm, float& right_mm) {
    switch (phase) {
        case 0:  // SHIFT_R: unload the left foot.
        case 2:  // PLANT_L.
            left_mm = g_base_mm + g_shift_mm;
            right_mm = g_base_mm - g_shift_mm;
            break;
        case 1:  // LIFT_L: shorten only the unloaded left leg.
            left_mm = g_base_mm + g_shift_mm - g_lift_mm;
            right_mm = g_base_mm - g_shift_mm;
            break;
        case 3:  // SHIFT_L: unload the right foot.
        case 5:  // PLANT_R.
            left_mm = g_base_mm - g_shift_mm;
            right_mm = g_base_mm + g_shift_mm;
            break;
        case 4:  // LIFT_R: shorten only the unloaded right leg.
            left_mm = g_base_mm - g_shift_mm;
            right_mm = g_base_mm + g_shift_mm - g_lift_mm;
            break;
        default:
            left_mm = right_mm = g_base_mm;
            break;
    }
}

std::uint32_t phaseDurationMs(int phase) {
    if (phase == 0 || phase == 3) return g_shift_duration_ms;
    if (phase == 1 || phase == 4) return g_lift_duration_ms;
    return g_plant_duration_ms;
}

bool validStance(float left_mm, float right_mm, float lean_rad) {
    float joints[tilt::NUM_JOINTS]{};
    return tilt_rock_test::parallelStance(
        left_mm, right_mm, lean_rad, joints);
}

bool allPhasesReachable(bool verbose) {
    bool all_valid = true;
    for (int phase = 0; phase < kPhaseCount; ++phase) {
        float left_mm = 0.0f;
        float right_mm = 0.0f;
        phaseTargets(phase, left_mm, right_mm);
        if (!validStance(left_mm, right_mm, g_lean_deg * tilt::DEG2RAD)) {
            all_valid = false;
            if (verbose) {
                std::printf("phase %d %s: UNREACHABLE (L %.1f / R %.1f mm).\n",
                            phase, phaseName(phase), left_mm, right_mm);
            }
        }
    }
    return all_valid;
}

bool beginHeightRamp(float left_mm, float right_mm, float lean_rad,
                     std::uint32_t duration_ms, RampKind kind,
                     int phase = -1, std::uint32_t hold_ms = 0) {
    if (!isArmed("move") || !g_parallel_pose_valid) {
        if (!g_parallel_pose_valid) {
            std::printf("Move rejected: current pose is not recognized as parallel.\n");
        }
        return false;
    }
    if (!validStance(left_mm, right_mm, lean_rad)) {
        ++g_stats.rejected_targets;
        std::printf("Unreachable or joint-limit target rejected: L %.1f R %.1f lean %+.1fdeg. "
                    "Last goal held.\n",
                    left_mm, right_mm, lean_rad * kRadToDeg);
        return false;
    }
    g_ramp.from_height[kLeft] = g_sent_height_mm[kLeft];
    g_ramp.from_height[kRight] = g_sent_height_mm[kRight];
    g_ramp.to_height[kLeft] = left_mm;
    g_ramp.to_height[kRight] = right_mm;
    g_ramp.from_lean_rad = g_sent_lean_rad;
    g_ramp.to_lean_rad = lean_rad;
    g_ramp.started_ms = nowMs();
    g_ramp.duration_ms = std::max<std::uint32_t>(duration_ms, 1);
    g_ramp.hold_ms = hold_ms;
    g_ramp.reached = false;
    g_ramp.reached_ms = 0;
    g_ramp.kind = kind;
    g_ramp.phase = phase;
    g_ramp.active = true;
    return true;
}

void printAttitudeInline() {
    if (g_roll_valid) std::printf("roll=%+.1f ", g_roll_deg);
    else std::printf("roll=-- ");
    if (g_pitch_valid) std::printf("pitch=%+.1f", g_pitch_deg);
    else std::printf("pitch=--");
}

void printPhase(int phase, std::uint32_t duration_ms) {
    const auto left = tilt_rock_test::parallelLeg(
        g_sent_height_mm[kLeft], g_sent_lean_rad);
    const auto right = tilt_rock_test::parallelLeg(
        g_sent_height_mm[kRight], g_sent_lean_rad);
    std::printf("[%d %s] L=%.1f R=%.1f (dt %lums) | "
                "L a=%.1f x%+.1f(d%+.1f) | "
                "R a=%.1f x%+.1f(d%+.1f) | support dx=%.1f | ",
                phase, phaseName(phase),
                g_sent_height_mm[kLeft], g_sent_height_mm[kRight],
                static_cast<unsigned long>(duration_ms),
                left.thigh_angle_rad * kRadToDeg,
                left.foot_x_mm, g_phase_dx_mm[kLeft],
                right.thigh_angle_rad * kRadToDeg,
                right.foot_x_mm, g_phase_dx_mm[kRight],
                g_phase_support_dx_mm);
    printAttitudeInline();
    std::printf("\n");
}

void printRangeInline(const char* label, const Range& range) {
    if (range.valid) {
        std::printf("%s %+.1f~%+.1f (p-p %.1f)", label,
                    range.minimum, range.maximum,
                    range.maximum - range.minimum);
    } else {
        std::printf("%s --", label);
    }
}

void printCycle() {
    std::printf("cyc#%lu base=%.1f s=%.1f l=%.1f lean=%+.1f "
                "t=%lu/%lu/%lu | ",
                static_cast<unsigned long>(g_stats.cycles),
                g_base_mm, g_shift_mm, g_lift_mm, g_lean_deg,
                static_cast<unsigned long>(g_shift_duration_ms),
                static_cast<unsigned long>(g_lift_duration_ms),
                static_cast<unsigned long>(g_plant_duration_ms));
    printRangeInline("roll", g_cycle_roll);
    std::printf(" ");
    printRangeInline("pitch", g_cycle_pitch);
    std::printf(" | lift L %.0f%% R %.0f%% | max support dx %.1fmm\n",
                g_lift_tracking[kLeft].last_ratio * 100.0f,
                g_lift_tracking[kRight].last_ratio * 100.0f,
                g_cycle_support_dx_mm);
    g_cycle_roll = {};
    g_cycle_pitch = {};
    g_cycle_support_dx_mm = 0.0f;
}

const char* patternName() {
    return g_pattern == RockPattern::PHASE6 ? "PHASE6" : "BRUTON";
}

void printTrackingRecent() {
    std::printf("  lift tracking (last 10; knee motion, not foot clearance): ");
    for (int leg = 0; leg < 2; ++leg) {
        const LiftTracking& tracking = g_lift_tracking[leg];
        if (tracking.count) {
            std::printf("%c mean %.0f%% min %.0f%% (n=%u)  ",
                        leg == kLeft ? 'L' : 'R',
                        tracking.recentMean() * 100.0f,
                        tracking.recentMinimum() * 100.0f, tracking.count);
        } else {
            std::printf("%c --  ", leg == kLeft ? 'L' : 'R');
        }
    }
    std::printf("\n");
}

void printModeKeys() {
    if (g_pattern == RockPattern::PHASE6) {
        std::printf("PHASE6 keys: space run/stop, n next, arrows/WASD base/shift, "
                    "+/- lift, 1/2 shift, 3/4 lift, 5/6 plant, "
                    "[/] lift hold, ;/' plant hold.\n");
    } else {
        std::printf("BRUTON keys: space run/stop, up/down base, +/- lift, "
                    "1/2 step time, 3/4 leg time, ,/. lean, i IMU timing. "
                    "n/shift keys inactive.\n");
    }
    std::printf("Common: m mode (stopped), k lift observation, r IMU zero, "
                "v status, 0 base, q exit, ! E-STOP.\n");
}

void printRockStatus() {
    std::printf("[ROCK %s %s] base=%.1f ", patternName(),
                (g_continuous.load() || g_bruton_running) ? "running" : "stopped",
                g_base_mm);
    if (g_pattern == RockPattern::PHASE6) {
        std::printf("shift=%.1f lift=%.1f lean=%+.1fdeg\n",
                    g_shift_mm, g_lift_mm, g_lean_deg);
        std::printf("  t: shift=%lu lift=%lu plant=%lu  hold: lift=%lu plant=%lu ms -> cycle %lums\n",
                    static_cast<unsigned long>(g_shift_duration_ms),
                    static_cast<unsigned long>(g_lift_duration_ms),
                    static_cast<unsigned long>(g_plant_duration_ms),
                    static_cast<unsigned long>(g_lift_hold_ms),
                    static_cast<unsigned long>(g_plant_hold_ms),
                    static_cast<unsigned long>(2 * (g_shift_duration_ms +
                        g_lift_duration_ms + g_plant_duration_ms +
                        g_lift_hold_ms + g_plant_hold_ms)));
    } else {
        std::printf("lift=%.1f lean=%+.1fdeg (shift ignored)\n",
                    g_bruton_lift_mm, g_lean_deg);
        std::printf("  step_time=%lu leg_time=%lu ms  IMU timing=%s\n",
                    static_cast<unsigned long>(g_step_time_base_ms),
                    static_cast<unsigned long>(g_leg_time_base_ms),
                    g_imu_timing_enabled ? "ON" : "OFF");
    }
    printTrackingRecent();
    const auto base = tilt_rock_test::parallelLeg(g_base_mm, 0.0f);
    if (base.reachable) {
        const float coupling =
            std::fabs(std::cos(base.thigh_angle_rad) /
                      std::sin(base.thigh_angle_rad));
        std::printf("  base pose: a=%.1fdeg foot_x=%+.1fmm  x coupling %.2f mm/mm\n",
                    base.thigh_angle_rad * kRadToDeg,
                    base.foot_x_mm, coupling);
    } else {
        std::printf("  base pose: UNREACHABLE\n");
    }
    if (g_pattern == RockPattern::PHASE6) {
        std::printf("  phase targets (L / R):\n");
        for (int phase = 0; phase < kPhaseCount; ++phase) {
            float left_mm = 0.0f;
            float right_mm = 0.0f;
            phaseTargets(phase, left_mm, right_mm);
            const bool reachable = validStance(
                left_mm, right_mm, g_lean_deg * tilt::DEG2RAD);
            std::printf("    %d %-8s %5.1f / %5.1f%s\n",
                        phase, phaseName(phase), left_mm, right_mm,
                        reachable ? "" : "  UNREACHABLE");
        }
    } else {
        const float short_mm = g_base_mm - g_bruton_lift_mm;
        std::printf("  BRUTON short=%.1f mm %s\n", short_mm,
                    validStance(short_mm, g_base_mm,
                        g_lean_deg * tilt::DEG2RAD) &&
                    validStance(g_base_mm, short_mm,
                        g_lean_deg * tilt::DEG2RAD) ? "" : "UNREACHABLE");
    }
    std::printf("  IMU ");
    if (g_roll_valid) {
        std::printf("roll=%+.1f", g_roll_deg);
        if (g_imu_zero_valid) {
            std::printf(" (rel %+.1f)", g_roll_deg - g_roll_zero_deg);
        }
    } else {
        std::printf("roll=--");
    }
    if (g_pitch_valid) {
        std::printf("  pitch=%+.1f", g_pitch_deg);
        if (g_imu_zero_valid) {
            std::printf(" (rel %+.1f)", g_pitch_deg - g_pitch_zero_deg);
        }
    } else {
        std::printf("  pitch=--");
    }
    std::printf("\n  recorded foot lifts: L %lu, R %lu  "
                "rejected targets: %lu\n",
                static_cast<unsigned long>(g_stats.left_lift_count),
                static_cast<unsigned long>(g_stats.right_lift_count),
                static_cast<unsigned long>(g_stats.rejected_targets));
}

void printRockSummary() {
    std::printf("\n-- rock result --\n");
    std::printf("  pattern: %s\n", patternName());
    if (g_pattern == RockPattern::PHASE6) {
        std::printf("  base %.1f shift %.1f lift %.1f lean %+.1f, "
                    "durations %lu/%lu/%lu, hold %lu/%lu, %lu cycles\n",
                    g_base_mm, g_shift_mm, g_lift_mm, g_lean_deg,
                    static_cast<unsigned long>(g_shift_duration_ms),
                    static_cast<unsigned long>(g_lift_duration_ms),
                    static_cast<unsigned long>(g_plant_duration_ms),
                    static_cast<unsigned long>(g_lift_hold_ms),
                    static_cast<unsigned long>(g_plant_hold_ms),
                    static_cast<unsigned long>(g_stats.cycles));
    } else {
        std::printf("  base %.1f lift %.1f lean %+.1f, step %lu leg %lu, "
                    "%lu cycles\n", g_base_mm, g_bruton_lift_mm,
                    g_lean_deg,
                    static_cast<unsigned long>(g_step_time_base_ms),
                    static_cast<unsigned long>(g_leg_time_base_ms),
                    static_cast<unsigned long>(g_stats.bruton_cycles));
    }
    std::printf("  lift tracking (knee motion, not foot clearance): ");
    for (int leg = 0; leg < 2; ++leg) {
        const LiftTracking& tracking = g_lift_tracking[leg];
        if (tracking.total_count) {
            std::printf("%c mean %.0f%% (n=%lu)  ",
                        leg == kLeft ? 'L' : 'R', tracking.totalMean() * 100.0f,
                        static_cast<unsigned long>(tracking.total_count));
        } else {
            std::printf("%c --  ", leg == kLeft ? 'L' : 'R');
        }
    }
    std::printf("\n");
    if (g_lift_tracking[kLeft].total_count &&
        g_lift_tracking[kRight].total_count &&
        g_lift_tracking[kLeft].totalMean() >= 0.85f &&
        g_lift_tracking[kRight].totalMean() >= 0.85f &&
        g_stats.lift_count == 0) {
        std::printf("  If feet did not lift despite high tracking, torso may "
                    "have followed downward; inspect foot position/hardware.\n");
    }
    std::printf("  ");
    printRangeInline("roll", g_stats.roll);
    std::printf("   ");
    printRangeInline("pitch", g_stats.pitch);
    std::printf("\n  max support dx: %.1f mm\n",
                g_stats.max_support_dx_mm);
    std::printf("  rejected targets: %lu, speed guard events: %lu\n",
                static_cast<unsigned long>(g_stats.rejected_targets),
                static_cast<unsigned long>(g_stats.speed_guard_events));
    std::printf("  foot-lift records: L %lu, R %lu\n",
                static_cast<unsigned long>(g_stats.left_lift_count),
                static_cast<unsigned long>(g_stats.right_lift_count));
    const std::uint32_t stored = std::min<std::uint32_t>(
        g_stats.lift_count, kMaxLiftRecords);
    for (std::uint32_t i = 0; i < stored; ++i) {
        const LiftRecord& record = g_stats.records[i];
        std::printf("    #%lu %s L=%.1f R=%.1f roll ",
                    static_cast<unsigned long>(i + 1),
                    phaseName(record.phase),
                    record.left_mm, record.right_mm);
        if (record.roll_valid) std::printf("%+.1f\n", record.roll_deg);
        else std::printf("--\n");
    }
    if (g_stats.lift_count > stored) {
        std::printf("    (%lu additional records not stored)\n",
                    static_cast<unsigned long>(g_stats.lift_count - stored));
    }
    std::printf("\n");
}

bool transmitStance(float left_mm, float right_mm, float lean_rad) {
    if (g_safety.load() != SafetyState::ARMED) return false;
    float target[tilt::NUM_JOINTS]{};
    if (!tilt_rock_test::parallelStance(
            left_mm, right_mm, lean_rad, target)) {
        ++g_stats.rejected_targets;
        if (!g_bruton_rejection_latched) {
            std::printf("Interpolated target rejected; last goal held.\n");
        }
        g_bruton_rejection_latched = true;
        g_ramp.active = false;
        return false;
    }
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const float step = target[joint] - g_goal_rad[joint];
        if (g_goal_valid &&
            std::fabs(step) > tilt_rock_test::kMaxJointStepRad) {
            if (!g_speed_guard_latched) {
                ESP_LOGW(kTag, "Speed guard blocked %s (%+.3fdeg/10ms).",
                         tilt::JOINT_NAME[joint], step * kRadToDeg);
                ++g_stats.speed_guard_events;
            }
            g_speed_guard_latched = true;
            g_continuous.store(false);
            g_bruton_running = false;
            for (int leg = 0; leg < 2; ++leg) {
                g_leg_z_target_mm[leg] = g_sent_height_mm[leg];
                g_leg_interp[leg].reset(g_sent_height_mm[leg], nowMs());
            }
            if (g_ramp.kind == RampKind::PHASE) {
                g_next_phase = g_ramp.phase;
            }
            for (int leg = 0; leg < 2; ++leg) {
                g_ramp.from_height[leg] = g_sent_height_mm[leg];
                g_ramp.to_height[leg] = g_sent_height_mm[leg];
            }
            g_ramp.from_lean_rad = g_sent_lean_rad;
            g_ramp.to_lean_rad = g_sent_lean_rad;
            g_ramp.active = false;
            // g_sent_height_mm and g_goal_rad change only after a successful
            // packet. Aborting here freezes the target at that actual packet.
            std::printf("Motion stopped; last transmitted goal held.\n");
            return false;
        }
    }
    std::uint16_t ticks[tilt::NUM_JOINTS]{};
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        ticks[joint] = radToTick(joint, target[joint]);
    }
    if (!busOk(g_bus.syncWritePositions(tilt::SERVO_ID, ticks,
                                         tilt::NUM_JOINTS,
                                         tilt_rock_test::kServoSpeedRaw),
               "syncWritePositions")) {
        emergencyStopNow();
        return false;
    }
    std::memcpy(g_goal_rad, target, sizeof(g_goal_rad));
    g_sent_height_mm[kLeft] = left_mm;
    g_sent_height_mm[kRight] = right_mm;
    g_sent_lean_rad = lean_rad;
    g_goal_valid = true;
    g_speed_guard_latched = false;
    g_bruton_rejection_latched = false;
    return true;
}

void measureLift(int leg, float h_start, float h_cmd) {
    const float lift_cmd = h_start - h_cmd;
    if (lift_cmd < 0.5f) return;
    const int knee_joint = leg == kLeft ? 2 : 5;
    std::uint16_t raw = 0;
    const esp_err_t result = g_bus.readPosition(
        tilt::SERVO_ID[knee_joint], raw);
    if (result != ESP_OK) {
        if (!g_measure_read_failure_latched) {
            ESP_LOGW(kTag, "Lift tracking read %s failed: %s; event skipped.",
                     tilt::JOINT_NAME[knee_joint], esp_err_to_name(result));
        }
        g_measure_read_failure_latched = true;
        return;
    }
    g_measure_read_failure_latched = false;
    const float knee_rad = tickToRad(knee_joint, raw);
    const float a_actual = knee_rad + tilt::ANKLE_FIXED_RAD +
                           tilt::KNEE_OFFSET_RAD;
    const float h_actual = tilt_rock_test::calfVerticalMm() +
                           tilt::THIGH_LENGTH_MM * std::cos(a_actual);
    const float lift_actual = h_start - h_actual;
    const float ratio = lift_actual / lift_cmd;
    if (!std::isfinite(ratio)) return;
    g_lift_tracking[leg].add(ratio);
    const float a_start = std::acos(std::clamp(
        (h_start - tilt_rock_test::calfVerticalMm()) /
            tilt::THIGH_LENGTH_MM, -1.0f, 1.0f));
    std::printf("LIFT_%c command -%.1fmm -> actual -%.1fmm (%.0f%%) "
                "a %.1f->%.1fdeg roll=",
                leg == kLeft ? 'L' : 'R', lift_cmd, lift_actual,
                ratio * 100.0f, a_start * kRadToDeg,
                a_actual * kRadToDeg);
    if (g_roll_valid) std::printf("%+.1f", g_roll_deg);
    else std::printf("--");
    std::printf(" [knee tracking; not ground clearance]\n");
}

bool startPhase(int phase) {
    float left_mm = 0.0f;
    float right_mm = 0.0f;
    phaseTargets(phase, left_mm, right_mm);
    const float lean_rad = g_lean_deg * tilt::DEG2RAD;
    if (!validStance(left_mm, right_mm, lean_rad)) {
        ++g_stats.rejected_targets;
        if (!g_phase_reject_warned[phase]) {
            std::printf("Phase %d %s rejected: unreachable or joint limit; "
                        "last goal held.\n", phase, phaseName(phase));
            g_phase_reject_warned[phase] = true;
        }
        return false;
    }
    const auto from_left = tilt_rock_test::parallelLeg(
        g_sent_height_mm[kLeft], g_sent_lean_rad);
    const auto from_right = tilt_rock_test::parallelLeg(
        g_sent_height_mm[kRight], g_sent_lean_rad);
    const auto to_left = tilt_rock_test::parallelLeg(left_mm, lean_rad);
    const auto to_right = tilt_rock_test::parallelLeg(right_mm, lean_rad);
    g_phase_dx_mm[kLeft] = to_left.foot_x_mm - from_left.foot_x_mm;
    g_phase_dx_mm[kRight] = to_right.foot_x_mm - from_right.foot_x_mm;
    const int support_leg = phase <= 2 ? kRight : kLeft;
    g_phase_support_dx_mm = std::fabs(g_phase_dx_mm[support_leg]);

    const std::uint32_t hold_ms = (phase == 1 || phase == 4)
        ? g_lift_hold_ms : ((phase == 2 || phase == 5)
            ? g_plant_hold_ms : 0);
    if (!beginHeightRamp(left_mm, right_mm, lean_rad,
                         phaseDurationMs(phase), RampKind::PHASE,
                         phase, hold_ms)) {
        return false;
    }
    if (phase == 1) g_phase_lift_start_mm[kLeft] = g_sent_height_mm[kLeft];
    if (phase == 4) g_phase_lift_start_mm[kRight] = g_sent_height_mm[kRight];
    g_current_phase = phase;
    g_next_phase = (phase + 1) % kPhaseCount;
    g_stats.max_support_dx_mm = std::max(
        g_stats.max_support_dx_mm, g_phase_support_dx_mm);
    if (g_continuous.load()) {
        g_cycle_support_dx_mm = std::max(
            g_cycle_support_dx_mm, g_phase_support_dx_mm);
    }
    return true;
}

void finishRamp() {
    const RampKind kind = g_ramp.kind;
    const int phase = g_ramp.phase;
    const std::uint32_t duration_ms = g_ramp.duration_ms;
    g_ramp.active = false;
    g_ramp.kind = RampKind::NONE;
    if (kind == RampKind::ENTER) {
        g_current_phase = -1;
        g_next_phase = 0;
        g_rock_ready.store(true);
        if (g_roll_valid && g_pitch_valid) {
            g_roll_zero_deg = g_roll_deg;
            g_pitch_zero_deg = g_pitch_deg;
            g_imu_zero_valid = true;
        }
        std::printf("Rock mode ready and STOPPED. n steps one phase; "
                    "space starts continuous motion.\n");
        printRockStatus();
    } else if (kind == RampKind::PHASE) {
        if (phase == 1 || phase == 4) {
            const int leg = phase == 1 ? kLeft : kRight;
            measureLift(leg, g_phase_lift_start_mm[leg],
                        g_ramp.to_height[leg]);
        }
        if (g_continuous.load()) {
            ++g_completed_phases_in_cycle;
            if (g_completed_phases_in_cycle == kPhaseCount) {
                g_completed_phases_in_cycle = 0;
                ++g_stats.cycles;
                printCycle();
            }
        } else {
            printPhase(phase, duration_ms);
        }
    } else if (kind == RampKind::ZERO) {
        g_current_phase = -1;
        g_next_phase = 0;
        std::printf("Returned to base parallel stand.\n");
    } else if (kind == RampKind::EXIT) {
        g_current_phase = -1;
        g_rock_ready.store(false);
        g_rock_mode.store(false);
        std::printf("Rock mode closed at base parallel stand.\n> ");
        std::fflush(stdout);
    } else if (kind == RampKind::STAND) {
        std::printf("Parallel stand complete: h=%.1fmm lean=%+.1fdeg.\n",
                    g_sent_height_mm[kLeft], g_sent_lean_rad * kRadToDeg);
    }
}

void serviceRamp(std::uint32_t now) {
    if (g_safety.load() != SafetyState::ARMED) {
        g_ramp.active = false;
        return;
    }
    if (!g_ramp.active) return;
    if (g_ramp.reached) {
        if (now - g_ramp.reached_ms >= g_ramp.hold_ms) finishRamp();
        return;
    }
    const std::uint32_t elapsed = now - g_ramp.started_ms;
    const float t = std::min(1.0f, static_cast<float>(elapsed) /
                                      g_ramp.duration_ms);
    const float left_mm = g_ramp.from_height[kLeft] +
        (g_ramp.to_height[kLeft] - g_ramp.from_height[kLeft]) * t;
    const float right_mm = g_ramp.from_height[kRight] +
        (g_ramp.to_height[kRight] - g_ramp.from_height[kRight]) * t;
    const float lean_rad = g_ramp.from_lean_rad +
        (g_ramp.to_lean_rad - g_ramp.from_lean_rad) * t;
    if (!transmitStance(left_mm, right_mm, lean_rad)) return;
    if (t >= 1.0f) {
        g_ramp.reached = true;
        g_ramp.reached_ms = now;
        if (g_ramp.hold_ms == 0) finishRamp();
    }
}

void serviceContinuous(std::uint32_t now) {
    if (g_pattern != RockPattern::PHASE6 || !g_continuous.load() ||
        !g_rock_ready.load() || g_ramp.active ||
        g_safety.load() != SafetyState::ARMED ||
        static_cast<std::int32_t>(now - g_retry_after_ms) < 0) {
        return;
    }
    const int phase = g_next_phase;
    if (!startPhase(phase)) {
        // Invalid goals are skipped at their normal cadence. They never cause
        // an automatic stop, and duplicate warnings stay suppressed.
        g_completed_phases_in_cycle = 0;
        g_cycle_roll = {};
        g_cycle_pitch = {};
        g_cycle_support_dx_mm = 0.0f;
        g_next_phase = (phase + 1) % kPhaseCount;
        g_retry_after_ms = now + phaseDurationMs(phase);
    }
}

std::uint32_t brutonStepTimeMs() {
    if (!g_imu_timing_enabled || !g_pitch_valid) return g_step_time_base_ms;
    return static_cast<std::uint32_t>(std::lround(std::clamp(
        static_cast<float>(g_step_time_base_ms) +
            std::fabs(g_pitch_deg) * tilt_rock_test::kBrutonStepPitchGain,
        static_cast<float>(g_step_time_base_ms) - 10.0f,
        static_cast<float>(g_step_time_base_ms) + 10.0f)));
}

std::uint32_t brutonLegTimeMs() {
    if (!g_imu_timing_enabled || !g_pitch_valid) return g_leg_time_base_ms;
    return static_cast<std::uint32_t>(std::lround(std::clamp(
        static_cast<float>(g_leg_time_base_ms) -
            std::fabs(g_pitch_deg) * tilt_rock_test::kBrutonLegPitchGain,
        static_cast<float>(g_leg_time_base_ms) - 20.0f,
        static_cast<float>(g_leg_time_base_ms))));
}

void printBrutonCycle() {
    std::printf("cyc#%lu [BRUTON] base=%.1f lift=%.1f step=%lu leg=%lu "
                "| lift L %.0f%% R %.0f%% | ",
                static_cast<unsigned long>(g_stats.bruton_cycles),
                g_base_mm, g_bruton_lift_mm,
                static_cast<unsigned long>(brutonStepTimeMs()),
                static_cast<unsigned long>(brutonLegTimeMs()),
                g_lift_tracking[kLeft].last_ratio * 100.0f,
                g_lift_tracking[kRight].last_ratio * 100.0f);
    printRangeInline("roll", g_cycle_roll);
    std::printf(" ");
    printRangeInline("pitch", g_cycle_pitch);
    std::printf("\n");
    g_cycle_roll = {};
    g_cycle_pitch = {};
}

void resetBrutonState(std::uint32_t now) {
    g_step_count = 0;
    g_prev_step_ms = now;
    g_last_triggered_leg = kRight;
    for (int leg = 0; leg < 2; ++leg) {
        g_leg_trigger[leg] = false;
        g_leg_count[leg] = 0;
        g_leg_completed[leg] = false;
        g_prev_leg_ms[leg] = now;
        g_leg_z_target_mm[leg] = g_base_mm;
        g_leg_interp[leg].reset(g_sent_height_mm[leg], now);
    }
    g_cycle_roll = {};
    g_cycle_pitch = {};
}

void serviceBruton(std::uint32_t now) {
    if (!g_bruton_running || g_pattern != RockPattern::BRUTON ||
        !g_rock_ready.load() || g_ramp.active ||
        g_safety.load() != SafetyState::ARMED) return;
    const std::uint32_t step_time = brutonStepTimeMs();
    const std::uint32_t leg_time = brutonLegTimeMs();
    if (now - g_prev_step_ms >= step_time) {
        const int leg = g_step_count == 0 ? kRight : kLeft;
        g_step_count = g_step_count == 0 ? 1 : 0;
        g_leg_trigger[leg] = true;
        g_last_triggered_leg = leg;
        g_prev_step_ms = now;
    }
    bool measure_due[2]{};
    for (int leg = 0; leg < 2; ++leg) {
        if (g_leg_trigger[leg] && g_leg_count[leg] == 0 &&
            now - g_prev_leg_ms[leg] >= leg_time) {
            g_bruton_lift_start_mm[leg] = g_base_mm;
            g_bruton_lift_target_mm[leg] = g_base_mm - g_bruton_lift_mm;
            g_leg_z_target_mm[leg] = g_bruton_lift_target_mm[leg];
            g_leg_trigger[leg] = false;
            g_leg_count[leg] = 1;
            g_prev_leg_ms[leg] = now;
        } else if (g_leg_count[leg] == 1 &&
                   now - g_prev_leg_ms[leg] >= leg_time) {
            measure_due[leg] = true;
            g_leg_count[leg] = 2;
            g_prev_leg_ms[leg] = now;
        } else if (g_leg_count[leg] == 2 &&
                   now - g_prev_leg_ms[leg] >= leg_time) {
            g_leg_z_target_mm[leg] = g_base_mm;
            g_leg_count[leg] = 3;
            g_prev_leg_ms[leg] = now;
        } else if (g_leg_count[leg] == 3 &&
                   now - g_prev_leg_ms[leg] >= leg_time) {
            g_leg_count[leg] = 0;
            g_prev_leg_ms[leg] = now;
            g_leg_completed[leg] = true;
        }
    }
    const float left_mm = g_leg_interp[kLeft].go(
        g_leg_z_target_mm[kLeft], leg_time, now);
    const float right_mm = g_leg_interp[kRight].go(
        g_leg_z_target_mm[kRight], leg_time, now);
    if (!transmitStance(left_mm, right_mm,
                        g_lean_deg * tilt::DEG2RAD)) return;
    for (int leg = 0; leg < 2; ++leg) {
        if (measure_due[leg]) {
            measureLift(leg, g_bruton_lift_start_mm[leg],
                        g_bruton_lift_target_mm[leg]);
        }
    }
    if (g_leg_completed[kLeft] && g_leg_completed[kRight]) {
        g_leg_completed[kLeft] = g_leg_completed[kRight] = false;
        ++g_stats.bruton_cycles;
        printBrutonCycle();
    }
}

void startBruton() {
    if (!g_rock_ready.load() || !isArmed("BRUTON start")) return;
    if (g_ramp.active) {
        std::printf("Wait for the current move to finish.\n");
        return;
    }
    const float short_mm = g_base_mm - g_bruton_lift_mm;
    const float lean_rad = g_lean_deg * tilt::DEG2RAD;
    if (!validStance(g_base_mm, g_base_mm, lean_rad) ||
        !validStance(short_mm, g_base_mm, lean_rad) ||
        !validStance(g_base_mm, short_mm, lean_rad)) {
        std::printf("BRUTON start refused: unreachable base/short target.\n");
        return;
    }
    if (std::fabs(g_sent_height_mm[kLeft] - g_base_mm) > 0.01f ||
        std::fabs(g_sent_height_mm[kRight] - g_base_mm) > 0.01f ||
        std::fabs(g_sent_lean_rad - lean_rad) > 0.001f) {
        std::printf("Return to base with 0 before starting BRUTON.\n");
        return;
    }
    resetBrutonState(nowMs());
    g_bruton_running = true;
    std::printf("BRUTON RUNNING (right leg first). Space stops; ! E-STOPS.\n");
}

void stopBruton() {
    if (!g_bruton_running) return;
    g_bruton_running = false;
    for (int leg = 0; leg < 2; ++leg) {
        g_leg_trigger[leg] = false;
        g_leg_z_target_mm[leg] = g_sent_height_mm[leg];
        g_leg_interp[leg].reset(g_sent_height_mm[leg], nowMs());
    }
    std::printf("BRUTON stopped; returning both legs to base.\n");
    beginHeightRamp(g_base_mm, g_base_mm, g_lean_deg * tilt::DEG2RAD,
                    tilt_rock_test::kPoseDurationMs, RampKind::ZERO);
}

void stopContinuous() {
    if (!g_continuous.exchange(false)) return;
    g_completed_phases_in_cycle = 0;
    if (g_ramp.active && g_ramp.kind == RampKind::PHASE) {
        g_next_phase = g_ramp.phase;
        g_ramp.active = false;
    }
    std::printf("Continuous rocking stopped; last transmitted pose held.\n");
}

void startContinuous() {
    if (g_pattern != RockPattern::PHASE6 || !g_rock_ready.load() ||
        !isArmed("rock start")) return;
    if (g_ramp.active) {
        std::printf("Wait for the current phase/move to finish.\n");
        return;
    }
    if (!allPhasesReachable(true)) {
        std::printf("Continuous mode refused: adjust unreachable phase targets.\n");
        return;
    }
    g_cycle_roll = {};
    g_cycle_pitch = {};
    g_cycle_support_dx_mm = 0.0f;
    g_completed_phases_in_cycle = 0;
    g_retry_after_ms = nowMs();
    g_continuous.store(true);
    std::printf("Continuous rocking RUNNING. Space stops; ! E-STOPS.\n");
}

void stepOnePhase() {
    if (g_pattern == RockPattern::BRUTON) {
        std::printf("BRUTON has dual timers; n is unavailable.\n");
        return;
    }
    if (!g_rock_ready.load() || !isArmed("next phase")) return;
    if (g_continuous.load()) {
        std::printf("Press space to stop continuous mode before using n.\n");
        return;
    }
    if (g_ramp.active) {
        std::printf("Wait for the current phase to finish.\n");
        return;
    }
    startPhase(g_next_phase);
}

void refreshStoppedTarget() {
    if (g_continuous.load() || g_bruton_running ||
        !g_rock_ready.load()) return;
    if (g_pattern == RockPattern::BRUTON) {
        beginHeightRamp(g_base_mm, g_base_mm,
                        g_lean_deg * tilt::DEG2RAD,
                        tilt_rock_test::kPoseDurationMs, RampKind::ADJUST);
        return;
    }
    float left_mm = 0.0f;
    float right_mm = 0.0f;
    phaseTargets(g_current_phase, left_mm, right_mm);
    const std::uint32_t duration = g_current_phase < 0
        ? g_shift_duration_ms : phaseDurationMs(g_current_phase);
    beginHeightRamp(left_mm, right_mm, g_lean_deg * tilt::DEG2RAD,
                    duration, RampKind::ADJUST);
}

void resetRejectionWarnings() {
    for (bool& warned : g_phase_reject_warned) warned = false;
}

void adjustHeightParameter(CommandType type) {
    if (g_pattern == RockPattern::BRUTON &&
        (type == CommandType::SHIFT_UP || type == CommandType::SHIFT_DOWN)) {
        std::printf("BRUTON mode: shift keys are unused.\n");
        return;
    }
    const float old_base = g_base_mm;
    const float old_lift = g_bruton_lift_mm;
    const float step = tilt_rock_test::kHeightStepMm;
    switch (type) {
        case CommandType::BASE_UP:
            g_base_mm = std::min(g_base_mm + step,
                                 tilt_rock_test::kBaseMaxMm);
            break;
        case CommandType::BASE_DOWN:
            g_base_mm = std::max(g_base_mm - step,
                                 tilt_rock_test::kBaseMinMm);
            break;
        case CommandType::SHIFT_UP:
            g_shift_mm = std::min(g_shift_mm + step,
                                  tilt_rock_test::kShiftMaxMm);
            break;
        case CommandType::SHIFT_DOWN:
            g_shift_mm = std::max(g_shift_mm - step, 0.0f);
            break;
        case CommandType::LIFT_UP:
            if (g_pattern == RockPattern::BRUTON) {
                g_bruton_lift_mm = std::min(g_bruton_lift_mm + step,
                                            tilt_rock_test::kLiftMaxMm);
            } else {
                g_lift_mm = std::min(g_lift_mm + step,
                                     tilt_rock_test::kLiftMaxMm);
            }
            break;
        case CommandType::LIFT_DOWN:
            if (g_pattern == RockPattern::BRUTON) {
                g_bruton_lift_mm = std::max(g_bruton_lift_mm - step, 0.0f);
            } else {
                g_lift_mm = std::max(g_lift_mm - step, 0.0f);
            }
            break;
        default: return;
    }
    if (g_pattern == RockPattern::BRUTON &&
        (!validStance(g_base_mm, g_base_mm, g_lean_deg * tilt::DEG2RAD) ||
         !validStance(g_base_mm - g_bruton_lift_mm, g_base_mm,
                      g_lean_deg * tilt::DEG2RAD) ||
         !validStance(g_base_mm, g_base_mm - g_bruton_lift_mm,
                      g_lean_deg * tilt::DEG2RAD))) {
        g_base_mm = old_base;
        g_bruton_lift_mm = old_lift;
        std::printf("BRUTON parameter rejected: unreachable target.\n");
        return;
    }
    resetRejectionWarnings();
    if (g_pattern == RockPattern::BRUTON) {
        std::printf("base=%.1f lift=%.1f mm%s\n", g_base_mm,
                    g_bruton_lift_mm,
                    g_bruton_running ? " (next leg state)" : "");
        if (!g_bruton_running) refreshStoppedTarget();
    } else {
        std::printf("base=%.1f shift=%.1f lift=%.1f mm%s\n",
                    g_base_mm, g_shift_mm, g_lift_mm,
                    g_continuous.load() ? " (next phase)" : "");
        refreshStoppedTarget();
    }
}

void adjustLean(bool increase) {
    const float old_lean = g_lean_deg;
    const float step = tilt_rock_test::kLeanStepDeg;
    g_lean_deg = std::clamp(
        g_lean_deg + (increase ? step : -step),
        tilt_rock_test::kLeanMinDeg,
        tilt_rock_test::kLeanMaxDeg);
    if (g_pattern == RockPattern::BRUTON &&
        (!validStance(g_base_mm, g_base_mm, g_lean_deg * tilt::DEG2RAD) ||
         !validStance(g_base_mm - g_bruton_lift_mm, g_base_mm,
                      g_lean_deg * tilt::DEG2RAD) ||
         !validStance(g_base_mm, g_base_mm - g_bruton_lift_mm,
                      g_lean_deg * tilt::DEG2RAD))) {
        g_lean_deg = old_lean;
        std::printf("BRUTON lean rejected: unreachable target.\n");
        return;
    }
    resetRejectionWarnings();
    std::printf("lean=%+.1fdeg%s\n", g_lean_deg,
                g_continuous.load() ? " (next phase)" :
                (g_bruton_running ? " (next loop)" : ""));
    refreshStoppedTarget();
}

void adjustDuration(std::uint32_t& value, bool increase,
                    const char* name) {
    const std::uint32_t step = tilt_rock_test::kDurationStepMs;
    if (increase) {
        value = std::min(value + step, tilt_rock_test::kDurationMaxMs);
    } else {
        value = value > tilt_rock_test::kDurationMinMs + step
                    ? value - step : tilt_rock_test::kDurationMinMs;
    }
    std::printf("%s=%lums%s\n", name,
                static_cast<unsigned long>(value),
                g_continuous.load() ? " (next phase)" : "");
}

void adjustBoundedTime(std::uint32_t& value, bool increase,
                       std::uint32_t minimum, std::uint32_t maximum,
                       const char* name) {
    const std::uint32_t step = tilt_rock_test::kDurationStepMs;
    value = increase ? std::min(value + step, maximum)
                     : (value > minimum + step ? value - step : minimum);
    std::printf("%s=%lums\n", name, static_cast<unsigned long>(value));
}

void recordLift() {
    if (!g_rock_ready.load()) return;
    LiftRecord record{};
    record.phase = g_pattern == RockPattern::BRUTON
        ? (g_last_triggered_leg == kLeft ? 1 : 4) : g_current_phase;
    record.left_mm = g_sent_height_mm[kLeft];
    record.right_mm = g_sent_height_mm[kRight];
    record.roll_valid = g_roll_valid;
    record.roll_deg = g_roll_deg;
    if (g_stats.lift_count < kMaxLiftRecords) {
        g_stats.records[g_stats.lift_count] = record;
    }
    ++g_stats.lift_count;
    if (record.phase == 1) ++g_stats.left_lift_count;
    if (record.phase == 4) ++g_stats.right_lift_count;
    std::printf("Foot-lift observation #%lu: %s, L=%.1f R=%.1f, roll ",
                static_cast<unsigned long>(g_stats.lift_count),
                phaseName(record.phase), record.left_mm, record.right_mm);
    if (record.roll_valid) std::printf("%+.1fdeg.\n", record.roll_deg);
    else std::printf("--.\n");
    if (record.phase != 1 && record.phase != 4) {
        std::printf("Note: recorded outside LIFT_L/LIFT_R.\n");
    }
}

void resetImuZero() {
    if (!g_roll_valid || !g_pitch_valid) {
        std::printf("IMU attitude unavailable; display zero unchanged.\n");
        return;
    }
    g_roll_zero_deg = g_roll_deg;
    g_pitch_zero_deg = g_pitch_deg;
    g_imu_zero_valid = true;
    std::printf("IMU display zero reset: roll=%+.1f pitch=%+.1fdeg.\n",
                g_roll_zero_deg, g_pitch_zero_deg);
}

void returnToBase(RampKind kind) {
    if (g_continuous.load()) stopContinuous();
    g_bruton_running = false;
    g_ramp.active = false;
    if (beginHeightRamp(g_base_mm, g_base_mm,
                        g_lean_deg * tilt::DEG2RAD,
                        tilt_rock_test::kPoseDurationMs, kind)) {
        std::printf("Returning to base parallel stand.\n");
    }
}

void enterRockMode() {
    if (!isArmed("rock") || g_rock_mode.load()) return;
    if (!g_parallel_pose_valid) {
        std::printf("rock rejected: arm from a parallel-leg pose first.\n");
        return;
    }
    g_stats = {};
    g_lift_tracking[kLeft] = {};
    g_lift_tracking[kRight] = {};
    g_measure_read_failure_latched = false;
    g_bruton_running = false;
    g_pattern = RockPattern::PHASE6;
    g_current_phase = -1;
    g_next_phase = 0;
    resetRejectionWarnings();
    g_continuous.store(false);
    g_rock_mode.store(true);
    g_rock_ready.store(false);
    if (!beginHeightRamp(g_base_mm, g_base_mm,
                         g_lean_deg * tilt::DEG2RAD,
                         tilt_rock_test::kPoseDurationMs,
                         RampKind::ENTER)) {
        g_rock_mode.store(false);
        return;
    }
    std::printf("Entering rock mode: moving to base parallel stand first.\n");
}

void quitRockMode() {
    if (!g_rock_mode.load()) return;
    if (g_continuous.load()) stopContinuous();
    g_bruton_running = false;
    g_ramp.active = false;
    printRockSummary();
    returnToBase(RampKind::EXIT);
}

void printStatus() {
    std::printf("state=%s rock=%s parallel=%s\n",
                safetyName(g_safety.load()),
                g_rock_mode.load()
                    ? ((g_continuous.load() || g_bruton_running)
                        ? "running" : "stopped")
                    : "off",
                g_parallel_pose_valid ? "yes" : "no");
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const esp_err_t ping = g_bus.ping(tilt::SERVO_ID[joint]);
        std::uint16_t raw = 0;
        const esp_err_t read =
            g_bus.readPosition(tilt::SERVO_ID[joint], raw);
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

void printHelp() {
    std::printf(
        "\nTILT rock_test commands (press Enter)\n"
        "  help | status | arm | disarm | recover | !\n"
        "  stand [height_mm] [lean_deg] | rock\n"
        "Rock keys: m switches PHASE6/BRUTON while stopped.\n");
    printModeKeys();
    std::printf("\n");
}

void handleCommand(const Command& command) {
    switch (command.type) {
        case CommandType::HELP: printHelp(); return;
        case CommandType::STATUS: printStatus(); return;
        case CommandType::ARM: arm(); return;
        case CommandType::DISARM: disarm(); return;
        case CommandType::RECOVER: recover(); return;
        case CommandType::STAND: {
            if (!isArmed("stand")) return;
            if (command.height_mm < tilt_rock_test::kBaseMinMm ||
                command.height_mm > tilt_rock_test::kBaseMaxMm ||
                command.lean_deg < tilt_rock_test::kLeanMinDeg ||
                command.lean_deg > tilt_rock_test::kLeanMaxDeg) {
                std::printf("stand range: height %.1f..%.1fmm, lean %+.1f..%+.1fdeg.\n",
                            tilt_rock_test::kBaseMinMm,
                            tilt_rock_test::kBaseMaxMm,
                            tilt_rock_test::kLeanMinDeg,
                            tilt_rock_test::kLeanMaxDeg);
                return;
            }
            if (beginHeightRamp(command.height_mm, command.height_mm,
                                command.lean_deg * tilt::DEG2RAD,
                                tilt_rock_test::kPoseDurationMs,
                                RampKind::STAND)) {
                g_base_mm = command.height_mm;
                g_lean_deg = command.lean_deg;
                resetRejectionWarnings();
                std::printf("Moving to parallel stand: h=%.1fmm lean=%+.1fdeg.\n",
                            g_base_mm, g_lean_deg);
            }
            return;
        }
        case CommandType::ROCK_ENTER: enterRockMode(); return;
        case CommandType::ROCK_TOGGLE:
            if (g_pattern == RockPattern::BRUTON) {
                if (g_bruton_running) stopBruton();
                else startBruton();
            } else {
                if (g_continuous.load()) stopContinuous();
                else startContinuous();
            }
            return;
        case CommandType::ROCK_NEXT: stepOnePhase(); return;
        case CommandType::MODE_TOGGLE:
            if (!g_rock_ready.load()) {
                std::printf("Wait until rock mode is ready.\n");
            } else if (g_continuous.load() || g_bruton_running ||
                       g_ramp.active) {
                std::printf("Stop motion and wait for the move to finish before switching mode.\n");
            } else {
                g_pattern = g_pattern == RockPattern::PHASE6
                    ? RockPattern::BRUTON : RockPattern::PHASE6;
                std::printf("Rock pattern: %s.\n", patternName());
                printModeKeys();
            }
            return;
        case CommandType::IMU_TIMING_TOGGLE:
            if (g_pattern != RockPattern::BRUTON) {
                std::printf("IMU timing is BRUTON-only.\n");
            } else {
                g_imu_timing_enabled = !g_imu_timing_enabled;
                std::printf("BRUTON IMU timing %s%s.\n",
                            g_imu_timing_enabled ? "ON" : "OFF",
                            g_imu_timing_enabled && !g_pitch_valid
                                ? " (pitch unavailable; using base times)" : "");
            }
            return;
        case CommandType::BASE_UP:
        case CommandType::BASE_DOWN:
        case CommandType::SHIFT_UP:
        case CommandType::SHIFT_DOWN:
        case CommandType::LIFT_UP:
        case CommandType::LIFT_DOWN:
            adjustHeightParameter(command.type); return;
        case CommandType::SHIFT_TIME_DOWN:
            if (g_pattern == RockPattern::BRUTON)
                adjustBoundedTime(g_step_time_base_ms, false,
                    tilt_rock_test::kBrutonStepTimeMinMs,
                    tilt_rock_test::kBrutonStepTimeMaxMs, "step_time");
            else adjustDuration(g_shift_duration_ms, false, "t_shift");
            return;
        case CommandType::SHIFT_TIME_UP:
            if (g_pattern == RockPattern::BRUTON)
                adjustBoundedTime(g_step_time_base_ms, true,
                    tilt_rock_test::kBrutonStepTimeMinMs,
                    tilt_rock_test::kBrutonStepTimeMaxMs, "step_time");
            else adjustDuration(g_shift_duration_ms, true, "t_shift");
            return;
        case CommandType::LIFT_TIME_DOWN:
            if (g_pattern == RockPattern::BRUTON)
                adjustBoundedTime(g_leg_time_base_ms, false,
                    tilt_rock_test::kBrutonLegTimeMinMs,
                    tilt_rock_test::kBrutonLegTimeMaxMs, "leg_time");
            else adjustDuration(g_lift_duration_ms, false, "t_lift");
            return;
        case CommandType::LIFT_TIME_UP:
            if (g_pattern == RockPattern::BRUTON)
                adjustBoundedTime(g_leg_time_base_ms, true,
                    tilt_rock_test::kBrutonLegTimeMinMs,
                    tilt_rock_test::kBrutonLegTimeMaxMs, "leg_time");
            else adjustDuration(g_lift_duration_ms, true, "t_lift");
            return;
        case CommandType::PLANT_TIME_DOWN:
            if (g_pattern == RockPattern::BRUTON) {
                std::printf("BRUTON mode: this key is unused.\n"); return;
            }
            adjustDuration(g_plant_duration_ms, false, "t_plant"); return;
        case CommandType::PLANT_TIME_UP:
            if (g_pattern == RockPattern::BRUTON) {
                std::printf("BRUTON mode: this key is unused.\n"); return;
            }
            adjustDuration(g_plant_duration_ms, true, "t_plant"); return;
        case CommandType::LIFT_HOLD_DOWN:
        case CommandType::LIFT_HOLD_UP:
        case CommandType::PLANT_HOLD_DOWN:
        case CommandType::PLANT_HOLD_UP:
            if (g_pattern == RockPattern::BRUTON) {
                std::printf("BRUTON mode: this key is unused.\n"); return;
            }
            if (command.type == CommandType::LIFT_HOLD_DOWN ||
                command.type == CommandType::LIFT_HOLD_UP) {
                adjustBoundedTime(g_lift_hold_ms,
                    command.type == CommandType::LIFT_HOLD_UP, 0,
                    tilt_rock_test::kHoldMaxMs, "t_lift_hold");
            } else {
                adjustBoundedTime(g_plant_hold_ms,
                    command.type == CommandType::PLANT_HOLD_UP, 0,
                    tilt_rock_test::kHoldMaxMs, "t_plant_hold");
            }
            return;
        case CommandType::LEAN_DOWN: adjustLean(false); return;
        case CommandType::LEAN_UP: adjustLean(true); return;
        case CommandType::RECORD_LIFT: recordLift(); return;
        case CommandType::RESET_IMU: resetImuZero(); return;
        case CommandType::ROCK_STATUS: printRockStatus(); return;
        case CommandType::ROCK_ZERO: returnToBase(RampKind::ZERO); return;
        case CommandType::ROCK_QUIT: quitRockMode(); return;
    }
}

void enqueue(Command command) {
    if (g_queue == nullptr || xQueueSend(g_queue, &command, 0) != pdTRUE) {
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
    else if (strcasecmp(line, "rock") == 0) enqueue({CommandType::ROCK_ENTER});
    else if (strcasecmp(line, "stand") == 0) {
        enqueue({CommandType::STAND, g_base_mm, g_lean_deg});
    } else if (strncasecmp(line, "stand ", 6) == 0) {
        float height_mm = 0.0f;
        float lean_deg = g_lean_deg;
        char extra = '\0';
        const int count = std::sscanf(line + 6, "%f %f %c",
                                      &height_mm, &lean_deg, &extra);
        if (count == 1 || count == 2) {
            enqueue({CommandType::STAND, height_mm, lean_deg});
        } else {
            std::printf("Usage: stand [height_mm] [lean_deg].\n");
        }
    } else {
        std::printf("Unknown command. Type help.\n");
    }
}

void enqueueRockKey(std::uint8_t key) {
    switch (key) {
        case ' ': enqueue({CommandType::ROCK_TOGGLE}); break;
        case 'n': case 'N': enqueue({CommandType::ROCK_NEXT}); break;
        case 'w': case 'W': enqueue({CommandType::BASE_UP}); break;
        case 's': case 'S': enqueue({CommandType::BASE_DOWN}); break;
        case 'd': case 'D': enqueue({CommandType::SHIFT_UP}); break;
        case 'a': case 'A': enqueue({CommandType::SHIFT_DOWN}); break;
        case '+': enqueue({CommandType::LIFT_UP}); break;
        case '-': enqueue({CommandType::LIFT_DOWN}); break;
        case '1': enqueue({CommandType::SHIFT_TIME_DOWN}); break;
        case '2': enqueue({CommandType::SHIFT_TIME_UP}); break;
        case '3': enqueue({CommandType::LIFT_TIME_DOWN}); break;
        case '4': enqueue({CommandType::LIFT_TIME_UP}); break;
        case '5': enqueue({CommandType::PLANT_TIME_DOWN}); break;
        case '6': enqueue({CommandType::PLANT_TIME_UP}); break;
        case '[': enqueue({CommandType::LIFT_HOLD_DOWN}); break;
        case ']': enqueue({CommandType::LIFT_HOLD_UP}); break;
        case ';': enqueue({CommandType::PLANT_HOLD_DOWN}); break;
        case '\'': enqueue({CommandType::PLANT_HOLD_UP}); break;
        case 'm': case 'M': enqueue({CommandType::MODE_TOGGLE}); break;
        case 'i': case 'I': enqueue({CommandType::IMU_TIMING_TOGGLE}); break;
        case ',': enqueue({CommandType::LEAN_DOWN}); break;
        case '.': enqueue({CommandType::LEAN_UP}); break;
        case 'k': case 'K': enqueue({CommandType::RECORD_LIFT}); break;
        case 'r': case 'R': enqueue({CommandType::RESET_IMU}); break;
        case 'v': case 'V': enqueue({CommandType::ROCK_STATUS}); break;
        case '0': enqueue({CommandType::ROCK_ZERO}); break;
        case 'q': case 'Q': enqueue({CommandType::ROCK_QUIT}); break;
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
        if (usb_serial_jtag_read_bytes(&byte, 1,
                                       pdMS_TO_TICKS(50)) <= 0) continue;
        if (byte == '!') {
            emergencyStopNow();
            length = 0;
            escape = EscapeState::NONE;
            continue;
        }
        if (g_rock_mode.load()) {
            if (escape == EscapeState::ESC) {
                escape = byte == '[' ? EscapeState::BRACKET
                                     : EscapeState::NONE;
                continue;
            }
            if (escape == EscapeState::BRACKET) {
                if (byte == 'A') enqueue({CommandType::BASE_UP});
                else if (byte == 'B') enqueue({CommandType::BASE_DOWN});
                else if (byte == 'C') enqueue({CommandType::SHIFT_UP});
                else if (byte == 'D') enqueue({CommandType::SHIFT_DOWN});
                escape = EscapeState::NONE;
                continue;
            }
            if (byte == 0x1B) escape = EscapeState::ESC;
            else enqueueRockKey(byte);
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
        ESP_LOGW(kTag, "MPU6050 unavailable; rocking still works without attitude display.");
    }
    torqueOffBestEffort();
    std::printf("\nBoot complete: DISARMED, torque OFF, no automatic motion.\n");
    std::printf("Keep a physical servo-power disconnect within reach.\n");

    g_queue = xQueueCreate(kQueueDepth, sizeof(Command));
    if (g_queue == nullptr) {
        ESP_LOGE(kTag, "could not create console command queue");
        return;
    }
    xTaskCreate(consoleTask, "rock_console", 4096, nullptr, 5, nullptr);

    TickType_t wake = xTaskGetTickCount();
    while (true) {
        const std::uint32_t now = nowMs();
        serviceImu(now);
        Command command{};
        for (int processed = 0; processed < 8 &&
                                xQueueReceive(g_queue, &command, 0) == pdTRUE;
             ++processed) {
            handleCommand(command);
        }
        serviceRamp(now);
        serviceContinuous(now);
        serviceBruton(now);
        sampleStats();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(tilt_rock_test::kLoopPeriodMs));
    }
}
