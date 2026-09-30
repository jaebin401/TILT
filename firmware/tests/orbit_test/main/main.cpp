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
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "tilt/sts3215/Sts3215Bus.h"
#include "tilt_config.h"

#include "Bezier.h"
#include "GaitController.h"
#include "LegPose.h"
#include "OrbitConfig.h"
#include "OrbitModel.h"
#include "RollEstimator.h"

namespace {

constexpr char kTag[] = "orbit_test";
constexpr int kLeft = 0;
constexpr int kRight = 1;
constexpr float kPoseToleranceRad = 2.0f * tilt::DEG2RAD;
constexpr std::size_t kLineCapacity = 96;
constexpr UBaseType_t kQueueDepth = 32;

enum class Safety : std::uint8_t { DISARMED, ARMED, ESTOP };
enum class Cmd : std::uint8_t {
    HELP, STATUS, MODEL, ARM, DISARM, RECOVER, CHECK, CHECK_LEFT,
    CHECK_RIGHT, CHECK_TILT_START, STAND, GAIT_ENTER, MODE_ROCK, MODE_OPEN, MODE_ORBIT,
    TOGGLE, PERIOD_DOWN, PERIOD_UP, LIFT_DOWN, LIFT_UP, PUSH_DOWN,
    PUSH_UP, ENERGY, SHIFT_DOWN, SHIFT_UP, ZERO_ROLL, CSV,
    LIFT_MEASURE, VIEW, QUIT,
};
enum class RampKind : std::uint8_t {
    NONE, STAND, CHECK_UP, CHECK_RETURN, GAIT_STOP, GAIT_EXIT,
};
enum class CheckStage : std::uint8_t {
    NONE, MOVING_UP, ASK_SIDE, RETURNING, WAIT_TILT, TILT,
};

struct Command {
    Cmd type;
    float height_mm = 0.0f;
    float lean_deg = 0.0f;
};

struct Ramp {
    float from_mm[2]{};
    float to_mm[2]{};
    float from_lean = 0.0f;
    float to_lean = 0.0f;
    std::uint32_t started_ms = 0;
    std::uint32_t duration_ms = 1;
    RampKind kind = RampKind::NONE;
    bool active = false;
};

struct RunningStats {
    float roll_min = 0.0f;
    float roll_max = 0.0f;
    bool roll_seen = false;
    float rate_sum = 0.0f;
    std::uint32_t rate_count = 0;
    float lift_ratio_sum = 0.0f;
    float lift_ratio_min = 0.0f;
    std::uint32_t lift_count = 0;
    float clearance_sum = 0.0f;
    float clearance_min = 0.0f;
    std::uint32_t clearance_count = 0;
    float max_abs_x_drift_mm = 0.0f;
    std::uint32_t rejected_targets = 0;
    std::uint32_t speed_guards = 0;
    std::uint32_t loop_delays = 0;
    std::uint32_t max_loop_delay_ms = 0;
};

struct MeasureRequest {
    std::uint32_t sequence = 0;
    int logical_leg = -1;
    int knee_joint = -1;
    float command_lift_mm = 0.0f;
};

struct MeasureResult {
    std::uint32_t sequence = 0;
    float command_lift_mm = NAN;
    float actual_lift_mm = NAN;
    bool success = false;
};

constexpr tilt::sts3215::BusConfig kBusConfig{
    static_cast<uart_port_t>(tilt::SERVO_UART_PORT),
    static_cast<gpio_num_t>(tilt::SERVO_UART_TX_PIN),
    static_cast<gpio_num_t>(tilt::SERVO_UART_RX_PIN),
    tilt::SERVO_UART_BAUD, 50,
};

tilt::sts3215::Sts3215Bus g_bus(kBusConfig);
tilt_orbit::RollEstimator g_estimator;
tilt_orbit::GaitController g_gait;
QueueHandle_t g_queue = nullptr;
QueueHandle_t g_measure_request_queue = nullptr;
QueueHandle_t g_measure_result_queue = nullptr;
SemaphoreHandle_t g_motion_mutex = nullptr;
std::atomic<Safety> g_safety{Safety::DISARMED};
std::atomic<bool> g_gait_console{false};
std::atomic<bool> g_check_answer_pending{false};

bool g_swap_sides = tilt_orbit::kSwapLegSides;
bool g_check_passed = tilt_orbit::kMappingConfigured;
bool g_pose_known = false;
bool g_goal_valid = false;
bool g_speed_warned = false;
bool g_target_warned = false;
bool g_imu_warned = false;
bool g_csv = false;
std::atomic<bool> g_lift_measure_enabled{tilt_orbit::kLiftMeasureDefault};
int g_measure_consecutive_failures = 0;
std::uint32_t g_measure_sequence = 0;
std::atomic<std::uint32_t> g_measure_pending_sequence{0};
float g_goal_rad[tilt::NUM_JOINTS]{};
float g_sent_height_mm[2]{tilt::ZERO_POSE_HEIGHT_MM,
                           tilt::ZERO_POSE_HEIGHT_MM};
float g_sent_lean_rad = 0.0f;
float g_lean_deg = tilt_orbit::kLeanDefaultDeg;
Ramp g_ramp{};
CheckStage g_check_stage = CheckStage::NONE;
bool g_id11_is_left = true;
float g_check_roll_baseline = 0.0f;
float g_check_roll_largest_delta = 0.0f;
std::uint32_t g_check_tilt_started_ms = 0;
RunningStats g_stats{};
float g_last_lift_actual_mm = NAN;
float g_last_lift_command_mm = NAN;
float g_last_lift_ratio = NAN;
std::uint32_t g_rock_cycles = 0;
std::uint32_t g_last_loop_tick_ms = 0;
std::uint32_t g_last_goal_sent_ms = 0;
std::uint32_t g_rejected_imu_at_start = 0;
bool g_loop_delay_warned = false;

std::uint32_t nowMs() {
    return static_cast<std::uint32_t>(
        xTaskGetTickCount() * portTICK_PERIOD_MS);
}

void monitorLoopDelay(std::uint32_t now) {
    if (g_last_loop_tick_ms != 0) {
        const std::uint32_t dt_ms = now - g_last_loop_tick_ms;
        if (dt_ms > 2 * tilt_orbit::kLoopPeriodMs) {
            ++g_stats.loop_delays;
            g_stats.max_loop_delay_ms =
                std::max(g_stats.max_loop_delay_ms, dt_ms);
            if (!g_loop_delay_warned) {
                ESP_LOGW(kTag, "control loop delayed %lums (limit %lums); "
                         "further warnings suppressed",
                         static_cast<unsigned long>(dt_ms),
                         static_cast<unsigned long>(
                             2 * tilt_orbit::kLoopPeriodMs));
                g_loop_delay_warned = true;
            }
        }
    }
    g_last_loop_tick_ms = now;
}

std::uint32_t rejectedImuSinceStart() {
    return g_estimator.rejectedSamples() - g_rejected_imu_at_start;
}

class MotionGuard {
public:
    explicit MotionGuard(TickType_t wait_ticks = portMAX_DELAY)
        : locked_(g_motion_mutex != nullptr &&
          xSemaphoreTake(g_motion_mutex, wait_ticks) == pdTRUE) {}
    ~MotionGuard() {
        if (locked_) xSemaphoreGive(g_motion_mutex);
    }
    explicit operator bool() const { return locked_; }
    MotionGuard(const MotionGuard&) = delete;
    MotionGuard& operator=(const MotionGuard&) = delete;
private:
    bool locked_ = false;
};

const char* safetyName(Safety safety) {
    switch (safety) {
        case Safety::DISARMED: return "DISARMED";
        case Safety::ARMED: return "ARMED";
        case Safety::ESTOP: return "ESTOP";
    }
    return "?";
}

float tickToRad(int physical_joint, std::uint16_t tick) {
    return (static_cast<int>(tick) -
            static_cast<int>(tilt::ZERO_TICK[physical_joint])) *
           tilt::RAD_PER_TICK * tilt::JOINT_SIGN[physical_joint];
}

bool radToTick(int physical_joint, float rad, std::uint16_t& tick) {
    if (!std::isfinite(rad)) return false;
    const long raw = std::lround(rad / tilt::RAD_PER_TICK *
                    tilt::JOINT_SIGN[physical_joint]) +
                    tilt::ZERO_TICK[physical_joint];
    if (raw < tilt::SERVO_POS_MIN || raw > tilt::SERVO_POS_MAX) return false;
    tick = static_cast<std::uint16_t>(raw);
    return true;
}

void torqueOffBestEffort() {
    const esp_err_t result = g_bus.setTorqueAll(
        tilt::SERVO_ID, tilt::NUM_JOINTS, false);
    if (result != ESP_OK) ESP_LOGE(kTag, "torque OFF failed: %s; cut power",
                                    esp_err_to_name(result));
}

void emergencyStopNow() {
    g_safety.store(Safety::ESTOP);
    g_gait_console.store(false);
    g_check_answer_pending.store(false);
    MotionGuard guard;
    g_safety.store(Safety::ESTOP);
    const esp_err_t result = g_bus.emergencyStop(
        tilt::SERVO_ID, tilt::NUM_JOINTS);
    if (result != ESP_OK) ESP_LOGE(kTag, "E-STOP bus failed: %s; cut power",
                                    esp_err_to_name(result));
    std::printf("\n!! E-STOP requested. Cut servo power if necessary. !!\n");
}

bool requireArmed(const char* action) {
    if (g_safety.load() == Safety::ARMED) return true;
    std::printf("%s requires ARMED (now %s).\n", action,
                safetyName(g_safety.load()));
    return false;
}

bool physicalPose(float height0, float height1, float lean,
                  float out_rad[tilt::NUM_JOINTS]) {
    const float heights[2]{height0, height1};
    for (int physical = 0; physical < 2; ++physical) {
        const tilt_orbit::LegAngles pose =
            tilt_orbit::legForHeight(heights[physical], lean);
        if (!pose.reachable) return false;
        const float angles[3]{pose.yaw, pose.hip, pose.knee};
        for (int axis = 0; axis < 3; ++axis) {
            const float angle = angles[axis];
            const tilt::JointLimit& limit = tilt::JOINT_LIMIT[physical][axis];
            if (angle < limit.minimum_rad || angle > limit.maximum_rad)
                return false;
            out_rad[physical * 3 + axis] = angle;
        }
    }
    return true;
}

bool sendPhysical(float height0, float height1, float lean) {
    if (g_safety.load() != Safety::ARMED || !g_pose_known) return false;
    float target[tilt::NUM_JOINTS]{};
    std::uint16_t ticks[tilt::NUM_JOINTS]{};
    bool valid = physicalPose(height0, height1, lean, target);
    if (valid) {
        for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
            if (!radToTick(joint, target[joint], ticks[joint])) valid = false;
        }
    }
    if (!valid) {
        ++g_stats.rejected_targets;
        if (!g_target_warned)
            std::printf("Unreachable/joint-limit target rejected; last sent goal held.\n");
        g_target_warned = true;
        g_ramp.active = false;
        return false;
    }
    esp_err_t result = ESP_FAIL;
    {
        MotionGuard guard(0);
        if (!guard) {
            // A low-priority measurement may own the UART briefly. Never
            // block the 100 Hz gait loop; hold the last transmitted goal.
            return g_gait.running();
        }
        if (g_safety.load() != Safety::ARMED) return false;
        const std::uint32_t elapsed_since_send_ms = g_last_goal_sent_ms == 0
            ? tilt_orbit::kLoopPeriodMs
            : std::clamp(nowMs() - g_last_goal_sent_ms,
                         tilt_orbit::kLoopPeriodMs,
                         static_cast<std::uint32_t>(100));
        const float max_step_rad = tilt_orbit::kMaxJointVelocityDegS *
            elapsed_since_send_ms / 1000.0f * tilt::DEG2RAD;
        for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
            if (g_goal_valid && std::fabs(target[joint] - g_goal_rad[joint]) >
                max_step_rad) {
                if (!g_speed_warned) {
                    ESP_LOGW(kTag, "speed guard blocked %s (%+.2f deg/tick)",
                             tilt::JOINT_NAME[joint],
                             (target[joint] - g_goal_rad[joint]) /
                                 tilt::DEG2RAD);
                    ++g_stats.speed_guards;
                }
                g_speed_warned = true;
                g_gait.stop();
                g_ramp.active = false;
                std::printf("Motion stopped; last successfully transmitted goal held.\n");
                return false;
            }
        }
        result = g_bus.syncWritePositions(
            tilt::SERVO_ID, ticks, tilt::NUM_JOINTS,
            tilt_orbit::kServoSpeedRaw);
        if (result == ESP_OK) {
            std::memcpy(g_goal_rad, target, sizeof(target));
            g_goal_valid = true;
            g_last_goal_sent_ms = nowMs();
            g_sent_height_mm[0] = height0;
            g_sent_height_mm[1] = height1;
            g_sent_lean_rad = lean;
        }
    }
    if (result != ESP_OK) {
        ESP_LOGE(kTag, "syncWritePositions failed: %s", esp_err_to_name(result));
        emergencyStopNow();
        return false;
    }
    g_speed_warned = false;
    g_target_warned = false;
    return true;
}

bool sendLogical(float left_mm, float right_mm, float lean) {
    return g_swap_sides
        ? sendPhysical(right_mm, left_mm, lean)
        : sendPhysical(left_mm, right_mm, lean);
}

bool beginRamp(float height0, float height1, float lean,
               std::uint32_t duration_ms, RampKind kind) {
    if (!requireArmed("move") || !g_pose_known) {
        if (!g_pose_known) std::printf("Current pose is not a known parallel pose.\n");
        return false;
    }
    float target[tilt::NUM_JOINTS]{};
    if (!physicalPose(height0, height1, lean, target)) {
        std::printf("Ramp destination unreachable; last goal held.\n");
        ++g_stats.rejected_targets;
        return false;
    }
    g_ramp.from_mm[0] = g_sent_height_mm[0];
    g_ramp.from_mm[1] = g_sent_height_mm[1];
    g_ramp.to_mm[0] = height0;
    g_ramp.to_mm[1] = height1;
    g_ramp.from_lean = g_sent_lean_rad;
    g_ramp.to_lean = lean;
    g_ramp.started_ms = nowMs();
    g_ramp.duration_ms = std::max<std::uint32_t>(1, duration_ms);
    g_ramp.kind = kind;
    g_ramp.active = true;
    return true;
}

bool recognizeParallelPose() {
    float heights[2]{};
    float lean[2]{};
    for (int physical = 0; physical < 2; ++physical) {
        const int index = physical * 3;
        if (std::fabs(g_goal_rad[index]) > kPoseToleranceRad) return false;
        heights[physical] = tilt_orbit::heightFromKnee(g_goal_rad[index + 2]);
        const float a = g_goal_rad[index + 2] +
                        tilt::ANKLE_FIXED_RAD + tilt::KNEE_OFFSET_RAD;
        lean[physical] = g_goal_rad[index + 1] + a;
    }
    if (std::fabs(lean[0] - lean[1]) > kPoseToleranceRad) return false;
    const float common_lean = (lean[0] + lean[1]) / 2.0f;
    float reconstructed[tilt::NUM_JOINTS]{};
    if (!physicalPose(heights[0], heights[1], common_lean, reconstructed))
        return false;
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        if (std::fabs(reconstructed[joint] - g_goal_rad[joint]) >
            kPoseToleranceRad) return false;
    }
    g_sent_height_mm[0] = heights[0];
    g_sent_height_mm[1] = heights[1];
    g_sent_lean_rad = common_lean;
    return true;
}

void arm() {
    MotionGuard guard;
    if (!guard) return;
    if (g_safety.load() != Safety::DISARMED) {
        std::printf("arm requires DISARMED.\n"); return;
    }
    std::uint16_t raw[tilt::NUM_JOINTS]{};
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        if (g_bus.readPosition(tilt::SERVO_ID[joint], raw[joint]) != ESP_OK) {
            std::printf("arm rejected: failed to read %s.\n",
                        tilt::JOINT_NAME[joint]);
            return;
        }
        g_goal_rad[joint] = tickToRad(joint, raw[joint]);
    }
    g_goal_valid = true;
    g_pose_known = recognizeParallelPose();
    if (g_bus.syncWritePositions(tilt::SERVO_ID, raw, tilt::NUM_JOINTS,
            tilt_orbit::kServoSpeedRaw) != ESP_OK) {
        std::printf("arm rejected: hold goal write failed.\n"); return;
    }
    g_last_goal_sent_ms = nowMs();
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        if (g_bus.setAcceleration(tilt::SERVO_ID[joint],
            tilt_orbit::kServoAcceleration) != ESP_OK) {
            torqueOffBestEffort();
            std::printf("arm rejected: acceleration setup failed.\n"); return;
        }
    }
    if (g_safety.load() != Safety::DISARMED) {
        std::printf("arm interrupted by E-STOP.\n"); return;
    }
    if (g_bus.setTorqueAll(tilt::SERVO_ID, tilt::NUM_JOINTS, true) != ESP_OK) {
        torqueOffBestEffort();
        std::printf("arm rejected: torque enable failed.\n"); return;
    }
    if (g_safety.load() == Safety::ESTOP) {
        torqueOffBestEffort();
        std::printf("arm interrupted by E-STOP.\n"); return;
    }
    g_safety.store(Safety::ARMED);
    std::printf("ARMED: present pose held; parallel pose %s.\n",
                g_pose_known ? "recognized" : "NOT recognized");
    if (!g_pose_known) std::printf("Stand/gait/check blocked. Disarm and position near ADR-008 zero pose.\n");
}

void disarm() {
    MotionGuard guard;
    if (!guard) return;
    if (g_safety.load() == Safety::ESTOP) {
        std::printf("recover first; ESTOP remains latched.\n"); return;
    }
    g_gait.stop();
    g_gait_console.store(false);
    g_check_answer_pending.store(false);
    g_check_stage = CheckStage::NONE;
    g_ramp.active = false;
    g_pose_known = false;
    g_safety.store(Safety::DISARMED);
    torqueOffBestEffort();
    std::printf("DISARMED: torque OFF requested.\n");
}

void recover() {
    MotionGuard guard;
    if (!guard) return;
    if (g_safety.load() != Safety::ESTOP) {
        std::printf("recover requires ESTOP.\n"); return;
    }
    torqueOffBestEffort();
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        if (g_bus.ping(tilt::SERVO_ID[joint]) != ESP_OK) {
            std::printf("recover failed: %s did not respond.\n",
                        tilt::JOINT_NAME[joint]);
            return;
        }
    }
    g_gait.stop();
    g_ramp.active = false;
    g_check_stage = CheckStage::NONE;
    g_pose_known = false;
    g_speed_warned = false;
    g_safety.store(Safety::DISARMED);
    std::printf("Recovered to DISARMED; inspect robot before arm.\n");
}

void printModel() {
    const tilt_orbit::OrbitParams p = tilt_orbit::computeParams();
    const tilt_orbit::OrbitPrediction x = tilt_orbit::predict(g_gait.periodS());
    std::printf("H-LIP P2: k=%.2fmm z_eff=%.2fmm lambda=%.3f/s "
                "T*=%.3fs u=%.2fmm D=%.2fmm\n",
                p.k_mm, p.z_eff_mm, p.lambda, p.t_star_s,
                p.u_mm, p.d_mm);
    std::printf("T=%.3fs lambda*T/2=%.3f sigma2=%.3f/s "
                "p*=%.2fmm v*=%.2fmm/s p_mid=%.2fmm\n",
                x.t_s, x.lambda_half_t, x.sigma2,
                x.p_star_mm, x.v_star_mm_s, x.p_mid_mm);
    std::printf("predicted roll amplitude=%.2fdeg, crossing rate=%.1fdeg/s, "
                "opposite hip rise=%.2fmm\n",
                x.roll_amp_deg, x.roll_rate_target_deg_s, x.hip_rise_mm);
}

void printStatus() {
    std::printf("state=%s gait=%s/%s mapping=%s check=%s IMU=%s\n",
                safetyName(g_safety.load()),
                tilt_orbit::modeName(g_gait.mode()),
                tilt_orbit::stateName(g_gait.state()),
                g_swap_sides ? "swapped" : "normal",
                g_check_passed ? "passed" : "NOT passed",
                g_estimator.available() ? "available" : "unavailable");
    for (int joint = 0; joint < tilt::NUM_JOINTS; ++joint) {
        const esp_err_t ping = g_bus.ping(tilt::SERVO_ID[joint]);
        std::uint16_t raw = 0;
        const esp_err_t read = g_bus.readPosition(tilt::SERVO_ID[joint], raw);
        if (read == ESP_OK) {
            std::printf("  %-3s ID %u ping=%s raw=%u angle=%+.2fdeg\n",
                        tilt::JOINT_NAME[joint], tilt::SERVO_ID[joint],
                        ping == ESP_OK ? "OK" : "FAIL", raw,
                        tickToRad(joint, raw) * tilt_orbit::kRadToDeg);
        } else {
            std::printf("  %-3s ID %u ping=%s read=%s\n",
                        tilt::JOINT_NAME[joint], tilt::SERVO_ID[joint],
                        ping == ESP_OK ? "OK" : "FAIL", esp_err_to_name(read));
        }
    }
}

void printView() {
    const tilt_orbit::OrbitPrediction p = tilt_orbit::predict(g_gait.periodS());
    const tilt_orbit::RollState roll = g_estimator.state();
    std::printf("[%s %s] T=%lums lift zmax=%.1fmm (command apex ~%.1fmm) "
                "push=%.1fmm energy=%s startup shift=%.1fmm\n",
                tilt_orbit::modeName(g_gait.mode()),
                tilt_orbit::stateName(g_gait.state()),
                static_cast<unsigned long>(std::lround(g_gait.periodS() * 1000.0f)),
                g_gait.liftMm(),
                tilt_orbit::kApproxPeakFraction * g_gait.liftMm(),
                g_gait.pushMm(), g_gait.energyEnabled() ? "ON" : "OFF",
                g_gait.startupShiftMm());
    std::printf("  predicted roll amp %.2fdeg rate %.1fdeg/s rise %.2fmm; "
                "now roll %+.2fdeg rate %+.1fdeg/s lateral %+.1fmm (%s)\n",
                p.roll_amp_deg, p.roll_rate_target_deg_s, p.hip_rise_mm,
                roll.roll_deg, roll.rate_deg_s, roll.lateral_mm,
                roll.valid ? "valid" : "invalid");
    std::printf("  switches=%lu watchdogs=%lu check=%s mapping=%s "
                "lift measure=%s (fail streak %d)\n",
                static_cast<unsigned long>(g_gait.switches()),
                static_cast<unsigned long>(g_gait.watchdogs()),
                g_check_passed ? "passed" : "NOT passed",
                g_swap_sides ? "swapped" : "normal",
                g_lift_measure_enabled.load() ? "ON" : "OFF",
                g_measure_consecutive_failures);
    std::printf("  rejected=%lu speed_guard=%lu loop delays=%lu (max %lums) "
                "rejected IMU=%lu\n",
                static_cast<unsigned long>(g_stats.rejected_targets),
                static_cast<unsigned long>(g_stats.speed_guards),
                static_cast<unsigned long>(g_stats.loop_delays),
                static_cast<unsigned long>(g_stats.max_loop_delay_ms),
                static_cast<unsigned long>(rejectedImuSinceStart()));
}

void printSummary() {
    const tilt_orbit::OrbitPrediction p = tilt_orbit::predict(g_gait.periodS());
    std::printf("\n-- orbit result --\n");
    std::printf("  mode %s, T %.0fms, lift %.1fmm, %lu switches "
                "(watchdogs %lu)\n",
                tilt_orbit::modeName(g_gait.mode()),
                g_gait.periodS() * 1000.0f, g_gait.liftMm(),
                static_cast<unsigned long>(g_gait.switches()),
                static_cast<unsigned long>(g_gait.watchdogs()));
    std::printf("  predicted/measured roll amplitude %.2f / ", p.roll_amp_deg);
    if (g_stats.roll_seen)
        std::printf("%.2fdeg\n", (g_stats.roll_max - g_stats.roll_min) / 2.0f);
    else std::printf("--\n");
    std::printf("  predicted/measured crossing rate %.1f / ",
                p.roll_rate_target_deg_s);
    if (g_stats.rate_count)
        std::printf("%.1fdeg/s\n", g_stats.rate_sum / g_stats.rate_count);
    else std::printf("--\n");
    if (g_stats.lift_count)
        std::printf("  lift knee tracking mean %.0f%% min %.0f%% (n=%lu)\n",
                    100.0f * g_stats.lift_ratio_sum / g_stats.lift_count,
                    100.0f * g_stats.lift_ratio_min,
                    static_cast<unsigned long>(g_stats.lift_count));
    else std::printf("  lift knee tracking --\n");
    if (g_stats.clearance_count)
        std::printf("  estimated foot clearance mean %.1fmm min %.1fmm "
                    "(NOT measured ground clearance)\n",
                    g_stats.clearance_sum / g_stats.clearance_count,
                    g_stats.clearance_min);
    else std::printf("  estimated foot clearance --\n");
    std::printf("  max x drift %.1fmm (support-foot fore-aft motion from push)\n",
                g_stats.max_abs_x_drift_mm);
    std::printf("  final push delta %.1fmm, rejected targets %lu, "
                "speed guard %lu\n", g_gait.pushMm(),
                static_cast<unsigned long>(g_stats.rejected_targets),
                static_cast<unsigned long>(g_stats.speed_guards));
    std::printf("  loop delays %lu (max %lums), rejected IMU samples %lu\n\n",
                static_cast<unsigned long>(g_stats.loop_delays),
                static_cast<unsigned long>(g_stats.max_loop_delay_ms),
                static_cast<unsigned long>(rejectedImuSinceStart()));
}

void finishRamp(std::uint32_t now) {
    const RampKind kind = g_ramp.kind;
    g_ramp.active = false;
    g_ramp.kind = RampKind::NONE;
    if (kind == RampKind::STAND) {
        std::printf("Parallel stand complete: h=%.1fmm lean=%+.1fdeg.\n",
                    g_sent_height_mm[0], g_sent_lean_rad * tilt_orbit::kRadToDeg);
    } else if (kind == RampKind::CHECK_UP) {
        g_check_stage = CheckStage::ASK_SIDE;
        g_check_answer_pending.store(true);
        std::printf("ID 11-13 block shortened by 5mm. Which physical leg rose? "
                    "Robot LEFT=l, RIGHT=r.\n");
    } else if (kind == RampKind::CHECK_RETURN) {
        g_check_stage = CheckStage::WAIT_TILT;
        std::printf("Hold the suspended robot upright. Type tilt + Enter, "
                    "then tilt it toward its own LEFT for 2 seconds.\n> ");
        std::fflush(stdout);
    } else if (kind == RampKind::GAIT_STOP) {
        std::printf("Gait stopped at nominal parallel stand.\n");
    } else if (kind == RampKind::GAIT_EXIT) {
        g_gait_console.store(false);
        std::printf("Gait closed at nominal stand.\n> ");
        std::fflush(stdout);
    }
}

void serviceRamp(std::uint32_t now) {
    if (!g_ramp.active) return;
    if (g_safety.load() != Safety::ARMED) {
        g_ramp.active = false; return;
    }
    const float phase = std::min(1.0f,
        static_cast<float>(now - g_ramp.started_ms) / g_ramp.duration_ms);
    const float h0 = g_ramp.from_mm[0] +
        (g_ramp.to_mm[0] - g_ramp.from_mm[0]) * phase;
    const float h1 = g_ramp.from_mm[1] +
        (g_ramp.to_mm[1] - g_ramp.from_mm[1]) * phase;
    const float lean = g_ramp.from_lean +
        (g_ramp.to_lean - g_ramp.from_lean) * phase;
    if (!sendPhysical(h0, h1, lean)) return;
    if (phase >= 1.0f) finishRamp(now);
}

void beginCheck() {
    if (!requireArmed("check") || !g_pose_known ||
        g_gait.running() || g_ramp.active ||
        g_check_stage != CheckStage::NONE) {
        std::printf("check requires ARMED, stopped, known parallel pose.\n");
        return;
    }
    if (!g_estimator.state().valid) {
        std::printf("check requires a working IMU for roll-sign validation.\n");
        return;
    }
    if (std::fabs(g_sent_height_mm[0] - g_sent_height_mm[1]) > 1.0f) {
        std::printf("check requires equal leg heights; stand first.\n");
        return;
    }
    const float target = g_sent_height_mm[0] - 5.0f;
    std::printf("CHECK: suspend robot in air and keep servo power cutoff close. "
                "Moving ID 11-13 block 5mm shorter.\n");
    if (beginRamp(target, g_sent_height_mm[1], g_sent_lean_rad,
                  tilt_orbit::kStandDurationMs, RampKind::CHECK_UP)) {
        // A persisted non-default mapping remains trusted while a re-check is
        // in progress. A successful check below still replaces runtime values.
        g_check_passed = tilt_orbit::kMappingConfigured;
        g_check_stage = CheckStage::MOVING_UP;
    }
}

void answerCheck(bool left) {
    if (g_check_stage != CheckStage::ASK_SIDE) return;
    g_check_answer_pending.store(false);
    g_id11_is_left = left;
    const float original_height = g_ramp.from_mm[0];
    if (beginRamp(original_height, g_sent_height_mm[1], g_sent_lean_rad,
                  tilt_orbit::kStandDurationMs, RampKind::CHECK_RETURN)) {
        g_check_stage = CheckStage::RETURNING;
        std::printf("Returning ID 11-13 block to its prior height.\n");
    } else {
        g_check_stage = CheckStage::NONE;
    }
}

void serviceCheck(std::uint32_t now) {
    if (g_check_stage != CheckStage::TILT) return;
    if (g_safety.load() != Safety::ARMED) {
        g_check_stage = CheckStage::NONE; return;
    }
    const tilt_orbit::RollState roll = g_estimator.state();
    if (roll.valid) {
        const float delta = g_estimator.rawRollDeg() - g_check_roll_baseline;
        if (std::fabs(delta) > std::fabs(g_check_roll_largest_delta))
            g_check_roll_largest_delta = delta;
    }
    if (now - g_check_tilt_started_ms < 2000) return;
    g_check_stage = CheckStage::NONE;
    if (std::fabs(g_check_roll_largest_delta) < 1.0f) {
        std::printf("check incomplete: left tilt was not detected (less than 1deg). "
                    "Repeat check; persisted config remains in effect.\n");
        return;
    }
    const bool checked_swap_sides = !g_id11_is_left;
    const int sign = g_check_roll_largest_delta >= 0.0f ? +1 : -1;
    const bool differs_from_config =
        checked_swap_sides != tilt_orbit::kSwapLegSides ||
        sign != tilt_orbit::kRollSign;
    g_swap_sides = checked_swap_sides;
    g_estimator.setSign(sign);
    g_check_passed = true;
    std::printf("check result:\n  ID 11-13 = robot %s leg -> "
                "kSwapLegSides = %s\n  left tilt raw roll %+.1fdeg -> "
                "kRollSign = %+d\nRuntime values applied. Edit OrbitConfig.h "
                "to retain after reboot.\n",
                g_id11_is_left ? "LEFT" : "RIGHT",
                g_swap_sides ? "true" : "false",
                g_check_roll_largest_delta, sign);
    if (differs_from_config)
        std::printf("WARNING: check result differs from OrbitConfig.h. "
                    "Update the header.\n");
}

void beginTiltMeasurement() {
    if (g_check_stage != CheckStage::WAIT_TILT) {
        std::printf("tilt requires the roll-sign stage of check.\n");
        return;
    }
    if (!g_estimator.state().valid ||
        std::fabs(g_estimator.state().rate_deg_s) > 5.0f) {
        std::printf("Hold upright and still, then type tilt again.\n");
        return;
    }
    g_check_roll_baseline = g_estimator.rawRollDeg();
    g_check_roll_largest_delta = 0.0f;
    g_check_tilt_started_ms = nowMs();
    g_check_stage = CheckStage::TILT;
    std::printf("Measuring for 2s. Tilt LEFT now.\n");
}

void startOrStopGait() {
    if (!g_gait_console.load() || !requireArmed("gait")) return;
    if (g_gait.running()) {
        g_gait.stop();
        if (beginRamp(tilt_orbit::kNominalHeightMm,
                      tilt_orbit::kNominalHeightMm,
                      g_lean_deg * tilt::DEG2RAD,
                      tilt_orbit::kStandDurationMs, RampKind::GAIT_STOP)) {
            std::printf("Stopping gait; returning both legs to nominal stand.\n");
        }
        return;
    }
    if (g_ramp.active || g_check_stage != CheckStage::NONE) {
        std::printf("Wait for the current move/check to finish.\n"); return;
    }
    if (g_gait.mode() == tilt_orbit::GaitMode::ORBIT && !g_check_passed) {
        std::printf("ORBIT refused: suspended-side/roll check has not passed "
                    "this boot. Exit gait and run check.\n");
        return;
    }
    if (!g_check_passed)
        std::printf("WARNING: physical left/right mapping is unverified. "
                    "ROCK/OPEN can move the wrong leg. Run suspended check.\n");
    if (std::fabs(g_sent_height_mm[0] - tilt_orbit::kNominalHeightMm) > 0.1f ||
        std::fabs(g_sent_height_mm[1] - tilt_orbit::kNominalHeightMm) > 0.1f) {
        std::printf("Start requires nominal stand %.1fmm. Exit gait and stand.\n",
                    tilt_orbit::kNominalHeightMm);
        return;
    }
    g_stats = {};
    g_rock_cycles = 0;
    g_last_lift_actual_mm = g_last_lift_command_mm = g_last_lift_ratio = NAN;
    g_imu_warned = false;
    g_measure_consecutive_failures = 0;
    g_measure_pending_sequence = 0;
    g_loop_delay_warned = false;
    g_last_loop_tick_ms = nowMs();
    g_rejected_imu_at_start = g_estimator.rejectedSamples();
    g_gait.start(nowMs());
    std::printf("%s STARTUP: building lateral orbit; no foot lifting yet.\n",
                tilt_orbit::modeName(g_gait.mode()));
}

void toggleLiftMeasurement() {
    const bool enabled = !g_lift_measure_enabled.load();
    g_lift_measure_enabled.store(enabled);
    g_measure_pending_sequence.store(0);
    g_measure_consecutive_failures = 0;
    g_last_lift_actual_mm = g_last_lift_command_mm = g_last_lift_ratio = NAN;
    std::printf("Lift measurement %s%s\n", enabled ? "ON" : "OFF",
                enabled ? " (single async read; auto-OFF after 5 failures)."
                        : ".");
}

void recordMeasurementFailure() {
    g_last_lift_actual_mm = g_last_lift_command_mm = g_last_lift_ratio = NAN;
    ++g_measure_consecutive_failures;
    if (g_measure_consecutive_failures <
        tilt_orbit::kLiftMeasureMaxConsecutiveFailures) return;
    g_lift_measure_enabled.store(false);
    g_measure_pending_sequence = 0;
    std::printf("WARNING: lift measurement failed %d consecutive times; "
                "automatically disabled. Press k to retry.\n",
                tilt_orbit::kLiftMeasureMaxConsecutiveFailures);
}

void requestSwingMeasurement(int logical_leg, float command_height) {
    g_last_lift_actual_mm = g_last_lift_command_mm = g_last_lift_ratio = NAN;
    if (!g_lift_measure_enabled.load()) return;
    const int physical = g_swap_sides ? 1 - logical_leg : logical_leg;
    const int knee_joint = physical * 3 + 2;
    const float command_lift = tilt_orbit::kNominalHeightMm - command_height;
    if (command_lift < 0.5f) return;
    MeasureRequest request{};
    request.sequence = ++g_measure_sequence;
    request.logical_leg = logical_leg;
    request.knee_joint = knee_joint;
    request.command_lift_mm = command_lift;
    g_measure_pending_sequence.store(request.sequence);
    if (g_measure_request_queue == nullptr ||
        xQueueSend(g_measure_request_queue, &request, 0) != pdTRUE) {
        g_measure_pending_sequence.store(0);
        recordMeasurementFailure();
    }
}

void serviceMeasurementResult() {
    if (g_measure_result_queue == nullptr) return;
    MeasureResult result{};
    while (xQueueReceive(g_measure_result_queue, &result, 0) == pdTRUE) {
        if (result.sequence != g_measure_pending_sequence.load()) continue;
        g_measure_pending_sequence.store(0);
        if (!g_lift_measure_enabled.load() || !result.success ||
            !std::isfinite(result.actual_lift_mm) ||
            !std::isfinite(result.command_lift_mm) ||
            result.command_lift_mm < 0.5f) {
            recordMeasurementFailure();
            continue;
        }
        const float ratio = result.actual_lift_mm / result.command_lift_mm;
        if (!std::isfinite(ratio)) {
            recordMeasurementFailure();
            continue;
        }
        g_measure_consecutive_failures = 0;
        g_last_lift_actual_mm = result.actual_lift_mm;
        g_last_lift_command_mm = result.command_lift_mm;
        g_last_lift_ratio = ratio;
        g_stats.lift_ratio_sum += ratio;
        g_stats.lift_ratio_min = g_stats.lift_count == 0
            ? ratio : std::min(g_stats.lift_ratio_min, ratio);
        ++g_stats.lift_count;
    }
}

void measurementTask(void*) {
    MeasureRequest request{};
    while (true) {
        if (xQueueReceive(g_measure_request_queue, &request,
                          portMAX_DELAY) != pdTRUE) continue;
        MeasureResult result{};
        result.sequence = request.sequence;
        result.command_lift_mm = request.command_lift_mm;
        if (g_lift_measure_enabled.load() &&
            request.sequence == g_measure_pending_sequence.load() &&
            g_safety.load() == Safety::ARMED) {
            MotionGuard guard(0);
            if (guard) {
                std::uint16_t raw = 0;
                if (g_bus.readPosition(tilt::SERVO_ID[request.knee_joint],
                                       raw) == ESP_OK) {
                    const float actual_height = tilt_orbit::heightFromKnee(
                        tickToRad(request.knee_joint, raw));
                    result.actual_lift_mm =
                        tilt_orbit::kNominalHeightMm - actual_height;
                    result.success = std::isfinite(result.actual_lift_mm);
                }
            }
        }
        xQueueOverwrite(g_measure_result_queue, &result);
    }
}

void printGaitEvent(const tilt_orbit::GaitEvent& event) {
    if (event.kind == tilt_orbit::GaitEventKind::NONE) return;
    if (event.kind == tilt_orbit::GaitEventKind::STARTUP_WARNING) {
        std::printf("STARTUP: model roll threshold not reached after %d cycles; "
                    "continuing rocking without foot lift.\n",
                    tilt_orbit::kStartupMaxCycles);
        return;
    }
    if (event.kind == tilt_orbit::GaitEventKind::STARTUP_READY) {
        std::printf("STARTUP threshold reached twice; entering %s SSP.\n",
                    tilt_orbit::modeName(event.mode));
        return;
    }
    const tilt_orbit::OrbitPrediction prediction =
        tilt_orbit::predict(g_gait.periodS());
    if (event.kind == tilt_orbit::GaitEventKind::ROCK_CYCLE) {
        const float pp = event.maximum_deg - event.minimum_deg;
        ++g_rock_cycles;
        if (g_csv) return;
        std::printf("rock#%lu half-period measured %lums (set %.0f) "
                    "roll p-p %.1fdeg (pred %.1f) ratio %.2f\n",
                    static_cast<unsigned long>(g_rock_cycles),
                    static_cast<unsigned long>(event.elapsed_ms),
                    g_gait.periodS() * 1000.0f, pp,
                    2.0f * prediction.roll_amp_deg,
                    pp / (2.0f * prediction.roll_amp_deg));
        return;
    }
    if (event.kind != tilt_orbit::GaitEventKind::HALF_CYCLE) return;
    g_stats.rate_sum += std::fabs(event.rate_deg_s);
    ++g_stats.rate_count;
    const float clearance = std::isfinite(g_last_lift_actual_mm)
        ? g_last_lift_actual_mm + tilt_orbit::computeParams().d_mm *
            std::sin(std::fabs(event.peak_deg) * tilt::DEG2RAD)
        : NAN;
    if (std::isfinite(clearance)) {
        g_stats.clearance_sum += clearance;
        g_stats.clearance_min = g_stats.clearance_count == 0
            ? clearance : std::min(g_stats.clearance_min, clearance);
        ++g_stats.clearance_count;
    }
    const float a = std::acos((tilt_orbit::kNominalHeightMm -
        tilt_orbit::calfVerticalMm()) / tilt::THIGH_LENGTH_MM);
    const float x_drift = event.delta_mm * std::cos(a) / std::sin(a);
    g_stats.max_abs_x_drift_mm = std::max(
        g_stats.max_abs_x_drift_mm, std::fabs(x_drift));
    const char stance = event.stance == tilt_orbit::GaitState::SSP_LEFT
        ? 'L' : 'R';
    if (g_csv) {
        g_last_lift_actual_mm = g_last_lift_command_mm = g_last_lift_ratio = NAN;
        return;
    }
    std::printf("#%lu [%s] stance=%c T=%lums rate_x=%+.0fdeg/s "
                "(target %.0f) peak=%+.1fdeg (pred %.1f)\n",
                static_cast<unsigned long>(g_gait.switches()),
                tilt_orbit::modeName(event.mode), stance,
                static_cast<unsigned long>(event.elapsed_ms),
                event.rate_deg_s, prediction.roll_rate_target_deg_s,
                event.peak_deg, prediction.roll_amp_deg);
    std::printf("    delta=%.1fmm lift command=", event.delta_mm);
    if (std::isfinite(g_last_lift_command_mm))
        std::printf("%.1f actual=%.1fmm (%.0f%%)",
                    g_last_lift_command_mm, g_last_lift_actual_mm,
                    100.0f * g_last_lift_ratio);
    else std::printf("-- actual=--");
    if (std::isfinite(clearance))
        std::printf(" estimated clearance=%.1fmm", clearance);
    else std::printf(" estimated clearance=--");
    std::printf(" x drift=%.1fmm%s\n", x_drift,
                event.watchdog ? " WD" : "");
    g_last_lift_actual_mm = g_last_lift_command_mm = g_last_lift_ratio = NAN;
}

void serviceGait(std::uint32_t now) {
    if (!g_gait.running() || g_ramp.active ||
        g_safety.load() != Safety::ARMED) return;
    const tilt_orbit::RollState roll = g_estimator.state();
    if (!roll.valid && !g_imu_warned) {
        std::printf("IMU unavailable: no automatic stop; ORBIT will use "
                    "watchdog timer switches.\n");
        g_imu_warned = true;
    } else if (roll.valid) {
        g_imu_warned = false;
    }
    if (roll.valid) {
        if (!g_stats.roll_seen) {
            g_stats.roll_min = g_stats.roll_max = roll.roll_deg;
            g_stats.roll_seen = true;
        } else {
            g_stats.roll_min = std::min(g_stats.roll_min, roll.roll_deg);
            g_stats.roll_max = std::max(g_stats.roll_max, roll.roll_deg);
        }
    }
    const tilt_orbit::GaitOutput out = g_gait.update(now, roll);
    if (!sendLogical(out.left_mm, out.right_mm,
                     g_lean_deg * tilt::DEG2RAD)) return;
    if (out.measure_swing && out.swing_leg >= 0) {
        const float swing_height = out.swing_leg == kLeft
            ? out.left_mm : out.right_mm;
        requestSwingMeasurement(out.swing_leg, swing_height);
    }
    printGaitEvent(out.event);
    if (g_csv) {
        std::printf("%lu,%s,%s,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                    static_cast<unsigned long>(now),
                    tilt_orbit::modeName(g_gait.mode()),
                    tilt_orbit::stateName(g_gait.state()),
                    roll.valid ? roll.roll_deg : NAN,
                    roll.valid ? roll.rate_deg_s : NAN,
                    out.left_mm, out.right_mm, out.delta_mm);
    }
}

void printHelp() {
    std::printf("\nTILT orbit_test (line commands + Enter):\n"
                "  help status model arm disarm recover !\n"
                "  check   (suspended: identify ID 11-13 physical side, then roll sign)\n"
                "  tilt    (only when check asks; start 2s roll-sign measurement)\n"
                "  stand [height_mm] [lean_deg]   gait\n"
                "Gait keys, no Enter: 1 ROCK / 2 OPEN / 3 ORBIT (stopped),\n"
                "  space run/stop, [/] T, +/- lift, ,/. push, e energy,\n"
                "  s/S startup shift, r zero roll (stopped), c CSV, k lift measure,\n"
                "  m model, v view, q exit, ! E-STOP.\n"
                "Do not start on the floor before suspended check.\n\n");
}

void handleCommand(const Command& cmd) {
    switch (cmd.type) {
        case Cmd::HELP: printHelp(); return;
        case Cmd::STATUS: printStatus(); return;
        case Cmd::MODEL: printModel(); return;
        case Cmd::ARM: arm(); return;
        case Cmd::DISARM: disarm(); return;
        case Cmd::RECOVER: recover(); return;
        case Cmd::CHECK: beginCheck(); return;
        case Cmd::CHECK_LEFT: answerCheck(true); return;
        case Cmd::CHECK_RIGHT: answerCheck(false); return;
        case Cmd::CHECK_TILT_START: beginTiltMeasurement(); return;
        case Cmd::STAND: {
            if (!requireArmed("stand") || g_gait.running() ||
                g_check_stage != CheckStage::NONE) {
                std::printf("stand requires stopped gait and no active check.\n");
                return;
            }
            if (cmd.height_mm < tilt_orbit::kHeightMinMm ||
                cmd.height_mm > tilt_orbit::kHeightMaxMm ||
                cmd.lean_deg < tilt_orbit::kLeanMinDeg ||
                cmd.lean_deg > tilt_orbit::kLeanMaxDeg) {
                std::printf("stand range: h %.0f..%.0fmm lean %+.0f..%+.0fdeg.\n",
                            tilt_orbit::kHeightMinMm,
                            tilt_orbit::kHeightMaxMm,
                            tilt_orbit::kLeanMinDeg,
                            tilt_orbit::kLeanMaxDeg);
                return;
            }
            if (beginRamp(cmd.height_mm, cmd.height_mm,
                          cmd.lean_deg * tilt::DEG2RAD,
                          tilt_orbit::kStandDurationMs, RampKind::STAND)) {
                g_lean_deg = cmd.lean_deg;
                std::printf("Moving to parallel stand %.1fmm %+.1fdeg.\n",
                            cmd.height_mm, cmd.lean_deg);
            }
            return;
        }
        case Cmd::GAIT_ENTER:
            if (!requireArmed("gait") || !g_pose_known ||
                g_ramp.active || g_check_stage != CheckStage::NONE) {
                std::printf("gait requires ARMED, known parallel pose, "
                            "and no active move/check.\n");
                return;
            }
            g_gait_console.store(true);
            std::printf("Gait mode ready, STOPPED. Select 1/2/3; space starts.\n");
            printView();
            return;
        case Cmd::MODE_ROCK:
        case Cmd::MODE_OPEN:
        case Cmd::MODE_ORBIT: {
            const tilt_orbit::GaitMode mode = cmd.type == Cmd::MODE_ROCK
                ? tilt_orbit::GaitMode::ROCK
                : (cmd.type == Cmd::MODE_OPEN
                    ? tilt_orbit::GaitMode::OPEN
                    : tilt_orbit::GaitMode::ORBIT);
            if (!g_gait.selectMode(mode))
                std::printf("Stop gait before selecting a mode.\n");
            else
                std::printf("Selected %s%s.\n", tilt_orbit::modeName(mode),
                    mode == tilt_orbit::GaitMode::ORBIT && !g_check_passed
                    ? " (locked until check passes)" : "");
            return;
        }
        case Cmd::TOGGLE: startOrStopGait(); return;
        case Cmd::PERIOD_DOWN:
        case Cmd::PERIOD_UP:
            g_gait.setPeriodS(g_gait.periodS() +
                (cmd.type == Cmd::PERIOD_UP ? +1.0f : -1.0f) *
                    tilt_orbit::kTimeStepMs / 1000.0f);
            std::printf("T=%.0fms\n", g_gait.periodS() * 1000.0f);
            return;
        case Cmd::LIFT_DOWN:
        case Cmd::LIFT_UP:
            g_gait.setLiftMm(g_gait.liftMm() +
                (cmd.type == Cmd::LIFT_UP ? +0.5f : -0.5f));
            std::printf("lift zmax=%.1fmm, command apex ~%.1fmm\n",
                        g_gait.liftMm(),
                        tilt_orbit::kApproxPeakFraction * g_gait.liftMm());
            return;
        case Cmd::PUSH_DOWN:
        case Cmd::PUSH_UP:
            g_gait.setPushMm(g_gait.pushMm() +
                (cmd.type == Cmd::PUSH_UP ? +0.5f : -0.5f));
            std::printf("push delta=%.1fmm\n", g_gait.pushMm());
            return;
        case Cmd::ENERGY:
            if (g_gait.mode() != tilt_orbit::GaitMode::ORBIT) {
                std::printf("Energy regulation is ORBIT-only.\n"); return;
            }
            g_gait.toggleEnergy();
            std::printf("ORBIT energy regulation %s.\n",
                        g_gait.energyEnabled() ? "ON" : "OFF");
            return;
        case Cmd::SHIFT_DOWN:
        case Cmd::SHIFT_UP:
            g_gait.setStartupShiftMm(g_gait.startupShiftMm() +
                (cmd.type == Cmd::SHIFT_UP ? +0.5f : -0.5f));
            std::printf("startup shift=%.1fmm\n", g_gait.startupShiftMm());
            return;
        case Cmd::ZERO_ROLL:
            if (g_gait.running() || g_ramp.active ||
                g_check_stage != CheckStage::NONE) {
                std::printf("Stop and stand still before resetting roll zero.\n");
            } else if (g_estimator.zero()) {
                std::printf("Roll zero reset while stationary.\n");
            } else {
                std::printf("Roll zero rejected: IMU invalid or motion >5deg/s.\n");
            }
            return;
        case Cmd::CSV:
            g_csv = !g_csv;
            std::printf("CSV %s.\n", g_csv ? "ON" : "OFF");
            if (g_csv) std::printf("t_ms,mode,state,roll_deg,rate_deg_s,h_L,h_R,delta\n");
            return;
        case Cmd::LIFT_MEASURE: toggleLiftMeasurement(); return;
        case Cmd::VIEW: printView(); return;
        case Cmd::QUIT:
            g_gait.stop();
            printSummary();
            g_csv = false;
            if (g_safety.load() != Safety::ARMED) {
                g_gait_console.store(false); return;
            }
            g_ramp.active = false;
            if (beginRamp(tilt_orbit::kNominalHeightMm,
                          tilt_orbit::kNominalHeightMm,
                          g_lean_deg * tilt::DEG2RAD,
                          tilt_orbit::kStandDurationMs,
                          RampKind::GAIT_EXIT)) {
                std::printf("Returning to nominal stand before leaving gait.\n");
            }
            return;
    }
}

void enqueue(Command cmd) {
    if (g_queue == nullptr || xQueueSend(g_queue, &cmd, 0) != pdTRUE)
        ESP_LOGW(kTag, "console command queue full");
}

void parseLine(char* line) {
    while (*line == ' ' || *line == '\t') ++line;
    if (strcasecmp(line, "help") == 0) enqueue({Cmd::HELP});
    else if (strcasecmp(line, "status") == 0) enqueue({Cmd::STATUS});
    else if (strcasecmp(line, "model") == 0) enqueue({Cmd::MODEL});
    else if (strcasecmp(line, "arm") == 0) enqueue({Cmd::ARM});
    else if (strcasecmp(line, "disarm") == 0) enqueue({Cmd::DISARM});
    else if (strcasecmp(line, "recover") == 0) enqueue({Cmd::RECOVER});
    else if (strcasecmp(line, "check") == 0) enqueue({Cmd::CHECK});
    else if (strcasecmp(line, "tilt") == 0) enqueue({Cmd::CHECK_TILT_START});
    else if (strcasecmp(line, "gait") == 0) enqueue({Cmd::GAIT_ENTER});
    else if (strcasecmp(line, "stand") == 0)
        enqueue({Cmd::STAND, tilt_orbit::kNominalHeightMm, 0.0f});
    else if (strncasecmp(line, "stand ", 6) == 0) {
        float h = 0.0f;
        float lean = 0.0f;
        char extra = '\0';
        const int count = std::sscanf(line + 6, "%f %f %c", &h, &lean, &extra);
        if (count == 1 || count == 2) enqueue({Cmd::STAND, h, lean});
        else std::printf("Usage: stand [height_mm] [lean_deg].\n");
    } else std::printf("Unknown command. Type help.\n");
}

void enqueueGaitKey(std::uint8_t key) {
    switch (key) {
        case '1': enqueue({Cmd::MODE_ROCK}); break;
        case '2': enqueue({Cmd::MODE_OPEN}); break;
        case '3': enqueue({Cmd::MODE_ORBIT}); break;
        case ' ': enqueue({Cmd::TOGGLE}); break;
        case '[': enqueue({Cmd::PERIOD_DOWN}); break;
        case ']': enqueue({Cmd::PERIOD_UP}); break;
        case '+': enqueue({Cmd::LIFT_UP}); break;
        case '-': enqueue({Cmd::LIFT_DOWN}); break;
        case ',': enqueue({Cmd::PUSH_DOWN}); break;
        case '.': enqueue({Cmd::PUSH_UP}); break;
        case 'e': case 'E': enqueue({Cmd::ENERGY}); break;
        case 's': enqueue({Cmd::SHIFT_DOWN}); break;
        case 'S': enqueue({Cmd::SHIFT_UP}); break;
        case 'r': case 'R': enqueue({Cmd::ZERO_ROLL}); break;
        case 'c': case 'C': enqueue({Cmd::CSV}); break;
        case 'k': case 'K': enqueue({Cmd::LIFT_MEASURE}); break;
        case 'm': case 'M': enqueue({Cmd::MODEL}); break;
        case 'v': case 'V': enqueue({Cmd::VIEW}); break;
        case 'q': case 'Q': enqueue({Cmd::QUIT}); break;
        default: break;
    }
}

void consoleTask(void*) {
    char line[kLineCapacity]{};
    std::size_t length = 0;
    printHelp();
    std::printf("> ");
    std::fflush(stdout);
    while (true) {
        std::uint8_t byte = 0;
        if (usb_serial_jtag_read_bytes(&byte, 1, pdMS_TO_TICKS(50)) <= 0)
            continue;
        if (byte == '!') {
            emergencyStopNow();
            length = 0;
            continue;
        }
        if (g_check_answer_pending.load()) {
            if (byte == 'l' || byte == 'L')
                enqueue({Cmd::CHECK_LEFT});
            else if (byte == 'r' || byte == 'R')
                enqueue({Cmd::CHECK_RIGHT});
            continue;
        }
        if (g_gait_console.load()) {
            enqueueGaitKey(byte);
            continue;
        }
        if (byte == '\r' || byte == '\n') {
            if (length) {
                line[length] = '\0';
                parseLine(line);
                length = 0;
            }
            std::printf("> ");
            std::fflush(stdout);
        } else if ((byte == 0x08 || byte == 0x7F) && length) {
            --length;
        } else if (byte >= 0x20 && byte <= 0x7E &&
                   length + 1 < sizeof(line)) {
            line[length++] = static_cast<char>(byte);
        }
    }
}

}  // namespace

extern "C" void app_main() {
    usb_serial_jtag_driver_config_t config =
        USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    const esp_err_t console_result = usb_serial_jtag_driver_install(&config);
    if (console_result != ESP_OK && console_result != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(kTag, "USB console init failed: %s",
                 esp_err_to_name(console_result));
        return;
    }
    ESP_ERROR_CHECK(g_bus.initialize());
    g_motion_mutex = xSemaphoreCreateMutex();
    if (g_motion_mutex == nullptr) {
        ESP_LOGE(kTag, "motion mutex allocation failed"); return;
    }
    if (!g_estimator.begin())
        ESP_LOGW(kTag, "IMU unavailable; ORBIT check cannot pass");
    torqueOffBestEffort();
    std::printf("\nBoot: DISARMED, torque OFF, no automatic motion. "
                "Keep physical servo-power cutoff nearby.\n");
    if (tilt_orbit::kMappingConfigured) {
        std::printf("Leg mapping from config: ID 11-13 = robot %s leg. "
                    "roll sign = %+d. Run `check` to re-verify.\n",
                    tilt_orbit::kSwapLegSides ? "RIGHT" : "LEFT",
                    static_cast<int>(tilt_orbit::kRollSign));
    } else {
        std::printf("WARNING: physical left/right mapping is unverified. "
                    "Run `check` before ORBIT.\n");
    }
    g_queue = xQueueCreate(kQueueDepth, sizeof(Command));
    g_measure_request_queue = xQueueCreate(1, sizeof(MeasureRequest));
    g_measure_result_queue = xQueueCreate(1, sizeof(MeasureResult));
    if (g_queue == nullptr || g_measure_request_queue == nullptr ||
        g_measure_result_queue == nullptr) {
        ESP_LOGE(kTag, "queue allocation failed"); return;
    }
    xTaskCreate(consoleTask, "orbit_console", 4096, nullptr, 5, nullptr);
    if (xTaskCreate(measurementTask, "orbit_measure", 3072, nullptr, 1,
                    nullptr) != pdPASS) {
        ESP_LOGE(kTag, "measurement task creation failed"); return;
    }
    TickType_t wake = xTaskGetTickCount();
    while (true) {
        const std::uint32_t now = nowMs();
        monitorLoopDelay(now);
        g_estimator.sample(now);
        Command command{};
        for (int i = 0; i < 8 &&
             xQueueReceive(g_queue, &command, 0) == pdTRUE; ++i) {
            handleCommand(command);
        }
        serviceRamp(now);
        serviceCheck(now);
        serviceMeasurementResult();
        serviceGait(now);
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(tilt_orbit::kLoopPeriodMs));
    }
}
