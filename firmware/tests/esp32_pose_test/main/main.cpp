// TILT 통합 pose 검증 앱
// Motion -> Safety -> ServoMapper -> STS3215 UART 전체 경로를 실제 하드웨어에서 검증한다.

#include <array>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <strings.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"

#include <tilt/actuators/ServoMapper.h>
#include <tilt/core/FaultReason.h>
#include <tilt/core/JointCommandSource.h>
#include <tilt/motion/PoseInterpolator.h>
#include <tilt/motion/PoseLibrary.h>
#include <tilt/safety/SafetyController.h>
#include <tilt/sts3215/Sts3215Bus.h>

#include "PoseTestConfig.h"

namespace {

constexpr std::size_t kJointCount = 6;
constexpr std::size_t kConsoleLineCapacity = 96;
constexpr char kImmediateStopKey = '!';

const char* kTag = "pose_test";

tilt::actuators::ServoCalibrationTable g_calibration =
    tilt_pose_test::kInitialCalibration;
tilt::sts3215::Sts3215Bus g_bus(tilt_pose_test::kBusConfig);
tilt::safety::SafetyController g_safety(
    tilt_pose_test::positionLimits(), tilt_pose_test::velocityLimits());
std::atomic<bool> g_stop_requested{false};
bool g_calibration_confirmed = false;
uint32_t g_sequence = 0;

std::array<uint8_t, kJointCount> servoIds() {
    std::array<uint8_t, kJointCount> result{};
    for (std::size_t index = 0; index < result.size(); ++index) {
        result[index] = g_calibration[index].id;
    }
    return result;
}

int jointIndex(const char* name) {
    if (name == nullptr) {
        return -1;
    }
    for (std::size_t index = 0; index < kJointCount; ++index) {
        if (strcasecmp(name, tilt_pose_test::kJointNames[index]) == 0) {
            return static_cast<int>(index);
        }
    }
    return -1;
}

const char* safetyStateName() {
    switch (g_safety.state()) {
        case tilt::safety::SafetyState::Disarmed: return "DISARMED";
        case tilt::safety::SafetyState::Armed: return "ARMED";
        case tilt::safety::SafetyState::Fault: return "FAULT";
    }
    return "UNKNOWN";
}

void torqueOffBestEffort() {
    const bool success = g_bus.setTorqueAll(servoIds(), false);
    if (!success) {
        ESP_LOGE(kTag, "토크 OFF 패킷 전송 실패: 물리 전원을 차단하세요.");
    }
}

void enterCommunicationFault(const char* reason) {
    torqueOffBestEffort();
    g_safety.reportFault(tilt::core::FaultReason::ServoCommunication);
    ESP_LOGE(kTag, "통신 Fault: %s", reason);
}

void emergencyStop() {
    g_stop_requested.store(true);
    g_safety.requestEmergencyStop();
    torqueOffBestEffort();
    std::printf("\n!! E-STOP: 소프트웨어 토크 OFF 요청. 필요하면 즉시 서보 전원을 차단하세요. !!\n");
}

bool readRawPositions(std::array<uint16_t, kJointCount>& raw) {
    if (!g_bus.readAllPositions(servoIds(), raw)) {
        enterCommunicationFault("6축 현재 위치 읽기 실패");
        return false;
    }
    return true;
}

bool readJointReference(tilt::core::JointPositionReference& reference) {
    std::array<uint16_t, kJointCount> raw{};
    if (!readRawPositions(raw)) {
        return false;
    }
    const auto mapped = tilt::actuators::mapToJointReference(raw, g_calibration);
    if (!mapped.has_value()) {
        torqueOffBestEffort();
        g_safety.disarm();
        ESP_LOGE(kTag, "현재 위치가 raw 소프트 한계를 벗어났습니다.");
        return false;
    }
    reference = *mapped;
    return true;
}

void printPositions() {
    std::array<uint16_t, kJointCount> raw{};
    if (!readRawPositions(raw)) {
        return;
    }
    const auto logical = tilt::actuators::mapToJointReference(raw, g_calibration);
    if (!logical.has_value()) {
        std::printf("현재 raw 위치가 설정 범위 밖입니다.\n");
        return;
    }
    for (std::size_t index = 0; index < kJointCount; ++index) {
        const float degrees = logical->angle_rad[index] * 57.29577951308232f;
        std::printf("  %s / ID %u : raw %u, logical %+7.2f deg, dir %+d\n",
                    tilt_pose_test::kJointNames[index], g_calibration[index].id,
                    raw[index], degrees, g_calibration[index].direction);
    }
}

bool pollImmediateStop(uint32_t duration_ms) {
    const uint32_t slices = duration_ms / 10u;
    for (uint32_t slice = 0; slice < slices; ++slice) {
        uint8_t byte = 0;
        const int received = usb_serial_jtag_read_bytes(
            &byte, 1, pdMS_TO_TICKS(10));
        if (received > 0 && byte == static_cast<uint8_t>(kImmediateStopKey)) {
            emergencyStop();
            return false;
        }
        if (g_stop_requested.load()) {
            return false;
        }
    }
    return true;
}

bool sendPoseSample(const tilt::motion::JointPose& pose,
                    const tilt::core::JointPositionReference& runtime_reference,
                    uint16_t duration_ms) {
    tilt::core::JointTargetBatch batch{};
    batch.seq = ++g_sequence;
    batch.timestamp_ms = static_cast<uint32_t>(
        xTaskGetTickCount() * portTICK_PERIOD_MS);
    batch.source = tilt::core::JointCommandSource::PoseTest;
    batch.duration_ms = duration_ms;
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        batch.angle_rad[joint] = pose.angle_rad[joint];
    }

    const auto safety_result = g_safety.evaluate(batch, runtime_reference);
    if (safety_result.status != tilt::safety::SafetyResult::Status::Accepted ||
        !safety_result.approved_batch.has_value()) {
        ESP_LOGE(kTag, "Safety가 seq=%lu 명령을 거부했습니다.",
                 static_cast<unsigned long>(batch.seq));
        torqueOffBestEffort();
        g_safety.disarm();
        return false;
    }

    const auto servo_targets = tilt::actuators::mapToServoTargets(
        *safety_result.approved_batch, g_calibration);
    if (!servo_targets.has_value()) {
        ESP_LOGE(kTag, "논리 각도 -> servo raw 변환을 거부했습니다.");
        torqueOffBestEffort();
        g_safety.disarm();
        return false;
    }
    if (!g_bus.syncWritePositions(*servo_targets,
                                  tilt_pose_test::kServoSpeedRaw)) {
        enterCommunicationFault("SYNC_WRITE 실패");
        return false;
    }
    return true;
}

bool playTransition(const tilt::motion::JointPose& target,
                    uint32_t duration_ms) {
    if (g_safety.state() != tilt::safety::SafetyState::Armed) {
        std::printf("먼저 confirm 후 arm 하세요.\n");
        return false;
    }

    tilt::core::JointPositionReference initial_reference{};
    if (!readJointReference(initial_reference)) {
        return false;
    }
    tilt::motion::JointPose start{};
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        start.angle_rad[joint] = initial_reference.angle_rad[joint];
    }

    g_stop_requested.store(false);
    for (uint32_t elapsed_ms = tilt_pose_test::kControlPeriodMs;
         elapsed_ms <= duration_ms;
         elapsed_ms += tilt_pose_test::kControlPeriodMs) {
        const auto sample = tilt::motion::samplePoseTransition(
            start, target,
            static_cast<float>(elapsed_ms) / 1000.0f,
            static_cast<float>(duration_ms) / 1000.0f);
        if (!sample.has_value()) {
            ESP_LOGE(kTag, "pose trajectory 생성 실패");
            torqueOffBestEffort();
            g_safety.disarm();
            return false;
        }

        tilt::core::JointPositionReference runtime_reference{};
        if (!readJointReference(runtime_reference) ||
            !sendPoseSample(sample->pose, runtime_reference,
                            tilt_pose_test::kControlPeriodMs)) {
            return false;
        }
        if (!pollImmediateStop(tilt_pose_test::kControlPeriodMs)) {
            return false;
        }
    }

    std::printf("pose 도착.\n");
    printPositions();
    return true;
}

bool initializeForArm() {
    if (!g_calibration_confirmed) {
        std::printf("방향을 확인한 뒤 confirm을 먼저 입력하세요.\n");
        return false;
    }

    tilt::core::JointPositionReference reference{};
    std::array<uint16_t, kJointCount> raw{};
    if (!readRawPositions(raw)) {
        return false;
    }
    const auto mapped = tilt::actuators::mapToJointReference(raw, g_calibration);
    if (!mapped.has_value()) {
        std::printf("현재 위치가 raw 소프트 범위 밖이라 arm할 수 없습니다.\n");
        return false;
    }
    reference = *mapped;
    if (!g_safety.initializeBootReference(reference) || !g_safety.arm()) {
        std::printf("Safety 초기화/arm 실패. limits, 현재 위치, Fault를 확인하세요.\n");
        return false;
    }

    // 토크를 켰을 때 과거 Goal로 튀지 않도록 현재 위치를 먼저 Goal에 기록한다.
    tilt::core::JointTargetBatch hold{};
    hold.seq = ++g_sequence;
    hold.duration_ms = tilt_pose_test::kControlPeriodMs;
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        hold.angle_rad[joint] = reference.angle_rad[joint];
    }
    const auto hold_targets = tilt::actuators::mapToServoTargets(hold, g_calibration);
    if (!hold_targets.has_value() ||
        !g_bus.syncWritePositions(*hold_targets, tilt_pose_test::kServoSpeedRaw)) {
        enterCommunicationFault("arm 전 hold target 설정 실패");
        return false;
    }
    for (const auto& servo : g_calibration) {
        if (!g_bus.setAcceleration(servo.id,
                                   tilt_pose_test::kServoAcceleration)) {
            enterCommunicationFault("가속도 설정 실패");
            return false;
        }
    }
    if (!g_bus.setTorqueAll(servoIds(), true)) {
        enterCommunicationFault("토크 ON 실패");
        return false;
    }
    std::printf("ARMED: 현재 위치 hold. '!'는 즉시 소프트웨어 E-STOP입니다.\n");
    return true;
}

void disarm() {
    torqueOffBestEffort();
    g_safety.disarm();
    g_stop_requested.store(false);
    std::printf("DISARMED: 전체 토크 OFF 요청 완료.\n");
}

void recoverFault() {
    torqueOffBestEffort();
    bool all_reachable = true;
    for (uint8_t id : servoIds()) {
        all_reachable = g_bus.ping(id) && all_reachable;
    }
    if (!all_reachable) {
        std::printf("6개 서보가 모두 응답하지 않아 Fault를 유지합니다.\n");
        return;
    }
    g_safety.reportFaultConditionCleared(
        tilt::core::FaultReason::ServoCommunication);
    g_safety.reportFaultConditionCleared(
        tilt::core::FaultReason::EmergencyStop);
    const auto result = g_safety.requestFaultRecovery();
    g_stop_requested.store(false);
    std::printf("recover 결과=%d, state=%s. 다시 arm해야 합니다.\n",
                static_cast<int>(result), safetyStateName());
}

void printPoseList() {
    std::printf("pose 목록:\n");
    for (std::size_t index = 0; index < tilt::motion::poseLibrarySize(); ++index) {
        std::printf("  %s%s\n", tilt::motion::poseLibrary()[index].name,
                    index == 0 ? " (ADR Home)" : " (검증용 초안)");
    }
}

void printHelp() {
    std::printf(
        "\nTILT 통합 pose test (명령 후 Enter)\n"
        "  status                         상태/현재 위치\n"
        "  directions                     6축 방향 표시\n"
        "  direction <joint> <+|->        논리->raw 방향 변경(Disarmed만)\n"
        "  confirm                        방향/영점 확인 완료 표시\n"
        "  arm                            Safety 승인 후 토크 ON + 현재 위치 hold\n"
        "  disarm                         토크 OFF\n"
        "  poses                          pose 목록\n"
        "  pose <name>                    3초 quintic trajectory 실행\n"
        "  home                           pose home 단축 명령\n"
        "  joint <joint> <delta-deg>      Joint Monkey (1회 최대 +/-5도)\n"
        "  recover                        통신 확인 후 Fault -> Disarmed\n"
        "  !                              즉시 소프트웨어 E-STOP\n"
        "joint: LHY LHP LKP RHY RHP RKP\n\n");
}

void printDirections() {
    for (std::size_t index = 0; index < kJointCount; ++index) {
        std::printf("  %s ID %u direction %+d, zero %u, raw [%u, %u]\n",
                    tilt_pose_test::kJointNames[index], g_calibration[index].id,
                    g_calibration[index].direction,
                    g_calibration[index].zero_raw,
                    g_calibration[index].min_raw,
                    g_calibration[index].max_raw);
    }
    std::printf("confirmed=%s\n", g_calibration_confirmed ? "yes" : "no");
}

void executeJointMonkey(const char* joint_name, float delta_degrees) {
    const int index = jointIndex(joint_name);
    if (index < 0 || !std::isfinite(delta_degrees) ||
        std::fabs(delta_degrees) >
            tilt_pose_test::kMaximumJointMonkeyDeltaDegrees ||
        delta_degrees == 0.0f) {
        std::printf("사용법: joint <LHY|LHP|LKP|RHY|RHP|RKP> <-5..+5>\n");
        return;
    }
    tilt::core::JointPositionReference current{};
    if (!readJointReference(current)) {
        return;
    }
    tilt::motion::JointPose target{};
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        target.angle_rad[joint] = current.angle_rad[joint];
    }
    target.angle_rad[index] += tilt_pose_test::degreesToRadians(delta_degrees);
    playTransition(target, tilt_pose_test::kJointDurationMs);
}

void executeCommand(char* line) {
    char command[20]{};
    char argument[24]{};
    char value[24]{};
    const int count = std::sscanf(line, "%19s %23s %23s", command, argument, value);
    if (count <= 0) {
        return;
    }

    if (strcasecmp(command, "help") == 0) {
        printHelp();
    } else if (strcasecmp(command, "status") == 0) {
        std::printf("state=%s, calibration=%s\n", safetyStateName(),
                    g_calibration_confirmed ? "confirmed" : "unconfirmed");
        printPositions();
    } else if (strcasecmp(command, "directions") == 0) {
        printDirections();
    } else if (strcasecmp(command, "direction") == 0 && count == 3) {
        if (g_safety.state() != tilt::safety::SafetyState::Disarmed) {
            std::printf("direction은 DISARMED에서만 바꿀 수 있습니다.\n");
            return;
        }
        const int index = jointIndex(argument);
        if (index < 0 || (std::strcmp(value, "+") != 0 &&
                          std::strcmp(value, "-") != 0)) {
            std::printf("사용법: direction <joint> <+|->\n");
            return;
        }
        g_calibration[index].direction = value[0] == '+' ? +1 : -1;
        g_calibration_confirmed = false;
        printDirections();
    } else if (strcasecmp(command, "confirm") == 0) {
        if (g_safety.state() != tilt::safety::SafetyState::Disarmed) {
            std::printf("confirm은 DISARMED에서만 가능합니다.\n");
            return;
        }
        std::array<uint16_t, kJointCount> raw{};
        if (readRawPositions(raw) &&
            tilt::actuators::mapToJointReference(raw, g_calibration).has_value()) {
            g_calibration_confirmed = true;
            std::printf("방향/영점 확인 상태를 이번 부팅 동안 저장했습니다.\n");
        }
    } else if (strcasecmp(command, "arm") == 0) {
        initializeForArm();
    } else if (strcasecmp(command, "disarm") == 0) {
        disarm();
    } else if (strcasecmp(command, "recover") == 0) {
        recoverFault();
    } else if (strcasecmp(command, "poses") == 0) {
        printPoseList();
    } else if (strcasecmp(command, "home") == 0) {
        playTransition(tilt::motion::homePose(),
                       tilt_pose_test::kPoseDurationMs);
    } else if (strcasecmp(command, "pose") == 0 && count >= 2) {
        const auto pose_id = tilt::motion::poseIdForName(argument);
        if (!pose_id.has_value()) {
            std::printf("알 수 없는 pose입니다. poses로 목록을 확인하세요.\n");
            return;
        }
        playTransition(tilt::motion::poseFor(*pose_id),
                       tilt_pose_test::kPoseDurationMs);
    } else if (strcasecmp(command, "joint") == 0 && count == 3) {
        char* end = nullptr;
        const float delta = std::strtof(value, &end);
        if (end == value || *end != '\0') {
            std::printf("delta-deg는 숫자여야 합니다.\n");
            return;
        }
        executeJointMonkey(argument, delta);
    } else {
        std::printf("알 수 없는 명령입니다. help를 입력하세요.\n");
    }
}

void consoleLoop() {
    char line[kConsoleLineCapacity]{};
    std::size_t length = 0;
    printHelp();
    std::printf("> ");
    std::fflush(stdout);

    while (true) {
        uint8_t byte = 0;
        if (usb_serial_jtag_read_bytes(&byte, 1, pdMS_TO_TICKS(50)) <= 0) {
            continue;
        }
        if (byte == static_cast<uint8_t>(kImmediateStopKey)) {
            emergencyStop();
            length = 0;
            std::printf("> ");
            continue;
        }
        if (byte == '\r' || byte == '\n') {
            if (length > 0) {
                line[length] = '\0';
                std::printf("\n");
                executeCommand(line);
                length = 0;
            }
            std::printf("> ");
            std::fflush(stdout);
            continue;
        }
        if ((byte == 0x08 || byte == 0x7F) && length > 0) {
            --length;
            continue;
        }
        if (std::isprint(byte) && length + 1 < sizeof(line)) {
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
        ESP_ERROR_CHECK(console_result);
    }

    ESP_ERROR_CHECK(g_bus.initialize());

    // 부팅은 항상 torque OFF다. 통신/위치 확인 전 자동 이동은 없다.
    torqueOffBestEffort();
    std::printf("\n부팅 완료: torque OFF / calibration 미확인 / state=DISARMED\n");

    std::array<uint16_t, kJointCount> initial_raw{};
    if (!readRawPositions(initial_raw)) {
        std::printf("초기 위치 읽기 실패. 전원/배선/ID를 확인한 뒤 recover 하세요.\n");
    } else {
        std::printf("6축 통신 확인 완료. 자동으로 arm하지 않습니다.\n");
        printPositions();
    }
    consoleLoop();
}
