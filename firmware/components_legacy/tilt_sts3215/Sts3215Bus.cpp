#include <tilt/sts3215/Sts3215Bus.h>

#include <algorithm>
#include <vector>

#include "freertos/FreeRTOS.h"

#include <tilt/sts3215/PacketCodec.h>

namespace tilt {
namespace sts3215 {

namespace {

constexpr uint8_t kAddressTorqueEnable = 40;
constexpr uint8_t kAddressAcceleration = 41;
constexpr uint8_t kAddressGoalPosition = 42;
constexpr uint8_t kAddressPresentPosition = 56;

constexpr std::size_t kRxBufferSize = 128;

uint8_t lowByte(uint16_t value) {
    return static_cast<uint8_t>(value & 0xFFu);
}

uint8_t highByte(uint16_t value) {
    return static_cast<uint8_t>((value >> 8u) & 0xFFu);
}

}  // namespace

Sts3215Bus::Sts3215Bus(const BusConfig& config) : config_(config) {}

esp_err_t Sts3215Bus::initialize() {
    if (initialized_) {
        return ESP_OK;
    }

    uart_config_t uart_config{};
    uart_config.baud_rate = config_.baud_rate;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    esp_err_t result = uart_driver_install(config_.uart_port, 512, 512, 0, nullptr, 0);
    if (result != ESP_OK) {
        return result;
    }
    result = uart_param_config(config_.uart_port, &uart_config);
    if (result != ESP_OK) {
        return result;
    }
    result = uart_set_pin(config_.uart_port, config_.tx_pin, config_.rx_pin,
                          UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (result == ESP_OK) {
        initialized_ = true;
    }
    return result;
}

bool Sts3215Bus::writeInstruction(uint8_t id,
                                  uint8_t instruction,
                                  const uint8_t* parameters,
                                  std::size_t parameter_count) {
    if (!initialized_) {
        return false;
    }
    const auto packet = makeInstructionPacket(
        id, static_cast<Instruction>(instruction), parameters, parameter_count);
    if (packet.empty()) {
        return false;
    }
    const int written = uart_write_bytes(
        config_.uart_port, packet.data(), packet.size());
    return written == static_cast<int>(packet.size()) &&
           uart_wait_tx_done(config_.uart_port,
                             pdMS_TO_TICKS(config_.response_timeout_ms)) == ESP_OK;
}

bool Sts3215Bus::transactForStatus(
    uint8_t id,
    uint8_t instruction,
    const uint8_t* parameters,
    std::size_t parameter_count,
    std::size_t expected_response_parameters,
    uint8_t* response_parameters) {
    if (expected_response_parameters > 0 && response_parameters == nullptr) {
        return false;
    }

    uart_flush_input(config_.uart_port);
    if (!writeInstruction(id, instruction, parameters, parameter_count)) {
        return false;
    }

    uint8_t buffer[kRxBufferSize]{};
    const int received = uart_read_bytes(
        config_.uart_port, buffer, sizeof(buffer),
        pdMS_TO_TICKS(config_.response_timeout_ms));
    if (received <= 0) {
        return false;
    }
    const auto status = findStatusPacket(
        buffer, static_cast<std::size_t>(received), id,
        expected_response_parameters);
    if (!status.has_value()) {
        return false;
    }
    std::copy(status->parameters.begin(), status->parameters.end(),
              response_parameters);
    return true;
}

bool Sts3215Bus::ping(uint8_t id) {
    uint8_t unused = 0;
    return transactForStatus(id, static_cast<uint8_t>(Instruction::Ping),
                             nullptr, 0, 0, &unused);
}

bool Sts3215Bus::readPosition(uint8_t id, uint16_t& position_raw) {
    const uint8_t parameters[] = {kAddressPresentPosition, 2};
    uint8_t response[2]{};
    if (!transactForStatus(id, static_cast<uint8_t>(Instruction::Read),
                           parameters, sizeof(parameters), sizeof(response), response)) {
        return false;
    }
    position_raw = static_cast<uint16_t>(response[0]) |
                   (static_cast<uint16_t>(response[1]) << 8u);
    return position_raw <= actuators::kStsPositionMax;
}

bool Sts3215Bus::readAllPositions(
    const std::array<uint8_t, actuators::kServoCount>& ids,
    std::array<uint16_t, actuators::kServoCount>& position_raw) {
    for (std::size_t index = 0; index < ids.size(); ++index) {
        if (!readPosition(ids[index], position_raw[index])) {
            return false;
        }
    }
    return true;
}

bool Sts3215Bus::setTorque(uint8_t id, bool enabled) {
    const uint8_t parameters[] = {kAddressTorqueEnable,
                                  static_cast<uint8_t>(enabled ? 1 : 0)};
    return writeInstruction(id, static_cast<uint8_t>(Instruction::Write),
                            parameters, sizeof(parameters));
}

bool Sts3215Bus::setTorqueAll(
    const std::array<uint8_t, actuators::kServoCount>& ids,
    bool enabled) {
    bool success = true;
    for (uint8_t id : ids) {
        success = setTorque(id, enabled) && success;
    }
    return success;
}

bool Sts3215Bus::setAcceleration(uint8_t id, uint8_t acceleration) {
    const uint8_t parameters[] = {kAddressAcceleration, acceleration};
    return writeInstruction(id, static_cast<uint8_t>(Instruction::Write),
                            parameters, sizeof(parameters));
}

bool Sts3215Bus::syncWritePositions(
    const actuators::ServoTargetBatch& targets,
    uint16_t speed_raw) {
    constexpr uint8_t kDataLength = 6;  // position, time, speed: 각각 2바이트.
    std::vector<uint8_t> parameters;
    parameters.reserve(2 + targets.id.size() * (1 + kDataLength));
    parameters.push_back(kAddressGoalPosition);
    parameters.push_back(kDataLength);
    for (std::size_t index = 0; index < targets.id.size(); ++index) {
        const uint16_t position = targets.position_raw[index];
        parameters.push_back(targets.id[index]);
        parameters.push_back(lowByte(position));
        parameters.push_back(highByte(position));
        parameters.push_back(0);  // goal time low: 속도 제한 방식을 사용한다.
        parameters.push_back(0);  // goal time high
        parameters.push_back(lowByte(speed_raw));
        parameters.push_back(highByte(speed_raw));
    }
    return writeInstruction(kBroadcastId,
                            static_cast<uint8_t>(Instruction::SyncWrite),
                            parameters.data(), parameters.size());
}

}  // namespace sts3215
}  // namespace tilt
