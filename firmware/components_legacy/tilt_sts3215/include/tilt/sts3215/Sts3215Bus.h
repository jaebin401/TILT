#pragma once

#include <array>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

#include <tilt/actuators/ServoMapper.h>

namespace tilt {
namespace sts3215 {

struct BusConfig {
    uart_port_t uart_port;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    int baud_rate;
    uint32_t response_timeout_ms;
};

class Sts3215Bus {
public:
    explicit Sts3215Bus(const BusConfig& config);

    esp_err_t initialize();
    bool ping(uint8_t id);
    bool readPosition(uint8_t id, uint16_t& position_raw);
    bool readAllPositions(
        const std::array<uint8_t, actuators::kServoCount>& ids,
        std::array<uint16_t, actuators::kServoCount>& position_raw);
    bool setTorque(uint8_t id, bool enabled);
    bool setTorqueAll(
        const std::array<uint8_t, actuators::kServoCount>& ids,
        bool enabled);
    bool setAcceleration(uint8_t id, uint8_t acceleration);
    bool syncWritePositions(const actuators::ServoTargetBatch& targets,
                            uint16_t speed_raw);

private:
    bool writeInstruction(uint8_t id,
                          uint8_t instruction,
                          const uint8_t* parameters,
                          std::size_t parameter_count);
    bool transactForStatus(uint8_t id,
                           uint8_t instruction,
                           const uint8_t* parameters,
                           std::size_t parameter_count,
                           std::size_t expected_response_parameters,
                           uint8_t* response_parameters);

    BusConfig config_;
    bool initialized_ = false;
};

}  // namespace sts3215
}  // namespace tilt
