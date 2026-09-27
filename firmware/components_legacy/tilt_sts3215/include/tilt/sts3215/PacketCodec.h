#pragma once

#include <optional>
#include <stddef.h>
#include <stdint.h>
#include <vector>

namespace tilt {
namespace sts3215 {

constexpr uint8_t kHeader = 0xFF;
constexpr uint8_t kBroadcastId = 0xFE;

enum class Instruction : uint8_t {
    Ping = 0x01,
    Read = 0x02,
    Write = 0x03,
    SyncWrite = 0x83,
};

struct StatusPacket {
    uint8_t id;
    uint8_t error;
    std::vector<uint8_t> parameters;
};

std::vector<uint8_t> makeInstructionPacket(
    uint8_t id,
    Instruction instruction,
    const uint8_t* parameters,
    std::size_t parameter_count);

// 수신 버퍼 안의 첫 정상 응답을 찾는다. 체크섬과 길이를 모두 확인한다.
std::optional<StatusPacket> findStatusPacket(
    const uint8_t* bytes,
    std::size_t byte_count,
    uint8_t expected_id,
    std::size_t expected_parameter_count);

}  // namespace sts3215
}  // namespace tilt
