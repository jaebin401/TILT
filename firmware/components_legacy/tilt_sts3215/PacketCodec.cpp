#include <tilt/sts3215/PacketCodec.h>

#include <limits>

namespace tilt {
namespace sts3215 {

namespace {

uint8_t checksum(const uint8_t* bytes, std::size_t first, std::size_t last) {
    uint32_t sum = 0;
    for (std::size_t index = first; index < last; ++index) {
        sum += bytes[index];
    }
    return static_cast<uint8_t>(~sum);
}

}  // namespace

std::vector<uint8_t> makeInstructionPacket(
    uint8_t id,
    Instruction instruction,
    const uint8_t* parameters,
    std::size_t parameter_count) {
    if (id > kBroadcastId || parameter_count > 253 ||
        (parameter_count > 0 && parameters == nullptr)) {
        return {};
    }

    const uint8_t length = static_cast<uint8_t>(parameter_count + 2);
    std::vector<uint8_t> packet;
    packet.reserve(parameter_count + 6);
    packet.push_back(kHeader);
    packet.push_back(kHeader);
    packet.push_back(id);
    packet.push_back(length);
    packet.push_back(static_cast<uint8_t>(instruction));
    for (std::size_t index = 0; index < parameter_count; ++index) {
        packet.push_back(parameters[index]);
    }
    packet.push_back(checksum(packet.data(), 2, packet.size()));
    return packet;
}

std::optional<StatusPacket> findStatusPacket(
    const uint8_t* bytes,
    std::size_t byte_count,
    uint8_t expected_id,
    std::size_t expected_parameter_count) {
    if (bytes == nullptr || expected_parameter_count > 253) {
        return std::nullopt;
    }

    const std::size_t expected_total = expected_parameter_count + 6;
    for (std::size_t offset = 0; offset + expected_total <= byte_count; ++offset) {
        if (bytes[offset] != kHeader || bytes[offset + 1] != kHeader ||
            bytes[offset + 2] != expected_id ||
            bytes[offset + 3] != expected_parameter_count + 2) {
            continue;
        }

        const std::size_t checksum_index = offset + expected_total - 1;
        if (bytes[checksum_index] != checksum(bytes, offset + 2, checksum_index)) {
            continue;
        }

        // 정상 status의 다섯 번째 바이트는 error다. TX echo의 instruction과 구별된다.
        if (bytes[offset + 4] != 0) {
            continue;
        }

        StatusPacket result{};
        result.id = expected_id;
        result.error = bytes[offset + 4];
        result.parameters.assign(bytes + offset + 5, bytes + checksum_index);
        return result;
    }
    return std::nullopt;
}

}  // namespace sts3215
}  // namespace tilt
