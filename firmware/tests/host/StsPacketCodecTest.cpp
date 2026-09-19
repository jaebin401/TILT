#include <tilt/sts3215/PacketCodec.h>

#include <cstdio>

namespace {

bool check(bool condition, const char* name) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", name);
    }
    return condition;
}

}  // namespace

int main() {
    const uint8_t read_parameters[] = {56, 2};
    const auto read = tilt::sts3215::makeInstructionPacket(
        11, tilt::sts3215::Instruction::Read,
        read_parameters, sizeof(read_parameters));
    if (!check(read.size() == 8 && read[0] == 0xFF && read[1] == 0xFF &&
                   read[2] == 11 && read[3] == 4 && read[4] == 2 &&
                   read[5] == 56 && read[6] == 2 && read[7] == 0xB4,
               "read packet bytes and checksum")) return 1;

    const uint8_t response[] = {0x12, 0xFF, 0xFF, 11, 4, 0, 0xFF, 0x07, 0xEA};
    const auto status = tilt::sts3215::findStatusPacket(
        response, sizeof(response), 11, 2);
    if (!check(status.has_value() && status->parameters.size() == 2 &&
                   status->parameters[0] == 0xFF && status->parameters[1] == 0x07,
               "valid status parsed after noise")) return 1;

    uint8_t corrupt[sizeof(response)]{};
    for (std::size_t index = 0; index < sizeof(response); ++index) {
        corrupt[index] = response[index];
    }
    corrupt[sizeof(corrupt) - 1] ^= 1;
    if (!check(!tilt::sts3215::findStatusPacket(
                    corrupt, sizeof(corrupt), 11, 2).has_value(),
               "bad checksum rejected")) return 1;

    std::puts("STS packet codec host tests passed");
    return 0;
}
