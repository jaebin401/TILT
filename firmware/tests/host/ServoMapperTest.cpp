#include <tilt/actuators/ServoMapper.h>

#include <cmath>
#include <cstdio>

namespace {

tilt::actuators::ServoCalibrationTable calibration() {
    return {{{11, +1, 2047, 0, 4095},
             {12, -1, 2047, 0, 4095},
             {13, +1, 2047, 0, 4095},
             {21, -1, 2047, 0, 4095},
             {22, +1, 2047, 0, 4095},
             {23, -1, 2047, 0, 4095}}};
}

bool check(bool condition, const char* name) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", name);
    }
    return condition;
}

}  // namespace

int main() {
    auto table = calibration();
    if (!check(tilt::actuators::isValidCalibration(table),
               "valid calibration")) return 1;

    tilt::core::JointTargetBatch batch{};
    batch.seq = 7;
    batch.duration_ms = 100;
    batch.angle_rad[0] = 3.14159265358979323846f / 2.0f;
    batch.angle_rad[1] = 3.14159265358979323846f / 2.0f;
    const auto mapped = tilt::actuators::mapToServoTargets(batch, table);
    if (!check(mapped.has_value() && mapped->position_raw[0] == 3071 &&
                   mapped->position_raw[1] == 1023 && mapped->id[5] == 23,
               "logical radians map with direction")) return 1;

    const auto round_trip = tilt::actuators::mapToJointReference(
        mapped->position_raw, table);
    if (!check(round_trip.has_value() &&
                   std::fabs(round_trip->angle_rad[0] - batch.angle_rad[0]) < 0.002f &&
                   std::fabs(round_trip->angle_rad[1] - batch.angle_rad[1]) < 0.002f,
               "raw feedback round trip")) return 1;

    batch.angle_rad[0] = 10.0f;
    if (!check(!tilt::actuators::mapToServoTargets(batch, table).has_value(),
               "raw soft limit rejects entire batch")) return 1;

    table[1].id = table[0].id;
    if (!check(!tilt::actuators::isValidCalibration(table),
               "duplicate IDs rejected")) return 1;

    std::puts("Servo mapper host tests passed");
    return 0;
}
