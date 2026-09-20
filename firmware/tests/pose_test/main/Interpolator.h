#pragma once

#include <cstdint>

#include "tilt_config.h"

namespace tilt_pose_test {

class Interpolator {
public:
    void start(const float from[tilt::NUM_JOINTS],
               const float to[tilt::NUM_JOINTS],
               std::uint32_t duration_ms,
               std::uint32_t now_ms);

    // Writes the current sample. Returns true only while interpolation remains
    // active after this sample; the final target is still written on false.
    bool update(std::uint32_t now_ms, float out[tilt::NUM_JOINTS]);
    bool isBusy() const;
    void abort();

private:
    float from_[tilt::NUM_JOINTS]{};
    float to_[tilt::NUM_JOINTS]{};
    std::uint32_t started_ms_ = 0;
    std::uint32_t duration_ms_ = 0;
    bool busy_ = false;
};

}  // namespace tilt_pose_test
