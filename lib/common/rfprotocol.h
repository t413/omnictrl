#pragma once
#include <stdint.h>

enum class Cmds : int8_t {
    None = 0,
    MotionControl = 0x13,
};

struct MotionControl {
    uint32_t state = 0;
    float fwd = 0;
    float yaw = 0;
    float pitch = 0;
    float roll = 0;
    uint32_t timestamp = 0;
};
constexpr uint8_t MOTION_CONTROL_SIZE = sizeof(MotionControl);
