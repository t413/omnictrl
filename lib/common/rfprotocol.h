#pragma once
#include <stdint.h>

enum class Cmds : uint8_t {
    None = 0,
    InfoStr = 0x05,
    MotionControl = 0x13,
    Telemetry = 0x14,
    Ping = 0x15,
    PingReply = 0x16,
    ModeChange = 0x20,
};

enum class Subt : uint8_t {
    None = 0,
    Robot = 0xbb,
    Controller = 0xcc,
};


struct MotionControl {
    uint32_t state = 0;
    float fwd = 0, side = 0, yaw = 0;
    float adjust = 0;
    float maxSpeed = 0;
    uint32_t timestamp = 0;
};
constexpr uint8_t MOTION_CONTROL_SIZE = sizeof(MotionControl);

struct Telem {
public:
    uint8_t state = 0;
    uint8_t mode = 0;
    uint16_t reserved2 = 0;
    float vbus = 0.0;
    float pitch = 0.0;
    uint32_t timestamp = 0;
};
