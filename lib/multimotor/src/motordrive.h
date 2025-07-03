#pragma once
#include <stdint.h>

enum class MotorMode {
    Disabled = 0,
    Current = 1,
    Speed = 2,
    Position = 3,
};

struct MotorState {
    float temperature = 0.0f; // in degrees Celsius
    float position = 0.0f; // in radians
    float velocity = 0.0f; // in radians per second
    float torque = 0.0f; // in Newton-meters
    MotorMode mode = MotorMode::Disabled;
};
class MotorDrive {
public:
    virtual void requestStatus() = 0;
    virtual void setMode(MotorMode mode) = 0;
    virtual void setSetpoint(MotorMode, float) = 0;
    virtual bool handleIncoming(uint32_t id, uint8_t* data, uint8_t len, uint32_t now) = 0;
    virtual uint32_t getLastStatusTime() const = 0;
    virtual uint32_t getLastFaults() const = 0;

    virtual MotorState getMotorState() const = 0;
};
