#pragma once
#include <stdint.h>

struct MotorState {
    float temperature = 0.0f; // in degrees Celsius
    float position = 0.0f; // in radians
    float velocity = 0.0f; // in radians per second
    float torque = 0.0f; // in Newton-meters
    uint8_t mode_status = 0; // status of the motor mode (e.g., position, speed, current)
};
class MotorDrive {
public:
    virtual void requestStatus() = 0;
    virtual void setModePosition() = 0;
    virtual void setModeSpeed() = 0;
    virtual void setModeCurrent() = 0;
    virtual void enable(bool enable) = 0;
    virtual void setPos(float) = 0;
    virtual void setSpeed(float) = 0;
    virtual void setCurrent(float) = 0;
    virtual bool handleIncoming(uint32_t id, uint8_t* data, uint8_t len, uint32_t now) = 0;
    virtual uint32_t getLastStatusTime() const = 0;
    virtual uint32_t getLastFaults() const = 0;

    virtual MotorState getMotorState() const = 0;
};
