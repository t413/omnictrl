#pragma once
#include <stdint.h>

class MotorDrive {
public:
    virtual void requestStatus() = 0;
    virtual void setModePosition() = 0;
    virtual void setModeSpeed() = 0;
    virtual void setModeCurrent() = 0;
    virtual void enable(bool enable) = 0;
    virtual void setPos(float pos) = 0;
    virtual void setSpeed(float speed) = 0;
    virtual bool handleIncoming(uint32_t id, uint8_t* data, uint8_t len, uint32_t now) = 0;
    virtual uint32_t getLastStatusTime() const = 0;
    virtual uint32_t getLastFaults() const = 0;
};
