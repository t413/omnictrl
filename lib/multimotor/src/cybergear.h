#pragma once
#include "motordrive.h"

class CanInterface;
enum class CyberGearMode {
    Unknown = 0,
    Position = 1,
    Speed = 2,
    Current = 3,
};

class CyberGearDriver : public MotorDrive {
    uint8_t id_ = 0;
    CanInterface* can_ = nullptr;
    uint8_t lastFaults_ = 0;
    uint32_t lastStatusTime_ = 0;
    bool enabled_ = false;
public:
    CyberGearDriver(uint8_t id, CanInterface* can);

    //contract
    void requestStatus() override;
    virtual void setModePosition() { setMode(CyberGearMode::Position); }
    virtual void setModeSpeed() { setMode(CyberGearMode::Speed); }
    virtual void setModeCurrent() { setMode(CyberGearMode::Current); }
    void enable(bool enable) override;
    void setSpeed(float speed) override;
    void setPos(float pos) override;
    void setCurrent(float current) override;
    bool handleIncoming(uint32_t id, uint8_t* data, uint8_t len, uint32_t now) override;
    uint32_t getLastStatusTime() const override { return lastStatusTime_; }
    uint32_t getLastFaults() const override { return lastFaults_; }

    void setMode(CyberGearMode mode);
};

