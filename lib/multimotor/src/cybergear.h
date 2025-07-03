#pragma once
#include "motordrive.h"

class CanInterface;

class CyberGearDriver : public MotorDrive {
    uint8_t id_ = 0;
    CanInterface* can_ = nullptr;
    uint8_t lastFaults_ = 0;
    uint32_t lastStatusTime_ = 0;
    bool enabled_ = false;

    MotorState lastStatus_; // Holds the state of the motor
public:
    CyberGearDriver(uint8_t id, CanInterface* can);

    //contract
    void requestStatus() override;
    void setMode(MotorMode) override;
    void setSetpoint(MotorMode, float) override;
    bool handleIncoming(uint32_t id, uint8_t* data, uint8_t len, uint32_t now) override;
    uint32_t getLastStatusTime() const override { return lastStatusTime_; }
    uint32_t getLastFaults() const override { return lastFaults_; }

    MotorState getMotorState() const override { return lastStatus_; }
    void setCyberMode(uint8_t mode);
    void setEnable(bool enable);
};

