#pragma once
#include "motordrive.h"

class CanInterface;
enum class OdriveCtrlMode {
    Voltage   = 0,
    Torque    = 1,
    Velocity  = 2,
    Position  = 3,
};
enum class CmdIDs : uint8_t;

class ODriveDriver : public MotorDrive {
    uint8_t id_ = 0;
    CanInterface* can_ = nullptr;
    uint32_t lastFaults_ = 0;
    uint8_t lastAxisState_ = 0;
    uint32_t lastHeartbeatTime_ = 0;
    float lastPos_ = 0, lastVel_ = 0; //encoder estimates packet
    uint32_t lastStatusTime_ = 0;
    float lastVolt_ = 0, lastCurr_ = 0; //encoder estimates packet
    uint32_t lastBusVoltTime_ = 0;
    bool enabled_ = false;
public:
    ODriveDriver(uint8_t id, CanInterface* can);

    //contract
    void requestStatus() override;
    void setMode(MotorMode) override;
    void setSetpoint(MotorMode, float) override;
    bool handleIncoming(uint32_t id, uint8_t* data, uint8_t len, uint32_t now) override;
    uint32_t getLastStatusTime() const override { return lastStatusTime_; }
    uint32_t getLastFaults() const override { return lastFaults_; }

    void setOdriveMode(OdriveCtrlMode);
    void setOdriveEnable(bool enable);
    void send(CmdIDs cmd, uint8_t* data, uint8_t len = 8, bool ss = true, bool rtr = false);
};

