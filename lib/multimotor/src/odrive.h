#pragma once
#include "motordrive.h"

class CanInterface;
enum class OdriveAxisState : uint8_t {
    Undefined = 0,
    Idle = 1,
    StartupSequence = 2,
    FullCalibrationSequence = 3,
    MotorCalibration = 4,
    EncoderIndexSearch = 5,
    EncoderOffsetCalibration = 6,
    ClosedLoopControl = 7,
    LockinSpin = 8,
    EncoderDirFind = 9,
    Homing = 10,
    EncoderHallPolarityCalibration = 11,
    EncoderHallPhaseCalibration = 12,
};
enum class OdriveCtrlMode : int8_t {
    Unknown   = -1,
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
    uint32_t lastStatusTime_ = 0;
    float lastVolt_ = 0, lastCurr_ = 0; //encoder estimates packet
    uint32_t lastBusVoltTime_ = 0;
    bool enabled_ = false;
    MotorState lastStatus_; // Holds the state of the motor
    MotorMode lastSentMode_ = MotorMode::Unknown;
public:
    ODriveDriver(uint8_t id, CanInterface* can);

    //contract
    void requestStatus() override;
    void setMode(MotorMode) override;
    void setSetpoint(MotorMode, float) override;
    bool handleIncoming(uint32_t id, uint8_t* data, uint8_t len, uint32_t now) override;
    uint32_t getLastStatusTime() const override { return lastStatusTime_; }
    uint32_t getLastFaults() const override { return lastFaults_; }

    MotorState getMotorState() const override { return lastStatus_; }
    void setOdriveMode(OdriveCtrlMode);
    void setOdriveEnable(bool enable);
    void send(CmdIDs cmd, uint8_t* data, uint8_t len = 8, bool ss = true, bool rtr = false);

    void fetchVBus() override;
    float getVBus() const { return lastVolt_; }
};

