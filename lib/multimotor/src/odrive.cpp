#include "odrive.h"
#include "can_interface.h"
#include <string.h>
#include <Arduino.h> //just for Serial

enum class CmdIDs : uint8_t {
    GetVersion            = 0x00,
    Heartbeat             = 0x01,
    Estop                 = 0x02,
    GetError              = 0x03,
    RxSdo                 = 0x04,
    TxSdo                 = 0x05,
    Address               = 0x06,
    SetAxisState          = 0x07,
    GetEncoderEstimates   = 0x09,
    SetControllerMode     = 0x0b,
    SetInputPos           = 0x0c,
    SetInputVel           = 0x0d,
    SetInputTorque        = 0x0e,
    SetLimits             = 0x0f,
    SetTrajVelLimit       = 0x11,
    SetTrajAccelLimits    = 0x12,
    SetTrajInertia        = 0x13,
    GetIq                 = 0x14,
    GetTemperature        = 0x15,
    Reboot                = 0x16,
    GetBusVoltageCurrent  = 0x17,
    ClearErrors           = 0x18,
    SetAbsolutePosition   = 0x19,
    SetPosGain            = 0x1a,
    SetVelGains           = 0x1b,
    GetTorques            = 0x1c,
    GetPowers             = 0x1d,
};

ODriveDriver::ODriveDriver(uint8_t id, CanInterface* can) : id_(id), can_(can) { }

uint16_t mkID(uint8_t id, CmdIDs cmd) {
    return (id << 4) | (uint16_t) cmd;
}

void ODriveDriver::send(CmdIDs cmd, uint8_t* data, uint8_t len, bool ss, bool rtr) {
    if (can_) can_->send(mkID(id_, (CmdIDs) cmd), data, len);
}

void ODriveDriver::requestStatus() {
    //no need, those are sent automatically
}

//get different combinations of payload
union Payload {
    uint8_t  bytes[8] = {0};
    uint16_t words[4];
    uint32_t dwords[2];
    float    floats[2];
};

void ODriveDriver::setOdriveMode(OdriveCtrlMode mode) {
    Payload p;
    p.dwords[0] = (uint32_t) mode;
    p.dwords[1] = 1; //passtrough input mode
    send(CmdIDs::SetControllerMode, p.bytes);
}

void ODriveDriver::setOdriveEnable(bool enable) {
    Payload p;
    p.dwords[0] = enable ? 8 : 0; //Closed loop control
    send(CmdIDs::SetAxisState, p.bytes, 8, false); //no single shot (enables retries)
}

void ODriveDriver::setMode(MotorMode mode) {
    if (mode == MotorMode::Disabled) {
        setOdriveEnable(false); //disable before setting mode
    } else {
        OdriveCtrlMode odriveMode = (mode == MotorMode::Speed) ? OdriveCtrlMode::Velocity :
                       (mode == MotorMode::Current) ? OdriveCtrlMode::Torque :
                       OdriveCtrlMode::Position;
        setOdriveMode(odriveMode);
        setOdriveEnable(true);
    }
}

void ODriveDriver::setSetpoint(MotorMode mode, float value) {
    Payload p;
    p.floats[0] = value;
    CmdIDs cmd = (mode == MotorMode::Position) ? CmdIDs::SetInputPos :
                 (mode == MotorMode::Speed)    ? CmdIDs::SetInputVel : CmdIDs::SetInputTorque;
    send(cmd, p.bytes);
}

bool ODriveDriver::handleIncoming(uint32_t id, uint8_t* data, uint8_t len, uint32_t now) {
    uint8_t inCanId = id >> 5;
    if (inCanId != id_) return false;
    CmdIDs cmd = (CmdIDs) (id & 0x1F);
    Payload p;
    memcpy(p.bytes, data, len);
    if (cmd == CmdIDs::GetEncoderEstimates) { //default ever 10ms
        lastPos_ = p.floats[0];
        lastVel_ = p.floats[1];
        lastStatusTime_ = now;
    } else if (cmd == CmdIDs::GetBusVoltageCurrent) {
        lastVolt_ = p.floats[0];
        lastCurr_ = p.floats[1];
        lastBusVoltTime_ = now;
    } else if (cmd == CmdIDs::Heartbeat) {
        lastFaults_ = p.dwords[0];
        lastAxisState_ = p.bytes[4];
        lastHeartbeatTime_ = now;
    } else if (Serial && Serial.availableForWrite()) {
        Serial.printf(" > o-rx cmd %x len %d: {", (uint8_t) cmd, len);
        for (int i = 0; i < len; i++)
            Serial.printf("0x%02x, ", data[i]);
        Serial.println("}");
    }
    return true;
}


