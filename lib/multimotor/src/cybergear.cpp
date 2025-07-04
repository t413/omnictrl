#include "cybergear.h"
#include "can_interface.h"
#include <string.h>
#include <Arduino.h> // just for Serial!

#define P_MIN (-12.5f)
#define P_MAX (12.5f)
#define V_MIN (-30.0f)
#define V_MAX (30.0f)
#define T_MIN (-12.0f)
#define T_MAX (12.0f)

float uint_to_float(uint16_t value, float value_min, float value_max) {
    uint16_t int_max = 0xFFFF;
    float span = value_max - value_min;
    return (float)value / int_max * span + value_min;
}

enum Cmds {
    CmdPosition              =  0x1,
    CmdRequest               =  0x2,
    CmdEnable                =  0x3,
    CmdStop                  =  0x4,
    CmdSetMechPositionToZero =  0x6,
    CmdSetCanId              =  0x7,
    CmdWriteParamLower       =  0x8,  // params 0x0000 - 0x302F
    CmdReadParamLower        =  0x9,  // params 0x0000 - 0x302F
    CmdReadParamUpper        = 0x11,  // params >= 0x7005
    CmdWriteParamUpper       = 0x12,  // params >= 0x7005
    CmdFault                 = 0x15, //also apparently used for status requests
};

enum Addresses {
    AddrRunMode = 0x7005,
    AddrSpeedSetpoint = 0x700A,
    AddrCurrentSetpoint = 0x7006,
    AddrPosSetpoint   = 0x7016,
    AddrVBUSmv   = 0x3007, //uint16_t, millivolts
    AddrVBUSfloat= 0x302B,
    AddrVBUSHigh = 0x701C,
    PARAM_UPPER_ADDR = 0x7000,  // Threshold for upper/lower commands
};

enum class CyberGearMode {
    Unknown = 0,
    Position = 1,
    Speed = 2,
    Current = 3,
};

CyberGearMode toCyberGearMode(MotorMode mode) {
    switch (mode) {
        case MotorMode::Position: return CyberGearMode::Position;
        case MotorMode::Speed: return CyberGearMode::Speed;
        case MotorMode::Current: return CyberGearMode::Current;
        default: return CyberGearMode::Unknown;
    }
}

MotorMode toMotorMode(CyberGearMode mode) {
    switch (mode) {
        case CyberGearMode::Position: return MotorMode::Position;
        case CyberGearMode::Speed: return MotorMode::Speed;
        case CyberGearMode::Current: return MotorMode::Current;
        default: return MotorMode::Disabled;
    }
}


CyberGearDriver::CyberGearDriver(uint8_t id, CanInterface* can) : id_(id), can_(can) { }

uint32_t mkID(uint8_t cmd, uint8_t opthi, uint8_t optlo, uint8_t id) {
    return (cmd << 24) | (opthi << 16) | (optlo << 8) | id;
}

void CyberGearDriver::requestStatus() {
    uint8_t data[8] = {0x00}; //TODO can this 0 bytes?
    if (can_) can_->send(mkID(CmdFault, 0, 0, id_), data, 8);
}

void CyberGearDriver::setCyberMode(uint8_t mode) {
    uint8_t data[8] = { AddrRunMode & 0x00FF, AddrRunMode >> 8, 0x00, 0x00, (uint8_t) mode, 0x00, 0x00, 0x00};
    if (can_) can_->send(mkID(CmdWriteParamUpper, 0, 0, id_), data, 8);
}

void CyberGearDriver::setEnable(bool enable) {
    uint8_t data[8] = {0x00};
    if (can_) can_->send(mkID(enable? CmdEnable : CmdStop, 0, 0, id_), data, 8);
}

void CyberGearDriver::setMode(MotorMode mode) {
    CyberGearMode out = toCyberGearMode(mode);
    if (out == CyberGearMode::Unknown) {
        setEnable(false); //disable if unknown mode
    } else {
        setCyberMode((uint8_t)out);
        setEnable(true);
        lastStatus_.mode = mode;
    }
}

void CyberGearDriver::setSetpoint(MotorMode mode, float value) {
    uint16_t addr;
    switch (mode) {
        case MotorMode::Position: addr = AddrPosSetpoint; break;
        case MotorMode::Speed:    addr = AddrSpeedSetpoint; break;
        case MotorMode::Current:  addr = AddrCurrentSetpoint; break;
        default: return;
    }
    uint8_t data[8] = { addr & 0x00FF, addr >> 8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    memcpy(&data[4], &value, 4);
    if (can_) can_->send(mkID(CmdWriteParamUpper, 0, 0, id_), data, 8);
}

void CyberGearDriver::fetchVBus() {
    uint8_t data[8] = { AddrVBUSfloat & 0x00FF, AddrVBUSfloat >> 8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (can_) can_->send(mkID(CmdReadParamLower, 0, 0, id_), data, 8);
}

bool CyberGearDriver::handleIncoming(uint32_t id, uint8_t* data, uint8_t len, uint32_t now) {
    uint8_t msgtype = (id & 0xFF000000) >> 24; //bits 24-28
    uint8_t driveid = (id & 0x0000FF00) >> 8; //bits 8-15
    if (driveid != id_) return false;

    if (msgtype == CmdRequest) { //status message reply!
        uint16_t pos_data = data[1] | (data[0] << 8);
        uint16_t vel_data = data[3] | (data[2] << 8);
        uint16_t torque_data = data[5] | (data[4] << 8);
        uint16_t raw_temp = data[7] | (data[6] << 8);

        MotorState updated = {};
        updated.position = uint_to_float(pos_data, P_MIN, P_MAX);
        updated.velocity = uint_to_float(vel_data, V_MIN, V_MAX);
        updated.torque = uint_to_float(torque_data, T_MIN, T_MAX);
        updated.temperature = raw_temp ? (float)raw_temp / 10.0f : 0.0f;

        uint8_t inmode = (id & 0x00C00000) >> 22; // bits 22-23
        updated.mode = toMotorMode((CyberGearMode)inmode);
        lastFaults_   = (id & 0x003F0000) >> 16; //bits 16-21 = [Uncalibrated, hall, magsense, overtemp, overcurrent, undervolt]
        lastStatus_ = updated; // Update the last status with the new values
        enabled_ = (updated.mode != MotorMode::Disabled);
        lastStatusTime_ = now;
        if (Serial && Serial.availableForWrite())
        return true;
    } else if (msgtype == CmdFault) { //fault message (decimal 21)
        if (Serial && Serial.availableForWrite()) {
            Serial.printf("drive %x: fault message %x: {", id_, id);
            for (int i = 0; i < len; i++)
                Serial.printf("0x%02x, ", data[i]);
            Serial.println("}");
        }
        return true;
    } else if (msgtype == CmdReadParamLower || msgtype == CmdReadParamUpper) {
        // Parameter response - extract address and value
        uint16_t addr = data[0] | (data[1] << 8);
        if (addr == AddrVBUSmv && len >= 8) {
            vbus_ = (data[4] | (data[5] << 8)) / 1000.0f; // Convert millivolts to volts
        } else if (addr == AddrVBUSfloat && len >= 8) {
            memcpy(&vbus_, &data[4], 4);
        }
        return true;
    } else {
        if (Serial && Serial.availableForWrite())
        Serial.printf("drive %x: unknown message type %d (header 0x%02x)\n", id_, msgtype, id);
        return true; //still was for us ..
    }
    return false;
}


