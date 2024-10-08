#include "cybergear.h"
#include "can_interface.h"
#include <string.h>
#include <Arduino.h> // just for Serial!

enum Cmds {
    CmdPosition              =  0x1,
    CmdRequest               =  0x2,
    CmdEnable                =  0x3,
    CmdStop                  =  0x4,
    CmdSetMechPositionToZero =  0x6,
    CmdSetCanId              =  0x7,
    CmdRamRead               = 0x11,
    CmdRamWrite              = 0x12,
    CmdGetStatus             = 0x15,
};

enum class Addr {
    RunMode = 0x7005,
    SpeedSetpoint = 0x700A,
    PosSetpoint   = 0x7016,
    VBusMv        = 0x3007,
};

union Payload {
    uint8_t  bytes[8] = {0};
    uint16_t words[4];
    uint32_t dwords[2];
    float    floats[2];
};

CyberGearDriver::CyberGearDriver(uint8_t id, CanInterface* can) : id_(id), can_(can) { }

int16_t CyberGearDriver::getValidDriveIDFromMsg(uint32_t canID, const uint8_t* data, uint8_t len) {
    if (len != 8) return -1;
    uint8_t id = canID & 0xFF;
    uint8_t cmd = (canID >> 24) & 0xFF;
    if (!id || !cmd) return -1;
    if (cmd == CmdRequest) //TODO .. others?
        return id;
    return -1;
}

String CyberGearDriver::getName() const {
    return String("CyberGear") + String(id_);
}

uint32_t mkID(uint8_t cmd, uint8_t opthi, uint8_t optlo, uint8_t id) {
    return (cmd << 24) | (opthi << 16) | (optlo << 8) | id;
}

void CyberGearDriver::requestStatus() {
    uint8_t data[8] = {0x00}; //TODO can this 0 bytes?
    if (can_) can_->send(mkID(CmdGetStatus, 0, 0, id_), data, 8);
}

void CyberGearDriver::requestVBus() {
    Payload p;
    //perhaps send CmdRequest with middle canID bytes as 0x30 0x07?
    //needs testing with the 'scope view' feature of the windows app
    p.words[0] = (uint16_t) Addr::VBusMv;
    if (can_) can_->send(mkID(CmdRamRead, 0, 0, id_), p.bytes, 8, true, true); //ss, rtr
}

void CyberGearDriver::setMode(CyberGearMode mode) {
    uint8_t data[8] = { Addr::RunMode & 0x00FF, Addr::RunMode >> 8, 0x00, 0x00, (uint8_t) mode, 0x00, 0x00, 0x00};
    if (can_) can_->send(mkID(CmdRamWrite, 0, 0, id_), data, 8);
}

void CyberGearDriver::enable(bool enable) {
    uint8_t data[8] = {0x00};
    if (can_) can_->send(mkID(enable? CmdEnable : CmdStop, 0, 0, id_), data, 8);
}

void CyberGearDriver::setPos(float pos) {
    uint8_t data[8] = { Addr::PosSetpoint & 0x00FF, Addr::PosSetpoint >> 8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    memcpy(&data[4], &pos, 4);
    if (can_) can_->send(mkID(CmdRamWrite, 0, 0, id_), data, 8);
}

void CyberGearDriver::setSpeed(float speed) {
    uint8_t data[8] = { Addr::SpeedSetpoint & 0x00FF, Addr::SpeedSetpoint >> 8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    memcpy(&data[4], &speed, 4);
    if (can_) can_->send(mkID(CmdRamWrite, 0, 0, id_), data, 8);
}

bool CyberGearDriver::handleIncoming(uint32_t id, const uint8_t* data, uint8_t len, uint32_t now) {
    uint8_t msgtype = (id & 0xFF000000) >> 24; //bits 24-28
    uint8_t driveid = (id & 0x0000FF00) >> 8; //bits 8-15
    if (driveid != id_) return false;
    if (msgtype == CmdRequest) { //status message reply!
        uint8_t mode  = (id & 0x00C00000) >> 22; //bits 22-23 = [mode]
        lastFaults_   = (id & 0x003F0000) >> 16; //bits 16-21 = [Uncalibrated, hall, magsense, overtemp, overcurrent, undervolt]
        //TODO read data payload
        enabled_ = (mode == 2);
        lastStatusTime_ = now;
        if (Serial && Serial.availableForWrite())
            Serial.print(".");
        return true;
    } else if (msgtype == 0x15) { //fault message (decimal 21)
        if (Serial && Serial.availableForWrite()) {
            Serial.printf("drive %x: fault message %x: {", id_, id);
            for (int i = 0; i < len; i++)
                Serial.printf("0x%02x, ", data[i]);
            Serial.println("}");
        }
        return true;
    } else {
        if (Serial && Serial.availableForWrite())
        Serial.printf("drive %x: unknown message type %d (header 0x%02x)\n", id_, msgtype, id);
        return true; //still was for us ..
    }
    return false;
}


