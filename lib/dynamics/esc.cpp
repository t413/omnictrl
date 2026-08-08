#include "esc.h"
#include <log.h>
#include <utils.h>

void ESCManager::setup() {
    uint8_t done = 0;
    for (uint8_t i = 0; i < driveCount_; ++i) {
        if (! drives_[i]) continue;
        auto* esc = static_cast<ESC*>(drives_[i]);
        D_LOG("mgr setting up ESC #%d id %d %s", i, esc->getId(), esc->getName());
        esc->dchannel_ = dshot_.addPin(esc->pwmPin_);
        done++;
    }
    D_LOG("ESCManager setup complete, %u drives", done);
}

void ESCManager::addDrive(MotorDrive* drive) {
    if (driveCount_ < MAX_DRIVES) {
        drives_[driveCount_++] = drive;
    }
}

MotorDrive* ESCManager::getDrive(uint8_t id) {
    for (uint8_t i = 0; i < driveCount_; ++i)
        if (drives_[i] && drives_[i]->getId() == id)
            return drives_[i];
    return nullptr;
}

uint8_t ESCManager::iterate(uint32_t now, uint32_t /*timeout_ms*/) { return 0; }
bool ESCManager::readOnce(uint32_t now, uint32_t /*timeout_us*/) { return false; }
bool ESCManager::handleIncoming(uint32_t, uint8_t const*, uint8_t, uint32_t) { return false; }

void ESCManager::actuate(uint32_t now) {
    dshot_.sendData();
    // for (uint8_t i = 0; i < driveCount_; ++i) {
    //     if (! drives_[i]) continue;
    //     auto* esc = static_cast<ESC*>(drives_[i]);
    //     esc->handleTelem(dshot_.getTelemetry(esc->dchannel_));
    // }
}



ESC::ESC(int pwmPin, ESCManager* mgr, const char* name) : MotorDrive(name), pwmPin_(pwmPin), mgr_(mgr) {
    if (mgr_) mgr_->addDrive(this); // registers
}

bool ESC::requestStatus() { return true; }

bool ESC::setMode(MotorMode mode) {
    lastState_.mode = mode;
    return supportsMode(mode);
}

bool ESC::setSetpoint(MotorMode mode, float value) {
    if (!mgr_) return false;
    if (mode == MotorMode::Disabled) {
        mgr_->dshot_.setCmd(dchannel_, DShotBB::DSHOT_CMD_MOTOR_STOP);
        return true;
    }
    if (mode != lastState_.mode) setMode(mode);
    value *= (mode == MotorMode::Speed)? 0.03f : 0.02f;
    float v = constrain(value, -1.0f, 1.0f);
    // Reverse: map [-1, 0] to [48, 1047]
    uint16_t tv = (abs(v) * 999.0f);
    uint16_t thr = ((v > 0.0f)? DShotBB::DSHOT_THR_MID : DShotBB::DSHOT_THR_MIN) + tv;
    lastState_.torque = v;
    lastState_.velocity = thr;
    //TODO add a deadband for +-1 and send sendCommand(DSHOT_CMD_MOTOR_STOP) instead
    mgr_->dshot_.setThrottle(dchannel_, thr);
    return true;
}

// void ESC::handleTelem(const DShotBB::Telem& telem) {
//     lastTelemTime_ = telem.time_ms;
//     lastState_.temperature = telem.temperature_C;
//     lastState_.velocity = telem.erpm;
//     vbus_ = telem.voltage_mV;
// }

bool ESC::handleIncoming(uint32_t /*id*/, uint8_t const* data, uint8_t len, uint32_t now) {
    return true;
}
