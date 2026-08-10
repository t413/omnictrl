#include "esc.h"
#include <log.h>
#include <utils.h>

void ESCManager::setup(HardwareSerial* telemport) {
    telemport_ = telemport;
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

uint8_t ESCManager::iterate(uint32_t now, uint32_t /*timeout_ms*/) {
    auto chan = dshot_.readTelem(now, telemport_);
    return 0;
}

bool ESCManager::readOnce(uint32_t now, uint32_t /*timeout_us*/) { return false; }
bool ESCManager::handleIncoming(uint32_t, uint8_t const*, uint8_t, uint32_t) { return false; }

void ESCManager::actuate(uint32_t now) {
    dshot_.sendData();
}



ESC::ESC(int pwmPin, ESCManager* mgr, const char* name) : MotorDrive(name), pwmPin_(pwmPin), mgr_(mgr) {
    if (mgr_) mgr_->addDrive(this); // registers
}

bool ESC::requestStatus() { return true; }

bool ESC::setMode(MotorMode mode) {
    lastmode_ = mode;
    return supportsMode(mode);
}

bool ESC::setSetpoint(MotorMode mode, float value) {
    if (!mgr_) return false;
    if (mode == MotorMode::Disabled) {
        mgr_->dshot_.setCmd(dchannel_, DShotBB::DSHOT_CMD_MOTOR_STOP);
        return true;
    }
    if (mode != lastmode_) setMode(mode);
    lastSetpoint_ = value;
    value *= (mode == MotorMode::Speed)? 0.03f : 0.20f;
    float v = constrain(value, -1.0f, 1.0f);
    // Reverse: map [-1, 0] to [48, 1047]
    uint16_t tv = (abs(v) * 999.0f);
    uint16_t thr = ((v > 0.0f)? DShotBB::DSHOT_THR_MID : DShotBB::DSHOT_THR_MIN) + tv;
    //TODO add a deadband for +-1 and send sendCommand(DSHOT_CMD_MOTOR_STOP) instead
    mgr_->dshot_.setThrottle(dchannel_, thr);
    return true;
}

uint32_t ESC::getLastStatusTime() const {
    return mgr_ ? mgr_->dshot_.getChannel(dchannel_).lastTelem : 0;
}

float ESC::getVBus() const {
    return mgr_ ? mgr_->dshot_.getChannel(dchannel_).telem.voltage : 0.0f;
}

MotorState ESC::getMotorState() const {
    if (!mgr_) return MotorState();
    const auto& chan = mgr_->dshot_.getChannel(dchannel_);
    MotorState ret;
    ret.temperature = chan.telem.temp;
    //TODO? ret.position = 0.0f;
    const float rpmToRadPerSec = 2.0f * M_PI / 60.0f;
    ret.velocity = chan.telem.rpm * rpmToRadPerSec * (lastSetpoint_ > 0.0f ? 1.0f : -1.0f); //rpm value is abs(rpm)
    ret.torque = chan.telem.current;
    ret.mode = lastmode_;
    return ret;
}

bool ESC::handleIncoming(uint32_t /*id*/, uint8_t const* data, uint8_t len, uint32_t now) {
    return true;
}
