#pragma once
#include "dshot_bb.h"
#include <Arduino.h>
#include <multimotor/motordrive.h>
#include <multimotor/drive_manager.h>

class ESC;

class ESCManager : public DriveManager {
public:
    ESCManager() { }

    virtual void addDrive(MotorDrive* drive) override;
    void setup(HardwareSerial*);
    virtual MotorDrive* getDrive(uint8_t id) override;
    virtual MotorDrive* const* getDrives() const override { return drives_; }
    virtual uint8_t getCount() const override { return driveCount_; }
    MotorDrive* first() const override { return driveCount_ > 0 ? drives_[0] : nullptr; }

    virtual bool handleIncoming(uint32_t id, uint8_t const* data, uint8_t len, uint32_t now) override;
    virtual bool readOnce(uint32_t now, uint32_t timeout_us) override;
    virtual uint8_t iterate(uint32_t now, uint32_t timeout_ms) override;
    void actuate(uint32_t now) override;

protected:
    friend class ESC;
    static constexpr uint8_t MAX_DRIVES = 4; // 4-in-1 ESC
    MotorDrive* drives_[MAX_DRIVES] = {nullptr};
    uint8_t driveCount_ = 0;
    DShotBB dshot_;
    HardwareSerial* telemport_ = nullptr;
};



class ESC : public MotorDrive {
public:
    ESC(int pwmPin, ESCManager* mgr, const char* name);

    // MotorDrive interface
    uint32_t getId() const override { return pwmPin_; }
    bool requestStatus() override;
    bool setMode(MotorMode mode) override;
    bool supportsMode(MotorMode mode) const override { return mode != MotorMode::Position; }
    bool setSetpoint(MotorMode mode, float value) override;
    bool handleIncoming(uint32_t id, uint8_t const* data, uint8_t len, uint32_t now) override;
    uint32_t getLastStatusTime() const override;
    uint32_t getLastFaults() const override { return 0; }
    MotorState getMotorState() const override;
    bool fetchVBus() override { return true; } // comes in with telem automatically
    float getVBus() const override;

    // Not applicable to PWM-driven ESCs — no bus addressing
    bool writeNewId(uint8_t, bool = true) override { return false; }
    MotorDrive* makeDuplicate(int16_t = -1) const override { return nullptr; }
    bool ping(int timeout_ms = 100) override {
        return (millis() - getLastStatusTime()) < 500; // "alive" = recent telem
    }
    bool validID(int) const override { return false; }
    int16_t discoverNext(bool = true, uint32_t = 100, uint32_t = 1000) override { return -1; }

protected:
    friend class ESCManager;
    int pwmPin_ = -1;
    uint8_t dchannel_ = UINT8_MAX;
    ESCManager* mgr_ = nullptr;
    MotorMode lastmode_ = MotorMode::Disabled;
    float lastSetpoint_ = 0.0f;
};

