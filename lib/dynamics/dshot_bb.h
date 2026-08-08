#pragma once
#include <stdint.h>

class DShotBB {
public:
    static constexpr uint32_t DSHOT_BIT_PERIOD_NS = 3333;  // DShot300 full bit period
    static constexpr uint32_t DSHOT_T1H_NS        = 2500;  // 1-bit high time
    static constexpr uint32_t DSHOT_T0H_NS        = DSHOT_T1H_NS / 2;  // T0H is always half of T1H
    static constexpr uint8_t  DSHOT_FRAME_BITS    = 16;
    static constexpr uint16_t DSHOT_CMD_MOTOR_STOP = 0;
    static constexpr uint16_t DSHOT_THR_MIN = 48;
    static constexpr uint16_t DSHOT_THR_MID = 1048;
    static constexpr uint16_t DSHOT_THR_MAX = 2047;
    static constexpr uint8_t  MAX_CHANNELS        = 8;

    DShotBB();

    uint8_t addPin(uint8_t pin); //returns channel index to use

    // Queue next-frame values per channel; nothing is sent until sendData().
    void setThrottle(uint8_t channel, uint16_t throttle, bool telemetry = false);
    void setCmd(uint8_t channel, uint16_t cmd, bool telemetry = false);

    // Sends one frame on every added pin simultaneously, then resets
    // all channels to motor-stop for the next call.
    void sendData();

private:
    struct PinReg {
        uint8_t pin;
    };

    PinReg pinRegs_[MAX_CHANNELS] = {};
    uint8_t pinCount_ = 0;
    uint32_t pinMask_ = 0;

    // Packed 12-bit value awaiting send: bits[10:0]=value, bit[11]=telemetry
    uint16_t toSend_[MAX_CHANNELS] = {};

    uint32_t cpuFreqMhz_ = 0;
    uint32_t t0hCycles_ = 0;
    uint32_t t1hCycles_ = 0;
    uint32_t bitPeriodCycles_ = 0;

    static uint16_t buildDShotPacket(uint16_t value11, bool telemetry);

    inline uint32_t nsToCycles(uint32_t ns) const {
        return (ns * cpuFreqMhz_) / 1000;
    }

    void delayCycles(uint32_t cycles) const;
};

