#include "dshot_bb.h"
#include <Arduino.h>
#include <driver/gpio.h>
#include <soc/gpio_struct.h>
#include <freertos/FreeRTOS.h>
#include <esp_idf_version.h>
#include <esp32/clk.h>
#if ESP_IDF_VERSION_MAJOR >= 5
#include <esp_cpu.h>
#include <esp_clk_tree.h>
#else
#include <hal/cpu_hal.h>
#endif

static inline uint32_t getCycleCount() {
#if ESP_IDF_VERSION_MAJOR >= 5
    return esp_cpu_get_cycle_count();
#else
    return cpu_hal_get_cycle_count();
#endif
}

DShotBB::DShotBB() {
    uint32_t cpuFreqHz = esp_clk_cpu_freq();
    cpuFreqMhz_ = cpuFreqHz / 1000000;

    t1hCycles_       = nsToCycles(DSHOT_T1H_NS);
    t0hCycles_       = t1hCycles_ >> 1;
    bitPeriodCycles_ = nsToCycles(DSHOT_BIT_PERIOD_NS);
}

uint8_t DShotBB::addPin(uint8_t pin) {
    if (pinCount_ >= MAX_CHANNELS) return UINT8_MAX;
    if (pin >= 32) return UINT8_MAX;  // TODO: GPIO.out1 support for pins 32+

    gpio_reset_pin((gpio_num_t)pin);
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << pin);
    cfg.mode = GPIO_MODE_OUTPUT_OD;  // open-drain for pull-up
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&cfg);

    gpio_set_level((gpio_num_t)pin, 1);  // idle high
    uint8_t channel = pinCount_;
    pinCount_++;
    pinRegs_[channel].pin = pin;
    pinMask_ |= (1UL << pin);
    toSend_[channel] = DSHOT_CMD_MOTOR_STOP;
    return channel;
}

void DShotBB::setThrottle(uint8_t channel, uint16_t throttle, bool telemetry) {
    if (channel >= pinCount_) return;
    telemetry = false; //TODO support bidirectional
    toSend_[channel] = (throttle & 0x7FF) | (telemetry ? 0x800 : 0);
}

void DShotBB::setCmd(uint8_t channel, uint16_t cmd, bool telemetry) {
    if (channel >= pinCount_) return;
    telemetry = false; //TODO support bidirectional
    toSend_[channel] = (cmd & 0x7FF) | (telemetry ? 0x800 : 0);
}

uint16_t DShotBB::buildDShotPacket(uint16_t value11, bool telemetry) {
    uint16_t packet = ((value11 & 0x7FF) << 1) | (telemetry ? 1 : 0);
    uint8_t csum = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    return (packet << 4) | csum;
}

void DShotBB::sendData() {
    uint16_t packets[MAX_CHANNELS];
    for (uint8_t c = 0; c < pinCount_; c++) {
        packets[c] = buildDShotPacket(toSend_[c] & 0x7FF, toSend_[c] & 0x800);
    }

    portDISABLE_INTERRUPTS();
    delayCycles(t0hCycles_); //initial delay to help with first bit jitter
    for (int8_t bitPos = DSHOT_FRAME_BITS - 1; bitPos >= 0; bitPos--) {
        uint32_t zeroMask = 0;
        uint32_t oneMask  = 0;
        for (uint8_t c = 0; c < pinCount_; c++) {
            uint32_t pinBit = (1UL << pinRegs_[c].pin);
            if ((packets[c] >> bitPos) & 1) oneMask |= pinBit;
            else                            zeroMask |= pinBit;
        }

        GPIO.out_w1ts = pinMask_;      // all pins high (mark start of bit)
        delayCycles(t0hCycles_);
        GPIO.out_w1tc = zeroMask;      // drop 0-bits early
        delayCycles(t1hCycles_ - t0hCycles_);
        GPIO.out_w1tc = oneMask;       // drop 1-bits; all pins now low
        delayCycles(bitPeriodCycles_ - t1hCycles_);
    }

    portENABLE_INTERRUPTS();

    // Reset queue -> anything not re-set before next sendData() disarms.
    for (uint8_t c = 0; c < pinCount_; c++) {
        toSend_[c] = DSHOT_CMD_MOTOR_STOP;
    }
}

inline void DShotBB::delayCycles(uint32_t cycles) const {
    uint32_t start = getCycleCount();
    while ((getCycleCount() - start) < cycles);
}
