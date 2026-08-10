#include "dshot_bb.h"
#include <log.h>
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
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode((gpio_num_t)pin, GPIO_PULLUP_ONLY);
    gpio_set_level((gpio_num_t)pin, 0); //idle low for unidirectional
    uint8_t channel = pinCount_;
    pinCount_++;
    channels_[channel].pin = pin;
    channels_[channel].index = channel;
    channels_[channel].bidirectional = false;
    pinMask_ |= (1UL << pin);
    toSend_[channel] = DSHOT_CMD_MOTOR_STOP;
    return channel;
}

void DShotBB::setThrottle(uint8_t channel, uint16_t throttle, bool telemetry) {
    if (channel >= pinCount_) return;
    toSend_[channel] = (throttle & 0x7FF) | (telemetry ? 0x800 : 0);
}

void DShotBB::setCmd(uint8_t channel, uint16_t cmd, bool telemetry) {
    if (channel >= pinCount_) return;
    toSend_[channel] = (cmd & 0x7FF) | (telemetry ? 0x800 : 0);
}

uint16_t DShotBB::buildDShotPacket(uint16_t value11, bool telemetry, bool inverted) {
    uint16_t packet = ((value11 & 0x7FF) << 1) | (telemetry ? 1 : 0);
    uint8_t csum = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    if (inverted) csum = (~csum) & 0x0F;
    return (packet << 4) | csum;
}

void DShotBB::sendData() {
    uint16_t packets[MAX_CHANNELS];
    lastKissTelemReq_ = (lastKissTelemReq_ + 1) % pinCount_;  // round-robin for next time
    for (uint8_t c = 0; c < pinCount_; c++) {
        bool telem = toSend_[c] & 0x800;
        if (telem && !channels_[c].bidirectional) {
            telem = (lastKissTelemReq_ == c); //needs round-robin, max one telem req per period
        }
        bool inverted = channels_[c].bidirectional;
        packets[c] = buildDShotPacket(toSend_[c] & 0x7FF, telem, inverted);
        if (telem) resetTelemRead();
    }

    portDISABLE_INTERRUPTS();
    delayCycles(t0hCycles_); //initial delay to help with first bit jitter
    for (int8_t bitPos = DSHOT_FRAME_BITS - 1; bitPos >= 0; bitPos--) {
        uint32_t zeroMask = 0;
        uint32_t oneMask  = 0;
        for (uint8_t c = 0; c < pinCount_; c++) {
            uint32_t pinBit = (1UL << channels_[c].pin);
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

    //last: restore all pins based bidirectional state
    for (uint8_t c = 0; c < pinCount_; c++) {
        bool high = channels_[c].bidirectional;
        if (high) GPIO.out_w1ts = (1UL << channels_[c].pin);
        else      GPIO.out_w1tc = (1UL << channels_[c].pin);
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

void DShotBB::resetTelemRead() {
    telembuf_.clear();
}

uint8_t DShotBB::KissPkt::calcCrc(uint8_t const* data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t crc_u = crc ^ data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc_u = (crc_u & 0x80) ? 0x7 ^ (crc_u << 1) : (crc_u << 1);
        crc = crc_u;
    }
    return crc;
}

DShotBB::KissPkt DShotBB::KissPkt::read(uint8_t const* data, uint8_t len) {
    return (KissPkt) {
        .temp = data[0],
        .voltage = ((data[1] << 8) | data[2]) / 100.0f,
        .current = ((data[3] << 8) | data[4]) / 100.0f,
        .mah_used = (float)((data[5] << 8) | data[6]),
        .rpm = ((data[7] << 8) | data[8]) / 7.0f, //assuming 14 pole motor
    };
}

DShotBB::Channel const* DShotBB::readTelem(uint32_t now, HardwareSerial* port) {
    if (!port || !port->available()) {
        return nullptr;
    }
    port->setTimeout(1);  // milliseconds
    constexpr uint8_t PKTSIZE = 10;
    uint8_t buf[PKTSIZE] = {0};
    size_t n = port->readBytes(buf, PKTSIZE);
    auto crc = KissPkt::calcCrc(buf, n - 1); //skip existing crc
    auto pkt = KissPkt::read(buf, n);
    // Serial.printf(" -> [%d] %dC %0.1fV %0.1fA %0.1fmah %0.1fRPM\n", lastKissTelemReq_, pkt.temp, pkt.voltage, pkt.current, pkt.mah_used, pkt.rpm);
    // Serial.flush();
    if (crc == buf[PKTSIZE - 1]) { //valid
        auto & ch = channels_[lastKissTelemReq_];
        ch.lastTelem = now;
        ch.telem = pkt;
        return &ch;
    } else {
        port->flush(); //clear misaligned incoming
        return nullptr;
    }
}
