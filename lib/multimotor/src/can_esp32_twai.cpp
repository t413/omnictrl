#include "can_esp32_twai.h"

#if defined(ARDUINO) && defined(ESP32)
#include <Arduino.h>
#include <driver/twai.h>

void CanEsp32Twai::setup(uint8_t rx, uint8_t tx, int baudrate, Stream* debug) {
    twai_status_info_t twai_status;
    twai_get_status_info(&twai_status);
    if (twai_status.state == TWAI_STATE_RUNNING)
        return;

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)tx, (gpio_num_t)rx, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    switch (baudrate) {
        case 1000000: t_config = TWAI_TIMING_CONFIG_1MBITS(); break;
        case 500000:  t_config = TWAI_TIMING_CONFIG_500KBITS(); break;
        case 250000:  t_config = TWAI_TIMING_CONFIG_250KBITS(); break;
        case 125000:  t_config = TWAI_TIMING_CONFIG_125KBITS(); break;
        case 100000:  t_config = TWAI_TIMING_CONFIG_100KBITS(); break;
        case 50000:   t_config = TWAI_TIMING_CONFIG_50KBITS(); break;
        case 20000:   t_config = TWAI_TIMING_CONFIG_20KBITS(); break;
        default:
            t_config = TWAI_TIMING_CONFIG_1MBITS();
            if (debug) debug->println("Unknown baudrate, defaulting to 1M");
    }

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        if (debug) debug->println("Failed to install driver");
        return;
    }
    if (twai_start() != ESP_OK) {
        if (debug) debug->println("Failed to start driver");
        return;
    }
    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR;
    if (twai_reconfigure_alerts(alerts_to_enable, NULL) != ESP_OK) {
        if (debug) debug->println("Failed to reconfigure alerts");
        return;
    }
}
void CanEsp32Twai::send(uint32_t id, uint8_t* data, uint8_t len, bool extended, bool ss, bool rtr) {
    twai_message_t message = {0};
    message.extd = extended? 1 : 0; //enable extended frame format
    message.ss = ss; //enable single shot transmission
    message.rtr = rtr; //enable remote transmission request
    message.identifier = id;
    message.data_length_code = len;
    memcpy(message.data, data, len);
    #if 0
    if (Serial && Serial.availableForWrite()) {
        Serial.printf(" > tx 0x%08x len=%d: ", id, len);
        for (int i = 0; i < len; i++)
            Serial.printf("0x%02x, ", data[i]);
        Serial.println();
    }
    #endif
    twai_transmit(&message, pdMS_TO_TICKS(1));
}

bool CanEsp32Twai::available() {
    uint32_t alerts = 0;
    twai_read_alerts(&alerts, pdMS_TO_TICKS(1));
    return alerts & TWAI_ALERT_RX_DATA;
}

CanMessage CanEsp32Twai::readOne() {
    twai_message_t message = {0};
    if (twai_receive(&message, pdMS_TO_TICKS(1)) != ESP_OK)
        return CanMessage{0, {0}, 0};
    CanMessage ret;
    ret.id = message.identifier;
    ret.len = message.data_length_code;
    memcpy(ret.data, message.data, ret.len);
    return ret;
}

String CanEsp32Twai::getAlerts() {
    uint32_t alerts = 0;
    twai_read_alerts(&alerts, pdMS_TO_TICKS(1));
    String ret;
    // if (alerts & TWAI_ALERT_RX_DATA) ret += "RX ";
    // if (alerts & TWAI_ALERT_TX_SUCCESS) ret += "TX_SUCCESS ";
    if (alerts & TWAI_ALERT_TX_IDLE) ret += "TX_IDLE ";
    if (alerts & TWAI_ALERT_TX_FAILED) ret += "TX_FAILED ";
    if (alerts & TWAI_ALERT_ERR_PASS) ret += "ERR_PASS ";
    if (alerts & TWAI_ALERT_BUS_ERROR) ret += "BUS_ERROR ";
    return ret;
}

#endif //ARDUINO && ESP32
