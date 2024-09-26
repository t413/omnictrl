#pragma once
#include "can_interface.h"
#include <Arduino.h>

struct CanMessage {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
};

class CanEsp32Twai : public CanInterface {
public:
    virtual void setup(uint8_t rx, uint8_t tx, Stream* debug = nullptr);
    virtual void send(uint32_t id, uint8_t* data, uint8_t len, bool ss = true);
    virtual bool available();
    virtual CanMessage readOne();
    virtual String getAlerts();
};
