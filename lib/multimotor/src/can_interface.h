#pragma once
#include <stdint.h>


class CanInterface {
public:
    virtual void send(uint32_t id, uint8_t* data, uint8_t len, bool extended, bool ss = true, bool rtr = false) = 0;
};

