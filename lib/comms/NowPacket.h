#pragma once
#include <stdint.h>

namespace comms {

    // Max ESP-NOW payload is 250 bytes; 2-byte header leaves 248 for audio frames.
    static constexpr uint8_t HEADER_OVERHEAD = 4;
    static constexpr uint8_t MAX_PACKET      = 250;
    static constexpr uint8_t MAX_PAYLOAD     = MAX_PACKET - HEADER_OVERHEAD;

    // Wire layout: [type:u8][chan:u8][payload...]
    struct NowPacket {
        uint8_t type = 0;
        uint8_t subt = 0;
        uint8_t payloadLen = 0;
        uint8_t* payload = nullptr;
        uint8_t mac[6] = {0}; //not transmitted; set by receiver for peer tracking

        static uint8_t serialise(uint8_t, uint8_t subt, const uint8_t* payload, uint8_t len, uint8_t* dst, uint8_t destmax);
        bool parse(const uint8_t* src, uint8_t len);
        static uint8_t checksum(const uint8_t* buf, uint16_t len, uint8_t startval = 0);
    };

} // namespace comms
