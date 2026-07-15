#include "NowPacket.h"
#include "string.h"

namespace comms {

    uint8_t NowPacket::serialise(uint8_t t, uint8_t chan, const uint8_t* payload, uint8_t len, uint8_t* dst, uint8_t destmax) {
        if (destmax < HEADER_OVERHEAD + len) return 0;
        dst[0] = t;
        dst[1] = chan;
        dst[2] = len;
        if (len && payload) memcpy(dst + 3, payload, len);
        dst[HEADER_OVERHEAD + len - 1] = checksum(dst, HEADER_OVERHEAD + len - 1);
        return HEADER_OVERHEAD + len;
    }

    bool NowPacket::parse(const uint8_t* src, uint8_t len) {
        if (len < HEADER_OVERHEAD || len > MAX_PACKET) return false;
        uint8_t chk = checksum(src, len - 1);
        if (chk != src[len - 1]) return false; // invalid checksum
        type       = src[0];
        chan       = src[1];
        payloadLen = src[2];
        if (payloadLen) payload = const_cast<uint8_t*>(src + 3);
        return true;
    }

    uint8_t NowPacket::checksum(const uint8_t* buf, uint16_t len, uint8_t startval) {
        // CRC-8, polynomial 0x07 (x^8 + x^2 + x^1 + 1)
        uint8_t crc = startval;
        for (uint16_t i = 0; i < len; i++) {
            crc ^= buf[i];
            for (uint8_t j = 0; j < 8; j++) {
                if (crc & 0x80) crc = (crc << 1) ^ 0x07;
                else crc <<= 1;
            }
        }
        return crc;
    }

} // namespace comms
