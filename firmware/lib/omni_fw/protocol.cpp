#include "protocol.h"

#include <string.h>

namespace fw {

uint16_t crc16(const uint8_t* d, size_t n) {
    uint16_t crc = 0xFFFF;                     // CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection
    for (size_t i = 0; i < n; ++i) {
        crc ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; ++b) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

size_t frame_encode(uint8_t type, uint8_t seq, const void* body, size_t body_len, uint8_t* out, size_t out_cap) {
    uint8_t raw[256];
    size_t n = body_len + 4;
    if (n > sizeof(raw) || n + n / 254 + 2 > out_cap) return 0;
    raw[0] = type;
    raw[1] = seq;
    memcpy(raw + 2, body, body_len);
    uint16_t crc = crc16(raw, body_len + 2);
    raw[body_len + 2] = (uint8_t)(crc & 0xFF);
    raw[body_len + 3] = (uint8_t)(crc >> 8);
    // COBS: no zero bytes inside the frame; 0x00 ends it
    size_t w = 1, code_at = 0;
    uint8_t code = 1;
    for (size_t i = 0; i < n; ++i) {
        if (raw[i] == 0) {
            out[code_at] = code;
            code_at = w++;
            code = 1;
        } else {
            out[w++] = raw[i];
            if (++code == 0xFF) {
                out[code_at] = code;
                code_at = w++;
                code = 1;
            }
        }
    }
    out[code_at] = code;
    out[w++] = 0x00;
    return w;
}

bool FrameDecoder::push(uint8_t byte, uint8_t& type, uint8_t* body_out, size_t body_cap, size_t& body_len) {
    if (byte != 0x00) {
        if (n_ < sizeof(buf_)) {
            buf_[n_++] = byte;
        } else {
            overflow_ = true;
        }
        return false;
    }
    // end of frame: COBS-decode in place
    size_t n = n_;
    n_ = 0;
    if (overflow_ || n < 2) {
        if (overflow_ || n > 0) ++errors;
        overflow_ = false;
        return false;
    }
    uint8_t raw[256];
    size_t r = 0, i = 0;
    while (i < n) {
        uint8_t code = buf_[i++];
        if (code == 0 || i + code - 1 > n) {
            ++errors;
            return false;
        }
        for (uint8_t k = 1; k < code; ++k) raw[r++] = buf_[i++];
        if (code < 0xFF && i < n) raw[r++] = 0;
    }
    if (r < 4) {
        ++errors;
        return false;
    }
    uint16_t crc = (uint16_t)raw[r - 2] | ((uint16_t)raw[r - 1] << 8);
    if (crc16(raw, r - 2) != crc) {
        ++errors;
        return false;
    }
    size_t len = r - 4;
    if (len > body_cap) {
        ++errors;
        return false;
    }
    type = raw[0];
    memcpy(body_out, raw + 2, len);
    body_len = len;
    return true;
}

}  // namespace fw
