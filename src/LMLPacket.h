#pragma once
#include "LMLTypes.h"

namespace lml {

inline uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

inline LMLError encode(const LMLPacket &pkt, uint8_t *buf, size_t bufLen, size_t *outLen) {
    if (pkt.len > LML_MAX_PAYLOAD)
        return LMLError::BAD_LENGTH;

    const size_t frameSize = LML_HEADER_SIZE + pkt.len + LML_CRC_SIZE;
    if (bufLen < frameSize)
        return LMLError::BAD_LENGTH;

    buf[0] = pkt.magic;
    buf[1] = static_cast<uint8_t>(pkt.type);
    buf[2] = pkt.seq;
    buf[3] = pkt.frag;
    buf[4] = pkt.total;
    buf[5] = pkt.len;
    if (pkt.len) memcpy(&buf[6], pkt.payload, pkt.len);

    const uint16_t crc = crc16(buf, LML_HEADER_SIZE + pkt.len);
    buf[LML_HEADER_SIZE + pkt.len]     = static_cast<uint8_t>(crc >> 8);
    buf[LML_HEADER_SIZE + pkt.len + 1] = static_cast<uint8_t>(crc & 0xFF);

    *outLen = frameSize;
    return LMLError::OK;
}

inline LMLError decode(const uint8_t *buf, size_t bufLen, LMLPacket &pkt) {
    if (bufLen < LML_HEADER_SIZE + LML_CRC_SIZE)
        return LMLError::BAD_LENGTH;

    if (buf[0] != LML_MAGIC)
        return LMLError::BAD_MAGIC;

    const uint8_t payloadLen = buf[5];
    if (payloadLen > LML_MAX_PAYLOAD)
        return LMLError::BAD_LENGTH;

    if (bufLen < static_cast<size_t>(LML_HEADER_SIZE + payloadLen + LML_CRC_SIZE))
        return LMLError::BAD_LENGTH;

    const uint16_t expected = crc16(buf, LML_HEADER_SIZE + payloadLen);
    const uint16_t received =
        (static_cast<uint16_t>(buf[LML_HEADER_SIZE + payloadLen]) << 8) |
         static_cast<uint16_t>(buf[LML_HEADER_SIZE + payloadLen + 1]);
    if (expected != received)
        return LMLError::BAD_CRC;

    pkt.magic = buf[0];
    pkt.type  = static_cast<LMLPacketType>(buf[1]);
    pkt.seq   = buf[2];
    pkt.frag  = buf[3];
    pkt.total = buf[4];
    pkt.len   = payloadLen;
    if (payloadLen) memcpy(pkt.payload, &buf[6], payloadLen);

    if (pkt.total == 0 || pkt.frag >= pkt.total)
        return LMLError::BAD_FRAG;

    return LMLError::OK;
}

inline LMLPacket makeCtrl(LMLPacketType type, uint8_t seq) {
    LMLPacket p{};
    p.magic = LML_MAGIC;
    p.type  = type;
    p.seq   = seq;
    p.frag  = 0;
    p.total = 1;
    p.len   = 0;
    return p;
}

} // namespace lml
