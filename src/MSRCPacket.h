#pragma once
#include "MSRCTypes.h"

namespace msrc_internal {

// CRC16-CCITT (poly 0x1021, init 0xFFFF)
inline uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
    return crc;
}

// Serialise pkt into buf.
// On success returns MSRCError::OK and writes total bytes to *outLen.
// *outLen is not modified on failure.
inline MSRCError encode(const MSRCPacket &pkt, uint8_t *buf, size_t bufLen, size_t *outLen) {
    if (pkt.len > MSRC_MAX_PAYLOAD)
        return MSRCError::BAD_LENGTH;

    const size_t frameSize = MSRC_HEADER_SIZE + pkt.len + MSRC_CRC_SIZE;
    if (bufLen < frameSize)
        return MSRCError::BAD_LENGTH;

    buf[0] = pkt.magic;
    buf[1] = static_cast<uint8_t>(pkt.type);
    buf[2] = pkt.seq;
    buf[3] = pkt.frag;
    buf[4] = pkt.total;
    buf[5] = pkt.len;
    if (pkt.len) memcpy(&buf[6], pkt.payload, pkt.len);

    const uint16_t crc = crc16(buf, MSRC_HEADER_SIZE + pkt.len);
    buf[MSRC_HEADER_SIZE + pkt.len]     = static_cast<uint8_t>(crc >> 8);
    buf[MSRC_HEADER_SIZE + pkt.len + 1] = static_cast<uint8_t>(crc & 0xFF);

    *outLen = frameSize;
    return MSRCError::OK;
}

// Deserialise buf into pkt. Returns an MSRCError describing any failure.
inline MSRCError decode(const uint8_t *buf, size_t bufLen, MSRCPacket &pkt) {
    if (bufLen < MSRC_HEADER_SIZE + MSRC_CRC_SIZE)
        return MSRCError::BAD_LENGTH;

    if (buf[0] != MSRC_MAGIC)
        return MSRCError::BAD_MAGIC;

    const uint8_t payloadLen = buf[5];
    if (payloadLen > MSRC_MAX_PAYLOAD)
        return MSRCError::BAD_LENGTH;

    if (bufLen < static_cast<size_t>(MSRC_HEADER_SIZE + payloadLen + MSRC_CRC_SIZE))
        return MSRCError::BAD_LENGTH;

    const uint16_t expected = crc16(buf, MSRC_HEADER_SIZE + payloadLen);
    const uint16_t received =
        (static_cast<uint16_t>(buf[MSRC_HEADER_SIZE + payloadLen]) << 8) |
         static_cast<uint16_t>(buf[MSRC_HEADER_SIZE + payloadLen + 1]);
    if (expected != received)
        return MSRCError::BAD_CRC;

    pkt.magic = buf[0];
    pkt.type  = static_cast<MSRCPacketType>(buf[1]);
    pkt.seq   = buf[2];
    pkt.frag  = buf[3];
    pkt.total = buf[4];
    pkt.len   = payloadLen;
    if (payloadLen) memcpy(pkt.payload, &buf[6], payloadLen);

    if (pkt.total == 0 || pkt.frag >= pkt.total)
        return MSRCError::BAD_FRAG;

    return MSRCError::OK;
}

// Build a zero-payload control packet.
inline MSRCPacket makeControl(MSRCPacketType type, uint8_t seq) {
    MSRCPacket p{};
    p.magic = MSRC_MAGIC;
    p.type  = type;
    p.seq   = seq;
    p.frag  = 0;
    p.total = 1;
    p.len   = 0;
    return p;
}

} // namespace msrc_internal
