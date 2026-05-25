#pragma once
#include <stdint.h>
#include <stddef.h>

constexpr size_t LML_MAX_MESSAGE = 512;

struct LMLMessage {
    uint8_t  data[LML_MAX_MESSAGE];
    size_t   len;
    float    rssi;
    uint32_t timestamp_ms;
};

// Wire frame layout:
//   [0]        magic   = LML_MAGIC
//   [1]        type    (LMLPacketType)
//   [2]        seq     wrapping 0-255 per-message sequence
//   [3]        frag    fragment index (0-based)
//   [4]        total   total fragments (1 = unfragmented)
//   [5]        len     payload bytes that follow (0..LML_MAX_PAYLOAD)
//   [6..5+len] payload
//   [last 2]   CRC16-CCITT over bytes [0..5+len-1]

constexpr uint8_t  LML_MAGIC       = 0xAC;
constexpr uint8_t  LML_HEADER_SIZE = 6;
constexpr uint8_t  LML_CRC_SIZE    = 2;
constexpr size_t   LML_MAX_PAYLOAD = 64;

enum class LMLPacketType : uint8_t {
    RTS  = 0x01,   // initiator → responder: request-to-send
    CTS  = 0x02,   // responder → initiator: clear-to-send
    NACK = 0x03,   // receiver: bad/unexpected frame, sender should retry
    DATA = 0x04,   // either direction: message fragment
    ACK  = 0x05,   // receiver: fragment acknowledged
    HALT = 0x06,   // either direction: max retries hit, abort to Standby
};

struct LMLPacket {
    uint8_t       magic;
    LMLPacketType type;
    uint8_t       seq;
    uint8_t       frag;
    uint8_t       total;
    uint8_t       len;
    uint8_t       payload[LML_MAX_PAYLOAD];
    uint16_t      crc16;
};

enum class LMLError : int8_t {
    OK          =  0,
    QUEUE_FULL  = -1,
    BAD_MAGIC   = -2,
    BAD_CRC     = -3,
    BAD_FRAG    = -4,
    BAD_LENGTH  = -5,
    TIMEOUT     = -6,
    RADIO_ERR   = -7,
    MSG_TOO_BIG = -8,
    EMPTY       = -9,
};

constexpr uint32_t LML_LINK_TIMEOUT_MS   = 6000;
constexpr uint8_t  LML_MAX_RETRIES       = 3;
constexpr uint32_t LML_ACK_TIMEOUT_MS    = 500;
constexpr uint32_t LML_STATUS_TIMEOUT_MS = 5000;
