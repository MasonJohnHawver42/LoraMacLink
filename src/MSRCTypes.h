#pragma once
#include <stdint.h>
#include <stddef.h>

constexpr size_t MSRC_MAX_MESSAGE = 512;

struct MSRCMessage {
    uint8_t  data[MSRC_MAX_MESSAGE];
    size_t   len;
    float    rssi;
    uint32_t timestamp_ms;
};

// ── Packet layout ──────────────────────────────────────────────────────────
// [0]        magic   = 0xAC
// [1]        type    (MSRCPacketType)
// [2]        seq     wrapping 0-255 sequence number
// [3]        frag    fragment index (0-based within current message)
// [4]        total   total fragments in this message (1 = unfragmented)
// [5]        len     payload bytes that follow (0..MSRC_MAX_PAYLOAD)
// [6..5+len] payload bytes
// [last 2]   CRC16-CCITT over bytes [0..5+len-1]
//
// Error-correction within payload bytes is the caller's responsibility.
// The library detects corruption via CRC and retries; it does not repair data.

constexpr uint8_t  MSRC_MAGIC       = 0xAC;
constexpr uint8_t  MSRC_HEADER_SIZE = 6;
constexpr uint8_t  MSRC_CRC_SIZE    = 2;
constexpr size_t   MSRC_MAX_PAYLOAD = 64;

enum class MSRCPacketType : uint8_t {
    RTS  = 0x01,   // master → slave: request to send
    CTS  = 0x02,   // slave  → master: clear to send
    // NACK is sent for ANY rejection: slave busy, bad magic, CRC mismatch,
    // or malformed header. The master does not need to know the reason.
    NACK = 0x03,
    DATA = 0x04,   // either direction: message fragment
    ACK  = 0x05,   // receiver → sender: fragment acknowledged
    HB   = 0x06,   // heartbeat (either direction)
};

struct MSRCPacket {
    uint8_t        magic;
    MSRCPacketType type;
    uint8_t        seq;
    uint8_t        frag;
    uint8_t        total;
    uint8_t        len;
    uint8_t        payload[MSRC_MAX_PAYLOAD];
    uint16_t       crc16;   // CRC16-CCITT over all fields above (excluding crc16 itself)
};

enum class MSRCError : int8_t {
    OK              =  0,
    QUEUE_FULL      = -1,   // TX or RX ring buffer is full
    NO_CTS          = -2,   // send() attempted without CTS granted
    BAD_MAGIC       = -3,   // first byte was not MSRC_MAGIC
    BAD_CRC         = -4,   // CRC16 mismatch
    BAD_FRAG        = -5,   // frag >= total, or total == 0
    BAD_LENGTH      = -6,   // declared payload length exceeds buffer
    TIMEOUT         = -7,   // ACK or CTS not received within deadline
    RADIO_ERR       = -8,   // RadioLib returned an error code
    MSG_TOO_BIG     = -9,   // message exceeds MSRC_MAX_MESSAGE
    EMPTY           = -10,  // read() called on an empty buffer
};

constexpr uint32_t MSRC_HEARTBEAT_INTERVAL_MS = 2000;
constexpr uint32_t MSRC_LINK_TIMEOUT_MS       = 6000;
constexpr uint8_t  MSRC_MAX_RETRIES           = 3;
constexpr uint32_t MSRC_ACK_TIMEOUT_MS        = 400;
