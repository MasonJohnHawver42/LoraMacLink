#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "MSRCTypes.h"
#include "MSRCPacket.h"

// Holds pre-fragmented packets for one enqueued message.
// Only one message is staged at a time; send() from the app task writes here
// while poll() drains it fragment by fragment.
class MSRCTxQueue {
public:
    MSRCTxQueue() : _count(0), _next(0), _seq(0) {
        _lock = xSemaphoreCreateMutex();
    }

    ~MSRCTxQueue() { vSemaphoreDelete(_lock); }

    // Fragment message into packets and stage them. Returns MSG_TOO_BIG if
    // len > MSRC_MAX_MESSAGE, QUEUE_FULL if a message is already staged.
    MSRCError enqueue(const uint8_t *data, size_t len) {
        if (len > MSRC_MAX_MESSAGE) return MSRCError::MSG_TOO_BIG;

        xSemaphoreTake(_lock, portMAX_DELAY);
        MSRCError err = MSRCError::OK;

        if (_count > 0) {
            err = MSRCError::QUEUE_FULL;
        } else {
            const uint8_t total = static_cast<uint8_t>(
                (len + MSRC_MAX_PAYLOAD - 1) / MSRC_MAX_PAYLOAD);

            for (uint8_t i = 0; i < total; ++i) {
                MSRCPacket &p = _pkts[i];
                p.magic = MSRC_MAGIC;
                p.type  = MSRCPacketType::DATA;
                p.seq   = _seq;
                p.frag  = i;
                p.total = total;

                const size_t offset    = i * MSRC_MAX_PAYLOAD;
                const size_t remaining = len - offset;
                p.len = static_cast<uint8_t>(
                    remaining < MSRC_MAX_PAYLOAD ? remaining : MSRC_MAX_PAYLOAD);
                memcpy(p.payload, data + offset, p.len);
            }

            _count = total;
            _next  = 0;
            ++_seq;
        }

        xSemaphoreGive(_lock);
        return err;
    }

    // Peek at the next unsent packet. Returns EMPTY if none pending.
    MSRCError peek(MSRCPacket &out) const {
        xSemaphoreTake(_lock, portMAX_DELAY);
        MSRCError err = MSRCError::OK;
        if (_next >= _count) {
            err = MSRCError::EMPTY;
        } else {
            out = _pkts[_next];
        }
        xSemaphoreGive(_lock);
        return err;
    }

    // Advance past the current packet after successful ACK.
    void advance() {
        xSemaphoreTake(_lock, portMAX_DELAY);
        if (_next < _count) ++_next;
        if (_next >= _count) { _count = 0; _next = 0; }
        xSemaphoreGive(_lock);
    }

    bool empty() const {
        xSemaphoreTake(_lock, portMAX_DELAY);
        const bool e = (_next >= _count);
        xSemaphoreGive(_lock);
        return e;
    }

    // Discard all staged fragments (used when a send attempt is abandoned).
    void reset() {
        xSemaphoreTake(_lock, portMAX_DELAY);
        _count = 0;
        _next  = 0;
        xSemaphoreGive(_lock);
    }

private:
    // Max fragments = MSRC_MAX_MESSAGE / MSRC_MAX_PAYLOAD = 512/64 = 8
    static constexpr uint8_t MAX_FRAGS = MSRC_MAX_MESSAGE / MSRC_MAX_PAYLOAD;

    MSRCPacket _pkts[MAX_FRAGS];
    uint8_t    _count;
    uint8_t    _next;
    uint8_t    _seq;
    mutable SemaphoreHandle_t _lock;
};
