#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "LMLTypes.h"
#include "LMLPacket.h"

class LMLTxQueue {
public:
    LMLTxQueue() : _count(0), _next(0), _seq(0) {
        _lock = xSemaphoreCreateMutex();
    }
    ~LMLTxQueue() { vSemaphoreDelete(_lock); }

    LMLError enqueue(const uint8_t *data, size_t len) {
        if (len > LML_MAX_MESSAGE) return LMLError::MSG_TOO_BIG;

        xSemaphoreTake(_lock, portMAX_DELAY);
        LMLError err = LMLError::OK;

        if (_count > 0) {
            err = LMLError::QUEUE_FULL;
        } else {
            const uint8_t total = static_cast<uint8_t>(
                (len + LML_MAX_PAYLOAD - 1) / LML_MAX_PAYLOAD);

            for (uint8_t i = 0; i < total; ++i) {
                LMLPacket &p = _pkts[i];
                p.magic = LML_MAGIC;
                p.type  = LMLPacketType::DATA;
                p.seq   = _seq;
                p.frag  = i;
                p.total = total;

                const size_t offset    = i * LML_MAX_PAYLOAD;
                const size_t remaining = len - offset;
                p.len = static_cast<uint8_t>(
                    remaining < LML_MAX_PAYLOAD ? remaining : LML_MAX_PAYLOAD);
                memcpy(p.payload, data + offset, p.len);
            }

            _count = total;
            _next  = 0;
            ++_seq;
        }

        xSemaphoreGive(_lock);
        return err;
    }

    LMLError peek(LMLPacket &out) const {
        xSemaphoreTake(_lock, portMAX_DELAY);
        LMLError err = LMLError::OK;
        if (_next >= _count) {
            err = LMLError::EMPTY;
        } else {
            out = _pkts[_next];
        }
        xSemaphoreGive(_lock);
        return err;
    }

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

    void reset() {
        xSemaphoreTake(_lock, portMAX_DELAY);
        _count = 0;
        _next  = 0;
        xSemaphoreGive(_lock);
    }

    uint8_t fragIndex() const {
        xSemaphoreTake(_lock, portMAX_DELAY);
        const uint8_t v = _next;
        xSemaphoreGive(_lock);
        return v;
    }

private:
    static constexpr uint8_t MAX_FRAGS = LML_MAX_MESSAGE / LML_MAX_PAYLOAD;

    LMLPacket _pkts[MAX_FRAGS];
    uint8_t   _count;
    uint8_t   _next;
    uint8_t   _seq;
    mutable SemaphoreHandle_t _lock;
};
