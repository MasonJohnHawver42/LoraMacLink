#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "MSRCTypes.h"

// Fixed-capacity ring buffer of MSRCMessage, safe to push from poll() and
// pop from any other task. Capacity is set at construction.
class MSRCRingBuffer {
public:
    explicit MSRCRingBuffer(size_t capacity)
        : _cap(capacity), _head(0), _tail(0), _count(0)
    {
        _buf  = new MSRCMessage[capacity];
        _lock = xSemaphoreCreateMutex();
    }

    ~MSRCRingBuffer() {
        delete[] _buf;
        vSemaphoreDelete(_lock);
    }

    // Push from poll() task. Returns QUEUE_FULL if no space.
    MSRCError push(const MSRCMessage &msg) {
        xSemaphoreTake(_lock, portMAX_DELAY);
        MSRCError err = MSRCError::OK;
        if (_count >= _cap) {
            err = MSRCError::QUEUE_FULL;
        } else {
            _buf[_tail] = msg;
            _tail = (_tail + 1) % _cap;
            ++_count;
        }
        xSemaphoreGive(_lock);
        return err;
    }

    // Pop from any task. Returns EMPTY if nothing waiting.
    MSRCError pop(MSRCMessage &out) {
        xSemaphoreTake(_lock, portMAX_DELAY);
        MSRCError err = MSRCError::OK;
        if (_count == 0) {
            err = MSRCError::EMPTY;
        } else {
            out   = _buf[_head];
            _head = (_head + 1) % _cap;
            --_count;
        }
        xSemaphoreGive(_lock);
        return err;
    }

    uint8_t available() const {
        xSemaphoreTake(_lock, portMAX_DELAY);
        const uint8_t n = static_cast<uint8_t>(_count);
        xSemaphoreGive(_lock);
        return n;
    }

private:
    size_t         _cap;
    size_t         _head;
    size_t         _tail;
    size_t         _count;
    MSRCMessage   *_buf;
    mutable SemaphoreHandle_t _lock;
};
