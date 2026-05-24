#pragma once
#include <RadioLib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "MSRCTypes.h"

class MSRCSlave {
public:
    explicit MSRCSlave(SX1262 &radio, uint8_t irqPin, size_t rxDepth = 4);
    ~MSRCSlave();

    void init();
    void poll(uint32_t timeout_ms = 100);

    // Flow control
    bool getRTS() const;
    void setCTS(bool ready);

    // RX ring buffer — thread-safe
    MSRCError read(MSRCMessage &out);
    uint8_t   available() const;

    // TX — thread-safe
    MSRCError send(const uint8_t *data, size_t len);

    // Link state
    bool  isConnected() const;
    float lastRSSI()    const;

private:
    SX1262 &_radio;

    struct Impl;
    Impl *_impl;

    void drainTx();
};
