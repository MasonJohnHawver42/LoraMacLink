#pragma once
#include <RadioLib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "MSRCTypes.h"

class MSRCMaster {
public:
    explicit MSRCMaster(SX1262 &radio, uint8_t irqPin, size_t rxDepth = 8);
    ~MSRCMaster();

    void init();
    void poll(uint32_t timeout_ms = 100);

    // Flow control
    void setRTS(bool ready);
    bool getCTS() const;
    bool waitCTS(uint32_t timeout_ms = 2000);

    // TX — thread-safe
    MSRCError send(const uint8_t *data, size_t len);

    // RX ring buffer — thread-safe
    MSRCError read(MSRCMessage &out);
    uint8_t   available() const;

    // Link state
    bool  isConnected() const;
    float lastRSSI()    const;

private:
    SX1262 &_radio;

    // Forward-declared implementation objects
    struct Impl;
    Impl *_impl;
};
