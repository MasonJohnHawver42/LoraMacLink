#include "MSRCMaster.h"
#include "MSRCPacket.h"
#include "MSRCRingBuffer.h"
#include "MSRCTxQueue.h"
#include "MSRCReassembly.h"

struct MSRCMaster::Impl {
    MSRCRingBuffer rx;
    MSRCTxQueue    tx;
    MSRCReassembly reassembly;

    uint8_t  irqPin      = 0;

    volatile bool     rts         = false;
    volatile bool     cts         = false;
    volatile bool     connected   = false;
    volatile float    lastRssi    = 0.0f;
    volatile uint32_t lastHeardMs = 0;

    uint8_t  ctrlSeq     = 0;
    uint8_t  txBuf[MSRC_HEADER_SIZE + MSRC_MAX_PAYLOAD + MSRC_CRC_SIZE];
    uint8_t  rxBuf[MSRC_HEADER_SIZE + MSRC_MAX_PAYLOAD + MSRC_CRC_SIZE];

    SemaphoreHandle_t stateLock;   // protects rts, cts, connected, lastRssi

    explicit Impl(size_t rxDepth) : rx(rxDepth) {
        stateLock = xSemaphoreCreateMutex();
    }
    ~Impl() { vSemaphoreDelete(stateLock); }
};

MSRCMaster::MSRCMaster(SX1262 &radio, uint8_t irqPin, size_t rxDepth)
    : _radio(radio), _impl(new Impl(rxDepth)) { _impl->irqPin = irqPin; }

MSRCMaster::~MSRCMaster() { delete _impl; }

void MSRCMaster::init() {
    _impl->lastHeardMs = millis();
    _radio.startReceive(); // arm RX once; sendPacket() re-arms after every transmit
}

// ── Helpers ────────────────────────────────────────────────────────────────

static MSRCError sendPacket(SX1262 &radio, const MSRCPacket &pkt,
                            uint8_t *buf, size_t bufCap)
{
    size_t frameLen = 0;
    MSRCError err = msrc_internal::encode(pkt, buf, bufCap, &frameLen);
    if (err != MSRCError::OK) return err;

    int state = radio.transmit(buf, frameLen);
    radio.startReceive(); // re-arm RX immediately after every transmit
    return (state == RADIOLIB_ERR_NONE) ? MSRCError::OK : MSRCError::RADIO_ERR;
}

static MSRCError recvPacket(SX1262 &radio, uint8_t irqPin, MSRCPacket &pkt,
                            uint8_t *buf, size_t bufCap,
                            uint32_t timeout_ms, float &rssiOut)
{
    // Do NOT call startReceive() here — radio is already armed by sendPacket()
    // or by the previous readData() re-arm. Calling it again mid-flight aborts reception.
    const uint32_t deadline = millis() + timeout_ms;
    while (millis() < deadline) {
        if (digitalRead(irqPin)) {
            size_t len = bufCap;
            int state  = radio.readData(buf, len);
            // readData() puts the radio in standby — re-arm for next reception
            radio.startReceive();
            if (state != RADIOLIB_ERR_NONE) return MSRCError::RADIO_ERR;
            rssiOut = radio.getRSSI();
            len     = radio.getPacketLength();
            return msrc_internal::decode(buf, len, pkt);
        }
        vTaskDelay(1);
    }
    return MSRCError::TIMEOUT;
}

// ── poll() state machine ───────────────────────────────────────────────────

void MSRCMaster::poll(uint32_t timeout_ms) {
    auto &im = *_impl;

    // Update link-alive flag
    {
        xSemaphoreTake(im.stateLock, portMAX_DELAY);
        im.connected = (millis() - im.lastHeardMs) < MSRC_LINK_TIMEOUT_MS;
        xSemaphoreGive(im.stateLock);
    }

    // ── RTS / CTS handshake ──────────────────────────────────────────────
    xSemaphoreTake(im.stateLock, portMAX_DELAY);
    const bool wantRTS = im.rts;
    const bool hasCTS  = im.cts;
    xSemaphoreGive(im.stateLock);

    if (wantRTS && !hasCTS) {
        MSRCPacket rtsPacket = msrc_internal::makeControl(MSRCPacketType::RTS, im.ctrlSeq++);
        Serial.printf("[msrc/m] TX RTS seq=%u\n", rtsPacket.seq);
        MSRCError txErr = sendPacket(_radio, rtsPacket, im.txBuf, sizeof(im.txBuf));
        Serial.printf("[msrc/m] TX RTS result=%d\n", (int)txErr);

        MSRCPacket reply{};
        float rssi = 0.0f;
        MSRCError err = recvPacket(_radio, im.irqPin, reply, im.rxBuf, sizeof(im.rxBuf),
                                   MSRC_ACK_TIMEOUT_MS, rssi);
        Serial.printf("[msrc/m] RX after RTS result=%d type=%d\n", (int)err,
                      err == MSRCError::OK ? (int)reply.type : -1);

        if (err == MSRCError::OK) {
            xSemaphoreTake(im.stateLock, portMAX_DELAY);
            im.lastHeardMs = millis();
            im.lastRssi    = rssi;
            if (reply.type == MSRCPacketType::CTS)  im.cts = true;
            if (reply.type == MSRCPacketType::NACK) im.cts = false;
            xSemaphoreGive(im.stateLock);
        }
        return;
    }

    // ── TX: drain staged fragments if CTS granted ────────────────────────
    if (hasCTS && !im.tx.empty()) {
        MSRCPacket frag{};
        if (im.tx.peek(frag) != MSRCError::OK) return;

        for (uint8_t attempt = 0; attempt < MSRC_MAX_RETRIES; ++attempt) {
            sendPacket(_radio, frag, im.txBuf, sizeof(im.txBuf));

            MSRCPacket reply{};
            float rssi = 0.0f;
            MSRCError err = recvPacket(_radio, im.irqPin, reply, im.rxBuf, sizeof(im.rxBuf),
                                       MSRC_ACK_TIMEOUT_MS, rssi);

            if (err == MSRCError::OK && reply.type == MSRCPacketType::ACK
                && reply.seq == frag.seq && reply.frag == frag.frag) {
                xSemaphoreTake(im.stateLock, portMAX_DELAY);
                im.lastHeardMs = millis();
                im.lastRssi    = rssi;
                xSemaphoreGive(im.stateLock);

                im.tx.advance();

                // All fragments ACKed — clear RTS and CTS
                if (im.tx.empty()) {
                    xSemaphoreTake(im.stateLock, portMAX_DELAY);
                    im.rts = false;
                    im.cts = false;
                    xSemaphoreGive(im.stateLock);
                }
                return;
            }
        }
        // Retries exhausted — leave CTS set, caller can retry or give up
        return;
    }

    // ── Heartbeat + opportunistic RX ─────────────────────────────────────
    {
        static uint32_t lastHbMs = 0;
        if (millis() - lastHbMs >= MSRC_HEARTBEAT_INTERVAL_MS) {
            MSRCPacket hb = msrc_internal::makeControl(MSRCPacketType::HB, im.ctrlSeq++);
            sendPacket(_radio, hb, im.txBuf, sizeof(im.txBuf));
            lastHbMs = millis();
        }
    }

    // Listen for inbound DATA or HB from slave
    MSRCPacket incoming{};
    float rssi = 0.0f;
    MSRCError err = recvPacket(_radio, im.irqPin, incoming, im.rxBuf, sizeof(im.rxBuf),
                               timeout_ms, rssi);

    if (err == MSRCError::TIMEOUT) {
        return; // silence — don't NACK, slave may be mid-transmission
    }
    if (err != MSRCError::OK) {
        // Corrupt frame (bad magic, CRC, length) — tell slave to retry
        MSRCPacket nack = msrc_internal::makeControl(MSRCPacketType::NACK, im.ctrlSeq++);
        sendPacket(_radio, nack, im.txBuf, sizeof(im.txBuf));
        return;
    }

    xSemaphoreTake(im.stateLock, portMAX_DELAY);
    im.lastHeardMs = millis();
    im.lastRssi    = rssi;
    xSemaphoreGive(im.stateLock);

    if (incoming.type == MSRCPacketType::DATA) {
        im.reassembly.setRSSI(rssi);
        MSRCMessage msg{};
        MSRCError re = im.reassembly.feed(incoming, msg);

        if (re != MSRCError::OK) {
            // Fragment was structurally valid but doesn't fit the current sequence
            MSRCPacket nack = msrc_internal::makeControl(MSRCPacketType::NACK, im.ctrlSeq++);
            sendPacket(_radio, nack, im.txBuf, sizeof(im.txBuf));
            return;
        }

        if (msg.len > 0) {
            im.rx.push(msg);
        }

        MSRCPacket ack = msrc_internal::makeControl(MSRCPacketType::ACK, incoming.seq);
        ack.frag = incoming.frag;
        sendPacket(_radio, ack, im.txBuf, sizeof(im.txBuf));
    }
}

// ── Public API ─────────────────────────────────────────────────────────────

void MSRCMaster::setRTS(bool ready) {
    xSemaphoreTake(_impl->stateLock, portMAX_DELAY);
    _impl->rts = ready;
    if (!ready) _impl->cts = false;
    xSemaphoreGive(_impl->stateLock);
}

bool MSRCMaster::getCTS() const {
    xSemaphoreTake(_impl->stateLock, portMAX_DELAY);
    const bool v = _impl->cts;
    xSemaphoreGive(_impl->stateLock);
    return v;
}

bool MSRCMaster::waitCTS(uint32_t timeout_ms) {
    const uint32_t deadline = millis() + timeout_ms;
    while (millis() < deadline) {
        if (getCTS()) return true;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return false;
}

MSRCError MSRCMaster::send(const uint8_t *data, size_t len) {
    if (!getCTS()) return MSRCError::NO_CTS;
    return _impl->tx.enqueue(data, len);
}

MSRCError MSRCMaster::read(MSRCMessage &out) {
    return _impl->rx.pop(out);
}

uint8_t MSRCMaster::available() const {
    return _impl->rx.available();
}

bool MSRCMaster::isConnected() const {
    xSemaphoreTake(_impl->stateLock, portMAX_DELAY);
    const bool v = _impl->connected;
    xSemaphoreGive(_impl->stateLock);
    return v;
}

float MSRCMaster::lastRSSI() const {
    xSemaphoreTake(_impl->stateLock, portMAX_DELAY);
    const float v = _impl->lastRssi;
    xSemaphoreGive(_impl->stateLock);
    return v;
}
