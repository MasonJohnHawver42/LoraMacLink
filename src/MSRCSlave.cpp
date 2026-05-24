#include "MSRCSlave.h"
#include "MSRCPacket.h"
#include "MSRCRingBuffer.h"
#include "MSRCTxQueue.h"
#include "MSRCReassembly.h"

struct MSRCSlave::Impl {
    MSRCRingBuffer rx;
    MSRCTxQueue    tx;
    MSRCReassembly reassembly;

    uint8_t  irqPin      = 0;

    volatile bool     rts         = false;
    volatile bool     cts         = false;
    volatile bool     connected   = false;
    volatile float    lastRssi    = 0.0f;
    volatile uint32_t lastHeardMs = 0;

    uint8_t ctrlSeq = 0;
    uint8_t txBuf[MSRC_HEADER_SIZE + MSRC_MAX_PAYLOAD + MSRC_CRC_SIZE];
    uint8_t rxBuf[MSRC_HEADER_SIZE + MSRC_MAX_PAYLOAD + MSRC_CRC_SIZE];

    SemaphoreHandle_t stateLock;

    explicit Impl(size_t rxDepth) : rx(rxDepth) {
        stateLock = xSemaphoreCreateMutex();
    }
    ~Impl() { vSemaphoreDelete(stateLock); }
};

MSRCSlave::MSRCSlave(SX1262 &radio, uint8_t irqPin, size_t rxDepth)
    : _radio(radio), _impl(new Impl(rxDepth)) { _impl->irqPin = irqPin; }

MSRCSlave::~MSRCSlave() { delete _impl; }

void MSRCSlave::init() {
    _impl->lastHeardMs = millis();
    _radio.startReceive(); // arm RX once; slaveSend() re-arms after every transmit
}

// ── Helpers ────────────────────────────────────────────────────────────────

static MSRCError slaveSend(SX1262 &radio, const MSRCPacket &pkt,
                           uint8_t *buf, size_t bufCap)
{
    size_t frameLen = 0;
    MSRCError err = msrc_internal::encode(pkt, buf, bufCap, &frameLen);
    if (err != MSRCError::OK) return err;
    int state = radio.transmit(buf, frameLen);
    radio.startReceive(); // re-arm RX immediately after every transmit
    return (state == RADIOLIB_ERR_NONE) ? MSRCError::OK : MSRCError::RADIO_ERR;
}

static MSRCError slaveRecv(SX1262 &radio, uint8_t irqPin, MSRCPacket &pkt,
                           uint8_t *buf, size_t bufCap,
                           uint32_t timeout_ms, float &rssiOut)
{
    // Do NOT call startReceive() here — radio must already be armed.
    // Calling startReceive() while a packet is in-flight aborts reception.
    const uint32_t deadline = millis() + timeout_ms;
    while (millis() < deadline) {
        if (digitalRead(irqPin)) {
            size_t len = bufCap;
            int state  = radio.readData(buf, len);
            // readData() puts the radio in standby — re-arm for next reception
            radio.startReceive();
            if (state != RADIOLIB_ERR_NONE) return MSRCError::RADIO_ERR;
            rssiOut = radio.getRSSI();
            len = radio.getPacketLength();
            return msrc_internal::decode(buf, len, pkt);
        }
        vTaskDelay(1);
    }
    return MSRCError::TIMEOUT;
}

void MSRCSlave::drainTx() {
    auto &im = *_impl;
    if (im.tx.empty()) return;

    MSRCPacket frag{};
    if (im.tx.peek(frag) != MSRCError::OK) return;

    for (uint8_t attempt = 0; attempt < MSRC_MAX_RETRIES; ++attempt) {
        slaveSend(_radio, frag, im.txBuf, sizeof(im.txBuf));

        MSRCPacket reply{};
        float rRSSI = 0.0f;
        MSRCError re = slaveRecv(_radio, im.irqPin, reply, im.rxBuf, sizeof(im.rxBuf),
                                 MSRC_ACK_TIMEOUT_MS, rRSSI);

        if (re == MSRCError::OK && reply.type == MSRCPacketType::ACK
            && reply.seq == frag.seq && reply.frag == frag.frag)
        {
            xSemaphoreTake(im.stateLock, portMAX_DELAY);
            im.lastRssi    = rRSSI;
            im.lastHeardMs = millis();
            xSemaphoreGive(im.stateLock);
            im.tx.advance();
            return;
        }
    }
}

// ── poll() ─────────────────────────────────────────────────────────────────

void MSRCSlave::poll(uint32_t timeout_ms) {
    auto &im = *_impl;

    {
        xSemaphoreTake(im.stateLock, portMAX_DELAY);
        im.connected = (millis() - im.lastHeardMs) < MSRC_LINK_TIMEOUT_MS;
        xSemaphoreGive(im.stateLock);
    }

    MSRCPacket incoming{};
    float rssi = 0.0f;
    MSRCError err = slaveRecv(_radio, im.irqPin, incoming, im.rxBuf, sizeof(im.rxBuf),
                              timeout_ms, rssi);

    if (err == MSRCError::TIMEOUT) {
        // Do NOT drain TX on timeout — the master may be mid-transmission and
        // switching the radio to TX would abort the in-flight packet.
        return;
    }

    if (err != MSRCError::OK) {
        Serial.printf("[msrc/s] RX error=%d — sending NACK\n", (int)err);
        MSRCPacket nack = msrc_internal::makeControl(MSRCPacketType::NACK, im.ctrlSeq++);
        slaveSend(_radio, nack, im.txBuf, sizeof(im.txBuf));
        return;
    }

    Serial.printf("[msrc/s] RX type=%d seq=%u rssi=%.1f\n",
                  (int)incoming.type, incoming.seq, rssi);

    {
        xSemaphoreTake(im.stateLock, portMAX_DELAY);
        im.lastHeardMs = millis();
        im.lastRssi    = rssi;
        xSemaphoreGive(im.stateLock);
    }

    switch (incoming.type) {
        case MSRCPacketType::RTS: {
            xSemaphoreTake(im.stateLock, portMAX_DELAY);
            im.rts = true;
            const bool ready = im.cts;
            xSemaphoreGive(im.stateLock);

            MSRCPacket resp = msrc_internal::makeControl(
                ready ? MSRCPacketType::CTS : MSRCPacketType::NACK,
                im.ctrlSeq++);
            slaveSend(_radio, resp, im.txBuf, sizeof(im.txBuf));
            break;
        }

        case MSRCPacketType::DATA: {
            // ACK first to minimise master retry window
            MSRCPacket ack = msrc_internal::makeControl(MSRCPacketType::ACK, incoming.seq);
            ack.frag = incoming.frag;
            slaveSend(_radio, ack, im.txBuf, sizeof(im.txBuf));

            im.reassembly.setRSSI(rssi);
            MSRCMessage msg{};
            if (im.reassembly.feed(incoming, msg) == MSRCError::OK && msg.len > 0) {
                im.rx.push(msg);
                xSemaphoreTake(im.stateLock, portMAX_DELAY);
                im.rts = false;
                xSemaphoreGive(im.stateLock);
            }
            break;
        }

        case MSRCPacketType::HB: {
            // If we have pending data, use the master's HB listen window to drain TX
            // instead of echoing HB — the master's DATA handler will ACK it.
            if (!im.tx.empty()) {
                drainTx();
            } else {
                MSRCPacket hb = msrc_internal::makeControl(MSRCPacketType::HB, im.ctrlSeq++);
                slaveSend(_radio, hb, im.txBuf, sizeof(im.txBuf));
            }
            break;
        }

        default:
            break;
    }
    // drainTx is intentionally NOT called here — only drain during the HB window
    // where the master is known to be listening.
}

// ── Public API ─────────────────────────────────────────────────────────────

bool MSRCSlave::getRTS() const {
    xSemaphoreTake(_impl->stateLock, portMAX_DELAY);
    const bool v = _impl->rts;
    xSemaphoreGive(_impl->stateLock);
    return v;
}

void MSRCSlave::setCTS(bool ready) {
    xSemaphoreTake(_impl->stateLock, portMAX_DELAY);
    _impl->cts = ready;
    xSemaphoreGive(_impl->stateLock);
}

MSRCError MSRCSlave::read(MSRCMessage &out) {
    return _impl->rx.pop(out);
}

uint8_t MSRCSlave::available() const {
    return _impl->rx.available();
}

MSRCError MSRCSlave::send(const uint8_t *data, size_t len) {
    return _impl->tx.enqueue(data, len);
}

bool MSRCSlave::isConnected() const {
    xSemaphoreTake(_impl->stateLock, portMAX_DELAY);
    const bool v = _impl->connected;
    xSemaphoreGive(_impl->stateLock);
    return v;
}

float MSRCSlave::lastRSSI() const {
    xSemaphoreTake(_impl->stateLock, portMAX_DELAY);
    const float v = _impl->lastRssi;
    xSemaphoreGive(_impl->stateLock);
    return v;
}
