#include "LMLP2PAsync.h"
#include "LMLPacket.h"
#include <Arduino.h>

namespace LML {

// ── Statics ──────────────────────────────────────────────────────────────────

volatile bool P2PAsync::s_pktRx   = false;
P2PAsync     *P2PAsync::s_instance = nullptr;

void IRAM_ATTR P2PAsync::s_onPktRx() {
    s_pktRx = true;
}

void P2PAsync::s_timerCb(TimerHandle_t h) {
    P2PAsync *self = static_cast<P2PAsync *>(pvTimerGetTimerID(h));
    Serial.printf("[lml] TIMER fired state=%d\n", (int)self->_state);
    IEvent ev{};
    ev.type = IEvType::Timeout;
    xQueueSend(self->_evQueue, &ev, 0);
}

// ── Constructor / Destructor ─────────────────────────────────────────────────

P2PAsync::P2PAsync(SX1262 &radio, uint8_t irqPin, size_t rxQueueDepth)
    : _radio(radio), _irqPin(irqPin),
      _state(State::Standby), _ctrlSeq(0), _retries(0)
{
    rx_queue  = xQueueCreate(rxQueueDepth, sizeof(Event));
    _evQueue  = xQueueCreate(16, sizeof(IEvent));
    _timer    = xTimerCreate("lml", pdMS_TO_TICKS(500), pdFALSE, this, s_timerCb);
    s_instance = this;
    Serial.println("[lml] P2PAsync constructed");
}

P2PAsync::~P2PAsync() {
    xTimerDelete(_timer, portMAX_DELAY);
    vQueueDelete(_evQueue);
    vQueueDelete(rx_queue);
}

// ── Init ─────────────────────────────────────────────────────────────────────

void P2PAsync::init() {
    Serial.println("[lml] init: registering ISR + startReceive");
    _radio.setPacketReceivedAction(s_onPktRx);
    _radio.startReceive();
    Serial.println("[lml] init done");
}

// ── Timer helpers ─────────────────────────────────────────────────────────────

void P2PAsync::startTimer(uint32_t ms) {
    Serial.printf("[lml] startTimer %ums state=%d\n", ms, (int)_state);
    xTimerChangePeriod(_timer, pdMS_TO_TICKS(ms), 0);
    xTimerStart(_timer, 0);
}

void P2PAsync::stopTimer() {
    Serial.printf("[lml] stopTimer state=%d\n", (int)_state);
    xTimerStop(_timer, 0);
}

// ── Radio TX ─────────────────────────────────────────────────────────────────

LMLError P2PAsync::transmit(const LMLPacket &pkt) {
    size_t frameLen = 0;
    LMLError err = lml::encode(pkt, _txBuf, sizeof(_txBuf), &frameLen);
    if (err != LMLError::OK) {
        Serial.printf("[lml] transmit: encode FAILED err=%d\n", (int)err);
        return err;
    }
    Serial.printf("[lml] transmit type=%d seq=%u frag=%u/%u len=%u frameLen=%u\n",
                  (int)pkt.type, pkt.seq, pkt.frag, pkt.total, pkt.len, frameLen);
    int state = _radio.transmit(_txBuf, frameLen);
    // Clear any TxDone-triggered ISR flag before re-arming RX.
    // The SX1262 asserts DIO1 for TxDone; without this clear, poll() would
    // call readData() on the just-transmitted data (shared FIFO loopback).
    s_pktRx = false;
    Serial.printf("[lml] transmit done radio_state=%d → startReceive\n", state);
    _radio.startReceive();
    return (state == RADIOLIB_ERR_NONE) ? LMLError::OK : LMLError::RADIO_ERR;
}

// ── Helpers ───────────────────────────────────────────────────────────────────

void P2PAsync::toStandby() {
    Serial.printf("[lml] toStandby (was state=%d retries=%u)\n", (int)_state, _retries);
    stopTimer();
    _retries = 0;
    _state   = State::Standby;
    _reassembly.reset();
}

void P2PAsync::pushApp(const Event &ev) {
    if (ev.tag == Event::Tag::Message) {
        Serial.printf("[lml] pushApp: Message len=%u\n", ev.msg.len);
    } else {
        Serial.printf("[lml] pushApp: Error err=%d\n", (int)ev.err);
    }
    if (xQueueSend(rx_queue, &ev, 0) != pdTRUE) {
        Serial.println("[lml] pushApp: rx_queue FULL — event dropped");
    }
}

// ── poll() ────────────────────────────────────────────────────────────────────

void P2PAsync::poll() {
    // Check ISR flag; if set, read radio and post to internal event queue.
    if (s_pktRx) {
        s_pktRx = false;
        Serial.println("[lml] poll: ISR flag set — reading radio");
        size_t len = sizeof(_rxBuf);
        int st     = _radio.readData(_rxBuf, len);
        // Read metadata BEFORE startReceive() — mode switch may reset RSSI register.
        float  rssi   = _radio.getRSSI();
        size_t pktLen = _radio.getPacketLength();
        _radio.startReceive();
        Serial.printf("[lml] poll: readData st=%d pktLen=%u rssi=%.1f\n", st, pktLen, rssi);

        IEvent ev{};
        ev.rssi = rssi;

        if (st == RADIOLIB_ERR_NONE) {
            LMLError derr = lml::decode(_rxBuf, pktLen, ev.pkt);
            if (derr == LMLError::OK) {
                ev.type = IEvType::PacketRx;
                Serial.printf("[lml] poll: decoded type=%d seq=%u frag=%u/%u\n",
                              (int)ev.pkt.type, ev.pkt.seq, ev.pkt.frag, ev.pkt.total);
            } else {
                ev.type = IEvType::BadFrame;
                Serial.printf("[lml] poll: decode FAILED err=%d\n", (int)derr);
            }
        } else {
            ev.type = IEvType::BadFrame;
            Serial.printf("[lml] poll: readData FAILED st=%d\n", st);
        }
        xQueueSend(_evQueue, &ev, 0);
    }

    // Process one event (or yield 1ms if nothing pending).
    IEvent ev{};
    if (xQueueReceive(_evQueue, &ev, pdMS_TO_TICKS(1)) == pdTRUE) {
        Serial.printf("[lml] poll: dispatch evtype=%d state=%d\n",
                      (int)ev.type, (int)_state);
        dispatchEvent(ev);
    }

    // In Standby, kick off TX if data is queued.
    if (_state == State::Standby && !_tx.empty()) {
        Serial.println("[lml] poll: Standby+TX pending — sending RTS");
        LMLPacket rts = lml::makeCtrl(LMLPacketType::RTS, _ctrlSeq++);
        if (transmit(rts) == LMLError::OK) {
            _state   = State::PendingCTS;
            _retries = 0;
            startTimer(LML_ACK_TIMEOUT_MS);
            Serial.println("[lml] poll: → PendingCTS");
        } else {
            Serial.println("[lml] poll: RTS TX failed — staying Standby");
        }
    }
}

// ── State machine ─────────────────────────────────────────────────────────────

void P2PAsync::dispatchEvent(const IEvent &ev) {
    switch (_state) {
        case State::Standby:       onStandby(ev);       break;
        case State::PendingCTS:    onPendingCTS(ev);    break;
        case State::PendingCmdAck: onPendingCmdAck(ev); break;
        case State::PendingCmd:    onPendingCmd(ev);    break;
    }
}

void P2PAsync::onStandby(const IEvent &ev) {
    if (ev.type == IEvType::Timeout) {
        Serial.println("[lml] onStandby: Timeout ignored");
        return;
    }
    if (ev.type == IEvType::BadFrame) {
        Serial.println("[lml] onStandby: BadFrame ignored");
        return;
    }

    Serial.printf("[lml] onStandby: PacketRx type=%d\n", (int)ev.pkt.type);

    if (ev.pkt.type == LMLPacketType::RTS) {
        Serial.println("[lml] onStandby: got RTS → sending CTS");
        LMLPacket cts = lml::makeCtrl(LMLPacketType::CTS, ev.pkt.seq);
        if (transmit(cts) == LMLError::OK) {
            _reassembly.reset();
            _reassembly.setRSSI(ev.rssi);
            _retries = 0;
            _state   = State::PendingCmd;
            startTimer(LML_ACK_TIMEOUT_MS);
            Serial.println("[lml] onStandby: → PendingCmd");
        } else {
            Serial.println("[lml] onStandby: CTS TX failed — staying Standby");
        }
    } else {
        Serial.printf("[lml] onStandby: unexpected pkt type=%d — ignored\n",
                      (int)ev.pkt.type);
    }
}

void P2PAsync::onPendingCTS(const IEvent &ev) {
    if (ev.type == IEvType::Timeout || ev.type == IEvType::BadFrame) {
        Serial.printf("[lml] onPendingCTS: %s retries=%u/%u\n",
                      ev.type == IEvType::Timeout ? "Timeout" : "BadFrame",
                      _retries, LML_MAX_RETRIES);
        if (++_retries >= LML_MAX_RETRIES) {
            Serial.println("[lml] onPendingCTS: max retries — push Error");
            Event appEv{};
            appEv.tag = Event::Tag::Error;
            appEv.err = LMLError::TIMEOUT;
            _tx.reset();
            pushApp(appEv);
            toStandby();
        } else {
            Serial.println("[lml] onPendingCTS: retry RTS");
            LMLPacket rts = lml::makeCtrl(LMLPacketType::RTS, _ctrlSeq++);
            transmit(rts);
            startTimer(LML_ACK_TIMEOUT_MS);
        }
        return;
    }

    Serial.printf("[lml] onPendingCTS: PacketRx type=%d\n", (int)ev.pkt.type);

    if (ev.pkt.type == LMLPacketType::CTS) {
        Serial.println("[lml] onPendingCTS: CTS received — sending first DATA frag");
        stopTimer();
        _retries = 0;
        LMLPacket frag{};
        if (_tx.peek(frag) != LMLError::OK) {
            Serial.println("[lml] onPendingCTS: TX empty?! → toStandby");
            toStandby();
            return;
        }
        if (transmit(frag) == LMLError::OK) {
            _state = State::PendingCmdAck;
            startTimer(LML_ACK_TIMEOUT_MS);
            Serial.println("[lml] onPendingCTS: → PendingCmdAck");
        } else {
            Serial.println("[lml] onPendingCTS: DATA TX failed → toStandby");
            toStandby();
        }
    } else if (ev.pkt.type == LMLPacketType::NACK) {
        Serial.println("[lml] onPendingCTS: NACK → toStandby+Error");
        _tx.reset();
        Event appEv{}; appEv.tag = Event::Tag::Error; appEv.err = LMLError::TIMEOUT;
        pushApp(appEv);
        toStandby();
    } else if (ev.pkt.type == LMLPacketType::HALT) {
        Serial.println("[lml] onPendingCTS: HALT → toStandby+Error");
        _tx.reset();
        Event appEv{}; appEv.tag = Event::Tag::Error; appEv.err = LMLError::TIMEOUT;
        pushApp(appEv);
        toStandby();
    } else {
        Serial.printf("[lml] onPendingCTS: unexpected type=%d — ignored\n",
                      (int)ev.pkt.type);
    }
}

void P2PAsync::onPendingCmdAck(const IEvent &ev) {
    LMLPacket frag{};
    _tx.peek(frag);

    bool shouldRetry = (ev.type == IEvType::Timeout || ev.type == IEvType::BadFrame)
                    || (ev.type == IEvType::PacketRx
                        && ev.pkt.type == LMLPacketType::NACK);

    if (shouldRetry) {
        Serial.printf("[lml] onPendingCmdAck: %s retries=%u/%u\n",
                      ev.type == IEvType::Timeout ? "Timeout" :
                      ev.type == IEvType::BadFrame ? "BadFrame" : "NACK",
                      _retries, LML_MAX_RETRIES);
        if (++_retries >= LML_MAX_RETRIES) {
            Serial.println("[lml] onPendingCmdAck: max retries — HALT+Error");
            LMLPacket halt = lml::makeCtrl(LMLPacketType::HALT, _ctrlSeq++);
            transmit(halt);
            _tx.reset();
            Event appEv{}; appEv.tag = Event::Tag::Error; appEv.err = LMLError::TIMEOUT;
            pushApp(appEv);
            toStandby();
        } else {
            Serial.printf("[lml] onPendingCmdAck: retry DATA frag=%u\n", frag.frag);
            transmit(frag);
            startTimer(LML_ACK_TIMEOUT_MS);
        }
        return;
    }

    if (ev.type != IEvType::PacketRx) return;

    Serial.printf("[lml] onPendingCmdAck: PacketRx type=%d seq=%u frag=%u (want seq=%u frag=%u)\n",
                  (int)ev.pkt.type, ev.pkt.seq, ev.pkt.frag, frag.seq, frag.frag);

    if (ev.pkt.type == LMLPacketType::HALT) {
        Serial.println("[lml] onPendingCmdAck: HALT → toStandby+Error");
        _tx.reset();
        Event appEv{}; appEv.tag = Event::Tag::Error; appEv.err = LMLError::TIMEOUT;
        pushApp(appEv);
        toStandby();
        return;
    }

    if (ev.pkt.type == LMLPacketType::ACK
        && ev.pkt.seq  == frag.seq
        && ev.pkt.frag == frag.frag)
    {
        Serial.println("[lml] onPendingCmdAck: ACK matched — advance");
        stopTimer();
        _retries = 0;
        _tx.advance();

        if (_tx.empty()) {
            Serial.println("[lml] onPendingCmdAck: TX done → toStandby");
            toStandby();
        } else {
            LMLPacket next{};
            _tx.peek(next);
            Serial.printf("[lml] onPendingCmdAck: sending next frag=%u\n", next.frag);
            if (transmit(next) == LMLError::OK) {
                startTimer(LML_ACK_TIMEOUT_MS);
            } else {
                toStandby();
            }
        }
    } else {
        Serial.printf("[lml] onPendingCmdAck: ACK mismatch or wrong type — ignored\n");
    }
}

void P2PAsync::onPendingCmd(const IEvent &ev) {
    if (ev.type == IEvType::Timeout) {
        Serial.printf("[lml] onPendingCmd: Timeout retries=%u/%u\n",
                      _retries, LML_MAX_RETRIES);
        if (++_retries >= LML_MAX_RETRIES) {
            Serial.println("[lml] onPendingCmd: max retries — HALT+toStandby");
            LMLPacket halt = lml::makeCtrl(LMLPacketType::HALT, _ctrlSeq++);
            transmit(halt);
            toStandby();
        } else {
            Serial.println("[lml] onPendingCmd: NACK+retry");
            LMLPacket nack = lml::makeCtrl(LMLPacketType::NACK, _ctrlSeq++);
            transmit(nack);
            startTimer(LML_ACK_TIMEOUT_MS);
        }
        return;
    }

    if (ev.type == IEvType::BadFrame) {
        Serial.println("[lml] onPendingCmd: BadFrame → NACK");
        LMLPacket nack = lml::makeCtrl(LMLPacketType::NACK, _ctrlSeq++);
        transmit(nack);
        startTimer(LML_ACK_TIMEOUT_MS);
        return;
    }

    Serial.printf("[lml] onPendingCmd: PacketRx type=%d seq=%u frag=%u/%u\n",
                  (int)ev.pkt.type, ev.pkt.seq, ev.pkt.frag, ev.pkt.total);

    if (ev.pkt.type == LMLPacketType::HALT) {
        Serial.println("[lml] onPendingCmd: HALT → toStandby");
        toStandby();
        return;
    }

    if (ev.pkt.type != LMLPacketType::DATA) {
        Serial.printf("[lml] onPendingCmd: unexpected type=%d — ignored\n",
                      (int)ev.pkt.type);
        return;
    }

    _reassembly.setRSSI(ev.rssi);
    LMLMessage msg{};
    LMLError re = _reassembly.feed(ev.pkt, msg);
    Serial.printf("[lml] onPendingCmd: feed result=%d msg.len=%u\n", (int)re, msg.len);

    if (re != LMLError::OK) {
        Serial.println("[lml] onPendingCmd: bad fragment → NACK");
        LMLPacket nack = lml::makeCtrl(LMLPacketType::NACK, _ctrlSeq++);
        transmit(nack);
        startTimer(LML_ACK_TIMEOUT_MS);
        return;
    }

    // ACK the fragment
    LMLPacket ack = lml::makeCtrl(LMLPacketType::ACK, ev.pkt.seq);
    ack.frag = ev.pkt.frag;
    Serial.printf("[lml] onPendingCmd: sending ACK seq=%u frag=%u\n",
                  ack.seq, ack.frag);
    transmit(ack);
    _retries = 0;

    if (msg.len > 0) {
        Serial.printf("[lml] onPendingCmd: message complete len=%u → push+toStandby\n",
                      msg.len);
        Event appEv{};
        appEv.tag = Event::Tag::Message;
        appEv.msg = msg;
        pushApp(appEv);
        toStandby();
    } else {
        Serial.println("[lml] onPendingCmd: fragment accepted, waiting for more");
        startTimer(LML_ACK_TIMEOUT_MS);
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

LMLError P2PAsync::queue_tx(const uint8_t *data, size_t len) {
    Serial.printf("[lml] queue_tx len=%u\n", len);
    LMLError err = _tx.enqueue(data, len);
    Serial.printf("[lml] queue_tx result=%d\n", (int)err);
    return err;
}

} // namespace LML
