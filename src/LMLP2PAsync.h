#pragma once
#include <RadioLib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/timers.h>
#include "LMLTypes.h"
#include "LMLTxQueue.h"
#include "LMLReassembly.h"

namespace LML {

// Outcome pushed to rx_queue for the app task.
// tag == Message: data arrived (read msg).
// tag == Error:   send failed (read err).
struct Event {
    enum class Tag : uint8_t { Message, Error } tag;
    union {
        LMLMessage msg;
        LMLError   err;
    };
    Event() : tag(Tag::Error), err(LMLError::OK) {}
};

// ISR-driven, FreeRTOS-queue-backed symmetric MAC layer.
// Either node may initiate: whoever calls queue_tx() first becomes Initiator
// for that exchange. The other node becomes Responder automatically.
//
// Call init() once, then poll() in a dedicated FreeRTOS task tight loop.
// App task reads from rx_queue and calls queue_tx() to send.
class P2PAsync {
public:
    explicit P2PAsync(SX1262 &radio, uint8_t irqPin,
                      size_t rxQueueDepth = 8);
    ~P2PAsync();

    // Arm radio and register ISR. Call before poll().
    void init();

    // Drive the state machine. Returns after processing one event (or 1ms idle).
    void poll();

    // Thread-safe: enqueue a message for transmission.
    // Returns QUEUE_FULL if an exchange is already in progress, MSG_TOO_BIG if
    // len > LML_MAX_MESSAGE.
    LMLError queue_tx(const uint8_t *data, size_t len);

    // Thread-safe: drop any not-yet-sent message from the TX queue so the next
    // queue_tx() is accepted. Does NOT abort an exchange already on the wire —
    // if one is mid-flight the state machine resolves it to Standby on its own.
    void clear_tx();

    // App task reads from here.
    QueueHandle_t rx_queue;

private:
    // ── Internal state machine ──────────────────────────────────────────────

    enum class State : uint8_t {
        Standby,
        PendingCTS,     // [Initiator] sent RTS, waiting for CTS
        PendingCmdAck,  // [Initiator] sent DATA[n], waiting for ACK
        PendingCmd,     // [Responder] sent CTS, waiting for DATA[n]
    };

    // Events flowing through _evQueue to drive the state machine.
    enum class IEvType : uint8_t { PacketRx, BadFrame, Timeout };
    struct IEvent {
        IEvType   type;
        LMLPacket pkt;  // valid when type == PacketRx
        float     rssi;
    };

    // ── Members ─────────────────────────────────────────────────────────────

    SX1262       &_radio;
    uint8_t       _irqPin;
    State         _state;
    uint8_t       _ctrlSeq;
    uint8_t       _retries;

    LMLTxQueue    _tx;
    LMLReassembly _reassembly;

    QueueHandle_t _evQueue;   // internal: state machine events
    TimerHandle_t _timer;     // one-shot FreeRTOS timer for per-state deadline

    uint8_t _txBuf[LML_HEADER_SIZE + LML_MAX_PAYLOAD + LML_CRC_SIZE];
    uint8_t _rxBuf[LML_HEADER_SIZE + LML_MAX_PAYLOAD + LML_CRC_SIZE];

    // ── ISR + timer statics ─────────────────────────────────────────────────

    static volatile bool s_pktRx;
    static P2PAsync     *s_instance; // single-instance assumption
    static void IRAM_ATTR s_onPktRx();
    static void           s_timerCb(TimerHandle_t h);

    // ── Private helpers ─────────────────────────────────────────────────────

    void     startTimer(uint32_t ms);
    void     stopTimer();
    LMLError transmit(const LMLPacket &pkt);
    void     toStandby();
    void     pushApp(const Event &ev);

    void dispatchEvent(const IEvent &ev);
    void onStandby(const IEvent &ev);
    void onPendingCTS(const IEvent &ev);
    void onPendingCmdAck(const IEvent &ev);
    void onPendingCmd(const IEvent &ev);
};

} // namespace LML
