# MSRC — Master Slave Protocol
## Library Design

Platform IO library built ontop of RadioLib.

### Philosophy

The library owns the **protocol**, not the **radio** and not the **tasks**.

The caller:
- Creates and configures the RadioLib module
- Creates FreeRTOS tasks, chooses cores, stack sizes, and priorities
- Decides when and how often to drive the protocol via `poll()`

The library:
- Hides packet framing, CRC, sequence numbers, ACK/NACK, and retries
- Hides fragmentation and reassembly (a message may span multiple packets)
- Manages RTS/CTS state on behalf of both sides
- Provides a thread-safe ring buffer of assembled messages to the caller

---

### Concepts

**Packet** — a single LoRa transmission. Internal to the library; the caller never sees packets.

**Message** — what the caller sends and receives. May be fragmented across multiple packets transparently. This is the unit the caller works with.

**RTS / CTS** — protocol-level flow control exchanged over the air.
- Master calls `setRTS(true)` to signal it has a message to send.
- Slave calls `setCTS(true/false)` to signal readiness.
- `poll()` on the master transmits the RTS exchange and updates `getCTS()` accordingly.
- `poll()` on the slave listens for RTS and responds based on the current CTS state.

---

### Types

```cpp
// MSRCTypes.h

#pragma once
#include <stdint.h>

constexpr size_t MSRC_MAX_MESSAGE = 512;   // max assembled message payload

struct MSRCMessage {
    uint8_t  data[MSRC_MAX_MESSAGE];
    size_t   len;
    float    rssi;          // RSSI of the final received packet
    uint32_t timestamp_ms;  // millis() when the message was fully assembled
};
```

---

### MSRCMaster

```cpp
// MSRCMaster.h

#pragma once
#include <RadioLib.h>
#include "MSRCTypes.h"

class MSRCMaster {
public:
    // radio     — fully initialised RadioLib module; caller owns it
    // rxDepth   — ring buffer capacity (number of messages)
    explicit MSRCMaster(SX1262 &radio, size_t rxDepth = 8);

    // Call once before poll(). Does not touch the radio or create tasks.
    void init();

    // Drive the protocol state machine. Call from the caller's radio task.
    // Blocks for up to timeout_ms waiting for an incoming packet, then
    // services the TX queue. Safe to call with timeout_ms = 0 for non-blocking.
    void poll(uint32_t timeout_ms = 100);

    // --- Flow control ---

    // Signal intent to send. poll() will transmit RTS packets until CTS is
    // received or the attempt times out. Automatically cleared after send().
    void setRTS(bool ready);

    // True once the slave has responded CTS to our RTS.
    bool getCTS() const;

    // Convenience: blocks the calling task (not poll) until CTS or timeout.
    // Returns true if CTS was granted.
    bool waitCTS(uint32_t timeout_ms = 2000);

    // --- TX ---

    // Enqueue a message. Fragmented internally if len > one packet's payload.
    // Returns false if the TX queue is full or CTS has not been granted.
    // Thread-safe — may be called from a different task than poll().
    bool send(const uint8_t *data, size_t len);

    // --- RX ring buffer ---

    // Pop the oldest assembled message. Returns false if the buffer is empty.
    // Thread-safe — may be called from a different task than poll().
    bool    read(MSRCMessage &out);
    uint8_t available() const;   // number of messages waiting in the buffer

    // --- Link state ---
    bool  isConnected() const;   // slave replied within the last heartbeat window
    float lastRSSI()    const;
};
```

---

### MSRCSlave

```cpp
// MSRCSlave.h

#pragma once
#include <RadioLib.h>
#include "MSRCTypes.h"

class MSRCSlave {
public:
    // radio    — fully initialised RadioLib module; caller owns it
    // rxDepth  — pending message queue depth; 1 is sufficient for most uses
    explicit MSRCSlave(SX1262 &radio, size_t rxDepth = 4);

    // Call once before poll(). Does not touch the radio or create tasks.
    void init();

    // Drive the protocol state machine. Call from the caller's radio task.
    // Blocks for up to timeout_ms waiting for an incoming packet.
    void poll(uint32_t timeout_ms = 100);

    // --- Flow control ---

    // True if the master has asserted RTS (it has a message to send us).
    bool getRTS() const;

    // Signal readiness to receive. When false, poll() responds NACK to
    // any RTS from the master — the caller can hold the master off while
    // it is busy processing a previous message.
    void setCTS(bool ready);

    // --- RX pending messages ---

    // Pop the oldest assembled message from the master. Returns false if empty.
    // Thread-safe — may be called from a different task than poll().
    bool    read(MSRCMessage &out);
    uint8_t available() const;   // number of pending messages from master

    // --- TX ---

    // Send a response / status message back to master.
    // Thread-safe — may be called from a different task than poll().
    bool send(const uint8_t *data, size_t len);

    // --- Link state ---
    bool  isConnected() const;
    float lastRSSI()    const;
};
```

---

### Threading model

`poll()` is **not** thread-safe with itself — it must be called from exactly one task that owns the radio. Everything else (`send`, `read`, `available`, `setRTS`, `setCTS`, `getCTS`, `getRTS`) is thread-safe and may be called from any task.

The caller decides the task structure. A typical two-core split:

```
Core 0                          Core 1
──────────────────────────      ──────────────────────────
protocol task                   app task
  while(true) msrc.poll(100);      msrc.setRTS / send / read
                                display task
                                  draws from its own state
```

---

### Usage — Master

```cpp
// main_master.cpp

SX1262   radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
MSRCMaster msrc(radio);

void protocolTask(void *) {
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
    radio.begin(915.0, 125.0, 9, 7, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14, 8, 1.8);
    radio.setDio2AsRfSwitch(true);
    msrc.init();

    while (true) {
        msrc.poll(100);
    }
}

void appTask(void *) {
    msrc.setRTS(true);

    if (!msrc.waitCTS(2000)) {
        // slave not ready — handle however the application requires
        vTaskDelay(pdMS_TO_TICKS(500));
        return;
    }

    uint8_t cmd[] = { 0x01, 0x02 };
    msrc.send(cmd, sizeof(cmd));   // setRTS auto-cleared after send enqueued

    MSRCMessage msg;
    while (msrc.read(msg)) {
        // drain ring buffer — update display, take action, etc.
    }
}

void setup() {
    xTaskCreatePinnedToCore(protocolTask, "msrc_m", 8192, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(appTask,      "app_m", 4096, NULL, 2, NULL, 1);
}
```

---

### Usage — Slave

```cpp
// main_slave.cpp

SX1262  radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
MSRCSlave msrc(radio);

void protocolTask(void *) {
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
    radio.begin(915.0, 125.0, 9, 7, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14, 8, 1.8);
    radio.setDio2AsRfSwitch(true);
    msrc.init();
    msrc.setCTS(true);

    while (true) {
        msrc.poll(100);
    }
}

void appTask(void *) {
    MSRCMessage msg;
    if (msrc.read(msg)) {
        // suspend CTS while processing so master is held off
        msrc.setCTS(false);

        // ... process message, do work ...

        uint8_t status[] = { 0xFF };
        msrc.send(status, sizeof(status));

        msrc.setCTS(true);
    }
}

void setup() {
    xTaskCreatePinnedToCore(protocolTask, "msrc_s", 8192, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(appTask,      "app_s", 4096, NULL, 2, NULL, 1);
}
```

---

### Open questions

1. **RTS timeout** — should `setRTS(true)` carry a deadline after which poll() stops
   attempting the handshake and surfaces an error?
2. **TX queue depth** — how many outgoing messages should queue before `send()` returns
   false? Currently implicit; could be a constructor parameter.
3. **On ring buffer overflow** — drop oldest (newest-wins) or return false from the
   poll-side enqueue and surface a counter the caller can inspect?
