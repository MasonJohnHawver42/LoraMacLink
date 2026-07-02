# LML — LoRa MAC Link

A PlatformIO library implementing a reliable, symmetric point-to-point MAC over
LoRa, built on top of [RadioLib](https://github.com/jgromes/RadioLib). Targets
the Heltec WiFi LoRa 32 V3 (SX1262).

---

## Philosophy

The library owns the **protocol**, not the **radio** and not the **tasks**.

The caller:
- Creates and configures the RadioLib module
- Creates FreeRTOS tasks, chooses cores, stack sizes, and priorities
- Decides when and how often to drive the protocol by calling `poll()`

The library:
- Hides packet framing, CRC, sequence numbers, ACK/NACK, retries, and HALT
- Hides fragmentation and reassembly (a message may span multiple packets)
- Drives an internal state machine through an ISR + FreeRTOS timer + event queue
- Surfaces results to the app task as `LML::Event` values on a FreeRTOS queue

---

## Concepts

**Packet** — a single LoRa transmission. Internal to the library; the caller
never sees raw packets.

**Message** — what the caller sends and receives, up to `LML_MAX_MESSAGE`
(512 bytes). Transparently fragmented across multiple packets when needed.

**Symmetric peers** — there is no fixed master or slave. Either node may
initiate an exchange; whoever calls `queue_tx()` first becomes the Initiator
for that exchange and the other side becomes the Responder. After the exchange
both sides return to `Standby`.

**RTS / CTS** — protocol-level handshake exchanged over the air. The Initiator
sends RTS; the Responder replies CTS once it has accepted the exchange. The
library manages the handshake automatically — the app does not call `setRTS` /
`setCTS`.

**Event-driven app interface** — the app does not poll the radio. It blocks on
`rx_queue` and receives one `LML::Event` per outcome (a delivered message, or
an error that aborted an outgoing send).

---

## Error codes

Every fallible API returns `LMLError`:

| Code | Meaning |
|------|---------|
| `OK` | Success |
| `QUEUE_FULL` | TX queue rejected — an exchange is already in progress |
| `BAD_MAGIC` | Received frame did not start with `0xAC` |
| `BAD_CRC` | CRC16 mismatch — frame corrupted |
| `BAD_FRAG` | Fragment index out of range or mismatched total |
| `BAD_LENGTH` | Declared payload length exceeds buffer capacity |
| `TIMEOUT` | ACK or CTS not received within the retry deadline |
| `RADIO_ERR` | RadioLib returned a non-zero status code |
| `MSG_TOO_BIG` | Message exceeds `LML_MAX_MESSAGE` (512 bytes) |
| `EMPTY` | Internal queue read on an empty buffer |

> The library detects corruption via CRC16 and retries. It does not repair
> data. If your application needs forward error correction, encode the payload
> before `queue_tx()` and decode it after the message arrives on `rx_queue`.

---

## Packet format

```
Byte   Field    Description
────   ─────    ───────────────────────────────────────────────
 0     magic    Always 0xAC
 1     type     LMLPacketType (RTS, CTS, NACK, DATA, ACK, HALT)
 2     seq      Sequence number (wraps 0–255)
 3     frag     Fragment index (0-based)
 4     total    Total fragments in this message (1 = unfragmented)
 5     len      Payload bytes that follow (0–64)
 6..N  payload  Application data
 N+1   crc_hi   CRC16-CCITT high byte  } over bytes 0..N
 N+2   crc_lo   CRC16-CCITT low byte   }
```

Control-packet types (RTS, CTS, NACK, ACK, HALT) carry no payload. NACK is sent
for any frame the receiver cannot accept right now: bad CRC, bad fragment, or
state-machine mismatch. HALT is sent when retries are exhausted and tears the
exchange back to `Standby` on both sides.

---

## State machine

```
                          ┌────────────┐
              queue_tx →  │  Standby   │  ← any RX outside an exchange
                          └─────┬──────┘
                                │ send RTS
                                ▼
                          ┌────────────┐
                          │ PendingCTS │  Initiator: waiting for CTS
                          └─────┬──────┘
                                │ CTS received → send first DATA frag
                                ▼
                          ┌────────────────┐
                          │ PendingCmdAck  │  Initiator: waiting for ACK
                          └─────┬──────────┘
                                │ ACK matched → next frag, or done
                                ▼
                            (Standby)


  RX of RTS in Standby:
      Responder sends CTS, transitions to PendingCmd
                          ┌────────────┐
                          │ PendingCmd │  Responder: receiving DATA frags
                          └─────┬──────┘
                                │ last frag ACKed → push message
                                ▼
                            (Standby)
```

Each non-Standby state arms a `LML_ACK_TIMEOUT_MS` (500 ms) one-shot timer.
On timeout or NACK the side retries up to `LML_MAX_RETRIES` (3); on the final
miss it sends HALT, surfaces `TIMEOUT` on `rx_queue`, and returns to Standby.

---

## Threading model

`poll()` is **not** thread-safe with itself — call it from exactly one task that
owns the radio. `queue_tx()` is thread-safe and may be called from any task.

The app task does not call `poll()` — it blocks on `rx_queue` and processes
`LML::Event` values.

A typical two-core split on the ESP32-S3:

```
Core 0                                Core 1
──────────────────────────            ──────────────────────────
protocol task                         app task
  while(true) mac.poll();               xQueueReceive(mac.rx_queue, ...)
                                        mac.queue_tx(...)
```

---

## API

```cpp
#include <LML.h>

namespace LML {

struct Event {
    enum class Tag : uint8_t { Message, Error } tag;
    union {
        LMLMessage msg;   // tag == Message
        LMLError   err;   // tag == Error
    };
};

class P2PAsync {
public:
    // radio        — fully-constructed RadioLib SX1262; caller owns it
    // irqPin       — DIO1 pin (used to wire the RX-done ISR)
    // rxQueueDepth — capacity of the app-facing rx_queue (events)
    P2PAsync(SX1262 &radio, uint8_t irqPin, size_t rxQueueDepth = 8);

    void init();        // register ISR + startReceive
    void poll();        // drive the state machine; call in a tight loop

    LMLError queue_tx(const uint8_t *data, size_t len);

    QueueHandle_t rx_queue;   // app reads LML::Event from here
};

} // namespace LML

struct LMLMessage {
    uint8_t  data[LML_MAX_MESSAGE];   // 512
    size_t   len;
    float    rssi;                    // RSSI of the final received packet
    uint32_t timestamp_ms;            // millis() when fully assembled
};
```

---

## Usage

Both peers run the same protocol; the only thing that differs is who calls
`queue_tx()` first. See `RemoteDetonator/src/main_master.cpp` and
`main_slave.cpp` for a complete request/response example.

```cpp
#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <LML.h>

#define LORA_NSS  8
#define LORA_DIO1 14
#define LORA_RST  12
#define LORA_BUSY 13
#define LORA_SCK  9
#define LORA_MISO 11
#define LORA_MOSI 10

SX1262        radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
LML::P2PAsync mac(radio, LORA_DIO1);

void protocolTask(void *) {
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
    radio.begin(915.0, 125.0, 9, 7,
                RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14, 8, 1.8);
    radio.setDio2AsRfSwitch(true);
    mac.init();
    while (true) mac.poll();
}

void appTask(void *) {
    uint8_t payload[] = { 0x01, 0x02 };
    mac.queue_tx(payload, sizeof(payload));

    while (true) {
        LML::Event ev{};
        xQueueReceive(mac.rx_queue, &ev, portMAX_DELAY);

        if (ev.tag == LML::Event::Tag::Error) {
            // TX aborted (TIMEOUT after retries, etc.) — decide whether to retry
            continue;
        }

        // ev.msg is a complete LMLMessage from the peer
    }
}

void setup() {
    Serial.begin(115200);
    xTaskCreatePinnedToCore(protocolTask, "lml", 8192, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(appTask,      "app", 4096, NULL, 2, NULL, 1);
}

void loop() { vTaskDelay(portMAX_DELAY); }
```

---

## Heltec V3 pin mapping

| Signal | GPIO |
|--------|------|
| NSS (CS) | 8 |
| DIO1 | 14 |
| RST | 12 |
| BUSY | 13 |
| SCK | 9 |
| MISO | 11 |
| MOSI | 10 |

---

## Project structure

```
LoraMacLink/
├── platformio.ini
├── library.json
└── src/
    ├── LML.h              # single-include convenience header
    ├── LMLTypes.h         # LMLMessage, LMLPacket, LMLError, constants
    ├── LMLPacket.h        # CRC16, encode(), decode(), makeCtrl()
    ├── LMLTxQueue.h       # thread-safe fragmentation queue
    ├── LMLReassembly.h    # fragment → message reassembly (poll() only)
    ├── LMLP2PAsync.h
    └── LMLP2PAsync.cpp    # ISR + timer + state machine
```

---

## Tunables

Defined in `LMLTypes.h`:

| Constant | Default | Meaning |
|----------|---------|---------|
| `LML_MAX_MESSAGE` | 512 | Max assembled message payload |
| `LML_MAX_PAYLOAD` | 64 | Max bytes per LoRa packet |
| `LML_MAX_RETRIES` | 3 | RTS/DATA retry budget before HALT |
| `LML_ACK_TIMEOUT_MS` | 500 | Per-state one-shot deadline |
| `LML_LINK_TIMEOUT_MS` | 6000 | Reserved for link-state tracking |
| `LML_STATUS_TIMEOUT_MS` | 5000 | Reserved for link-state tracking |

---

## Dependencies

- [RadioLib](https://github.com/jgromes/RadioLib) `^7.0.0`
- ESP-IDF FreeRTOS (provided by the Arduino-ESP32 framework)

---

## Known limitations

- **Single in-flight exchange.** `queue_tx()` returns `QUEUE_FULL` while the
  state machine is not in `Standby`. Concurrent senders must retry.
- **No collision avoidance.** If both peers call `queue_tx()` at nearly the
  same time, both transmit RTS, neither sees a CTS, and both time out into
  HALT. Retries from the app will eventually deconflict via jitter, but the
  library does not back off internally today.
- **Single-instance ISR.** `P2PAsync` uses a static ISR trampoline and assumes
  one instance per process.
- **`LML_MAX_MESSAGE` is bounded by the 8-bit fragment bitmap** in
  `LMLReassembly` — at the current `LML_MAX_PAYLOAD = 64`, a 512-byte message
  is exactly 8 fragments. Raising the max message requires widening
  `_received`.
