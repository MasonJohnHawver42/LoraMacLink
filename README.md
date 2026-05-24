# MSRC — Master Slave Radio Control

A PlatformIO library implementing a reliable point-to-point messaging protocol over LoRa, built on top of [RadioLib](https://github.com/jgromes/RadioLib). Targets the Heltec WiFi LoRa 32 V3 (SX1262).

---

## Philosophy

The library owns the **protocol**, not the **radio** and not the **tasks**.

- **You** create and configure the RadioLib module
- **You** create FreeRTOS tasks, choose cores, stack sizes, and priorities
- **You** decide when and how often to drive the protocol by calling `poll()`

The library handles packet framing, CRC integrity, sequence numbers, ACK/NACK, retries, fragmentation/reassembly, and RTS/CTS flow control.

---

## Concepts

**Packet** — a single LoRa transmission. Internal to the library; the caller never sees raw packets.

**Message** — what the caller sends and receives. May be transparently fragmented across multiple packets (up to 512 bytes total). This is the only unit the caller works with.

**RTS / CTS** — protocol-level flow control exchanged over the air.
- The master calls `setRTS(true)` to signal it has a message to send.
- The slave calls `setCTS(true/false)` to signal its readiness.
- `poll()` manages the handshake automatically on both sides.

**Error handling** — every fallible API returns `MSRCError`. Possible values:

| Code | Meaning |
|------|---------|
| `OK` | Success |
| `QUEUE_FULL` | TX or RX ring buffer has no space |
| `NO_CTS` | `send()` called before CTS was granted |
| `BAD_MAGIC` | Received frame did not start with the expected magic byte |
| `BAD_CRC` | CRC16 mismatch — frame corrupted |
| `BAD_FRAG` | Fragment index out of range or mismatched total |
| `BAD_LENGTH` | Declared payload length exceeds buffer capacity |
| `TIMEOUT` | ACK or CTS not received within the retry deadline |
| `RADIO_ERR` | RadioLib returned a non-zero status code |
| `MSG_TOO_BIG` | Message exceeds `MSRC_MAX_MESSAGE` (512 bytes) |
| `EMPTY` | `read()` called on an empty buffer |

> **Note on error correction:** The library detects corruption via CRC16 and retries. It does not repair data. If your application needs forward error correction (e.g. Reed-Solomon), encode the payload before `send()` and decode it after `read()`.

---

## Packet Format

```
Byte   Field    Description
────   ─────    ───────────────────────────────────────────────
 0     magic    Always 0xAC
 1     type     MSRCPacketType (RTS, CTS, NACK, DATA, ACK, HB)
 2     seq      Sequence number (wraps 0–255)
 3     frag     Fragment index (0-based)
 4     total    Total fragments in this message (1 = unfragmented)
 5     len      Payload bytes that follow (0–64)
 6..N  payload  Application data
 N+1   crc_hi   CRC16-CCITT high byte  } over bytes 0..N
 N+2   crc_lo   CRC16-CCITT low byte   }
```

A NACK is sent for any rejection: the receiver being busy, bad magic, CRC mismatch, or a malformed header. The sender does not need to know which reason triggered it.

---

## Threading Model

`poll()` is **not** thread-safe with itself — it must be called from exactly one task that owns the radio. Everything else (`send`, `read`, `available`, `setRTS`, `setCTS`, `getCTS`, `getRTS`) is thread-safe and may be called from any task.

A typical two-core split on the ESP32:

```
Core 0                           Core 1
──────────────────────────       ──────────────────────────
protocol task                    app task
  while(true) msrc.poll(100);      msrc.setRTS / send / read
```

---

## API

### MSRCMaster

```cpp
MSRCMaster(SX1262 &radio, size_t rxDepth = 8);

void      init();
void      poll(uint32_t timeout_ms = 100);

void      setRTS(bool ready);       // signal intent to send
bool      getCTS() const;           // true once slave grants CTS
bool      waitCTS(uint32_t timeout_ms = 2000);  // blocks calling task

MSRCError send(const uint8_t *data, size_t len);
MSRCError read(MSRCMessage &out);
uint8_t   available() const;

bool      isConnected() const;
float     lastRSSI() const;
```

### MSRCSlave

```cpp
MSRCSlave(SX1262 &radio, size_t rxDepth = 4);

void      init();
void      poll(uint32_t timeout_ms = 100);

bool      getRTS() const;           // true when master has asserted RTS
void      setCTS(bool ready);       // grant or withhold permission to send

MSRCError read(MSRCMessage &out);
uint8_t   available() const;

MSRCError send(const uint8_t *data, size_t len);

bool      isConnected() const;
float     lastRSSI() const;
```

### MSRCMessage

```cpp
struct MSRCMessage {
    uint8_t  data[512];
    size_t   len;
    float    rssi;           // RSSI of the final received packet
    uint32_t timestamp_ms;   // millis() when fully assembled
};
```

---

## Usage

### Master

```cpp
#include <MSRC.h>

SX1262     radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
MSRCMaster msrc(radio);

void protocolTask(void *) {
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
    radio.begin(915.0, 125.0, 9, 7, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14, 8, 1.8);
    radio.setDio2AsRfSwitch(true);
    msrc.init();
    while (true) msrc.poll(100);
}

void appTask(void *) {
    msrc.setRTS(true);

    if (!msrc.waitCTS(2000)) {
        // slave not ready
        return;
    }

    uint8_t cmd[] = { 0x01, 0x02 };
    msrc.send(cmd, sizeof(cmd));

    MSRCMessage msg;
    while (msrc.read(msg) == MSRCError::OK) {
        // process msg.data / msg.len
    }
}

void setup() {
    xTaskCreatePinnedToCore(protocolTask, "msrc_m", 8192, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(appTask,      "app_m",  4096, NULL, 2, NULL, 1);
}
```

### Slave

```cpp
#include <MSRC.h>

SX1262    radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
MSRCSlave msrc(radio);

void protocolTask(void *) {
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
    radio.begin(915.0, 125.0, 9, 7, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14, 8, 1.8);
    radio.setDio2AsRfSwitch(true);
    msrc.init();
    msrc.setCTS(true);
    while (true) msrc.poll(100);
}

void appTask(void *) {
    MSRCMessage msg;
    if (msrc.read(msg) == MSRCError::OK) {
        msrc.setCTS(false);   // hold master off while processing

        // ... process msg.data ...

        uint8_t status[] = { 0xFF };
        msrc.send(status, sizeof(status));

        msrc.setCTS(true);
    }
}

void setup() {
    xTaskCreatePinnedToCore(protocolTask, "msrc_s", 8192, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(appTask,      "app_s",  4096, NULL, 2, NULL, 1);
}
```

---

## Heltec V3 Pin Mapping

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

## Project Structure

```
msrc-maclink/
├── platformio.ini
├── library.json
└── src/
    ├── MSRC.h              # single-include convenience header
    ├── MSRCTypes.h         # MSRCMessage, MSRCPacket, MSRCError, constants
    ├── MSRCPacket.h        # CRC16, encode(), decode(), makeControl()
    ├── MSRCRingBuffer.h    # thread-safe FreeRTOS ring buffer
    ├── MSRCTxQueue.h       # thread-safe TX fragmentation queue
    ├── MSRCReassembly.h    # fragment → message reassembly (poll() only)
    ├── MSRCMaster.h / .cpp
    └── MSRCSlave.h  / .cpp
```

---

## Dependencies

- [RadioLib](https://github.com/jgromes/RadioLib) `^7.0.0`
- ESP-IDF FreeRTOS (provided by the Arduino-ESP32 framework)

---

## Known Limitations & Open Questions

- **RTS timeout** — `setRTS(true)` currently has no deadline. A future constructor parameter could surface a timeout error after `poll()` exhausts retries.
- **TX queue depth** — only one message can be staged at a time. Concurrent `send()` calls from the app task return `QUEUE_FULL` until the in-flight message is delivered.
- **RX overflow policy** — when the ring buffer is full, new messages are silently dropped. A future option could expose a drop counter.
