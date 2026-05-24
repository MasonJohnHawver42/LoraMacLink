#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <MSRC.h>

// ── Heltec V3 pin mapping ──────────────────────────────────────────────────
#define LORA_NSS   8
#define LORA_DIO1  14
#define LORA_RST   12
#define LORA_BUSY  13
#define LORA_SCK   9
#define LORA_MISO  11
#define LORA_MOSI  10

SX1262   radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
MSRCSlave msrc(radio);

// ── Protocol task (Core 0) ─────────────────────────────────────────────────
void protocolTask(void *) {
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
    int state = radio.begin(915.0, 125.0, 9, 7,
                            RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14, 8, 1.8);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[slave] radio.begin failed: %d\n", state);
        vTaskDelete(NULL);
    }
    radio.setDio2AsRfSwitch(true);
    msrc.init();
    msrc.setCTS(true);   // ready to receive from the start

    while (true) {
        msrc.poll(100);
    }
}

// ── App task (Core 1) ──────────────────────────────────────────────────────
void appTask(void *) {
    while (true) {
        MSRCMessage msg;
        if (msrc.read(msg) == MSRCError::OK) {
            // Hold master off while processing
            msrc.setCTS(false);

            Serial.printf("[slave] rx %u bytes  rssi=%.1f\n", msg.len, msg.rssi);
            // ... process msg.data ...

            uint8_t status[] = { 0xFF };
            MSRCError err = msrc.send(status, sizeof(status));
            if (err != MSRCError::OK) {
                Serial.printf("[slave] send failed: %d\n", static_cast<int>(err));
            }

            msrc.setCTS(true);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    Serial.begin(115200);
    xTaskCreatePinnedToCore(protocolTask, "msrc_s", 8192, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(appTask,      "app_s",  4096, NULL, 2, NULL, 1);
}

void loop() { vTaskDelay(portMAX_DELAY); }
