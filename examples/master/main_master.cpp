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

SX1262    radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
MSRCMaster msrc(radio);

// ── Protocol task (Core 0) ─────────────────────────────────────────────────
void protocolTask(void *) {
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
    int state = radio.begin(915.0, 125.0, 9, 7,
                            RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14, 8, 1.8);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[master] radio.begin failed: %d\n", state);
        vTaskDelete(NULL);
    }
    radio.setDio2AsRfSwitch(true);
    msrc.init();

    while (true) {
        msrc.poll(100);
    }
}

// ── App task (Core 1) ──────────────────────────────────────────────────────
void appTask(void *) {
    vTaskDelay(pdMS_TO_TICKS(2000));   // wait for radio init

    while (true) {
        msrc.setRTS(true);

        if (!msrc.waitCTS(2000)) {
            Serial.println("[master] CTS timeout — slave not ready");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        uint8_t cmd[] = { 0x01, 0x02 };
        MSRCError err = msrc.send(cmd, sizeof(cmd));
        if (err != MSRCError::OK) {
            Serial.printf("[master] send failed: %d\n", static_cast<int>(err));
        }

        // Drain any inbound messages from the slave
        MSRCMessage msg;
        while (msrc.read(msg) == MSRCError::OK) {
            Serial.printf("[master] rx %u bytes  rssi=%.1f\n", msg.len, msg.rssi);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void setup() {
    Serial.begin(115200);
    xTaskCreatePinnedToCore(protocolTask, "msrc_m", 8192, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(appTask,      "app_m",  4096, NULL, 2, NULL, 1);
}

void loop() { vTaskDelay(portMAX_DELAY); }
