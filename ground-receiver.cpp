#include <stdio.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico-lora/src/LoRa-RP2040.h"
#include "tusb.h"
#include "bsp/board.h"

#include "config.h"
#include "CanSat/src/commonTypes.h"
#include "CanSat/src/config.h"

// Global vars
const uint attempts = 3;
volatile bool received = false;
volatile char receivedBuf[RADIO_MAX_PACKET_SIZE + 1];

void initLoRa();
void putStr(const char* str, uint8_t printToItf = 0);
void putChar(const char c, uint8_t printToItf = 0);

// for (uint8_t i = 0; i < 3; i++) {
//     accel_g[i] = (float)accel_raw[i] / (16384.0f / 1);
//     gyro_dps[i] = (float)gyro_raw[i] / 131.0f;
//     mag_ut[i] = ((float)mag_raw[i] / 20) * 3;
// }

void onReceive(int packetSize) {
    // Read type
    // char type = LoRa.read();

    // if (type != 't') putchar(type);

    // Read packet
    received = true;

    for (int i = 0; i < packetSize; i++) {
        receivedBuf[i] = LoRa.read();
    }
    receivedBuf[packetSize] = '\0';

    // print RSSI of packet
    // printf("' with RSSI ");
    // printf("%d\n", LoRa.packetRssi());
}

int main() {
    stdio_init_all();
    board_init();

    // init device stack on configured roothub port
    tud_init(BOARD_TUD_RHPORT);
    // tusb_init();

    sleep_ms(2000);

    // Set pins and initialise LoRa
    LoRa.setPins(RADIO_NSS_PIN, RADIO_RESET_PIN, RADIO_DIO0_PIN);
    initLoRa();

    LoRa.onReceive(onReceive);
    LoRa.receive();

    while (true) {
        // putStr("Writing\n");
        // LoRa.beginPacket();
        // LoRa.write((const uint8_t*)"asdf", 4);
        // LoRa.endPacket();
        // LoRa.receive();
        tud_task();
        
        if (tud_cdc_n_available(0)) {
            uint8_t buf[RADIO_MAX_PACKET_SIZE]{};
            uint32_t count = tud_cdc_n_read(0, buf, sizeof(buf));

            LoRa.beginPacket();
            for (int i = 0; i < count; i++) {
                LoRa.write(buf[i]);
            }
            LoRa.endPacket(true);
        }

        if (received) {
            packet_t* packet = (packet_t*)receivedBuf;
            putStr((const char*)(packet->body), 0);
            puts("Asdf");
            received = false;
            LoRa.receive();
        }
    }
}

void initLoRa() {
    for (int i = 0; i < attempts; i++) {
        puts("Radio:      Trying to connect");

        int result = LoRa.begin(433000000);

        // Re-attempt if failed
        if (result != 1) {
            puts("Radio:      Failed to connect");
            sleep_ms(5000);

            if (i < attempts - 1)
                puts("Radio:      Retrying...");
            else {
                puts("Radio:      Failed to initialise");
                while (true);
            }

            continue;
        }

        puts("Radio:      Initialised successfully");
        LoRa.enableCrc();
        LoRa.setSignalBandwidth(RADIO_BANDWIDTH);
        LoRa.setSpreadingFactor(RADIO_SPREAD_FACTOR);

        return;
    }
}

void putStr(const char* str, uint8_t printToItf) {
    unsigned int len = strlen(str);

    for (int i = 0; i < len; i++) {
        if (str[i] == '\n') tud_cdc_n_write_char(printToItf, '\r');
        tud_cdc_n_write_char(printToItf, str[i]);
    }

    tud_cdc_n_write_flush(printToItf);
}

void putChar(const char c, uint8_t printToItf) {
    tud_cdc_n_write_char(printToItf, c);
    tud_cdc_n_write_flush(printToItf);
}