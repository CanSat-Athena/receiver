#include <stdio.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico-lora/src/LoRa-RP2040.h"
#include "tusb.h"
#include "bsp/board.h"
#include "pico/util/queue.h"

#include "config.h"
#include "CanSat/src/commonTypes.h"
#include "CanSat/src/config.h"

typedef struct packetGround_t {
    packet_t packet;
    int bodySize;
    int rssi;
} packetGround_t;

// Global vars
const uint attempts = 3;
packetGround_t receivedPacket;
queue_t receiveQueue;

void initLoRa();
void putStr(const char* str, uint8_t printToItf = 0);
void putnStr(const char* str, unsigned int n, uint8_t printToItf = 0);
void putChar(const char c, uint8_t printToItf = 0);
void handleDataPacket(packet_t body);

// for (uint8_t i = 0; i < 3; i++) {
//     accel_g[i] = (float)accel_raw[i] / (16384.0f / 1);
//     gyro_dps[i] = (float)gyro_raw[i] / 131.0f;
//     mag_ut[i] = ((float)mag_raw[i] / 20) * 3;
// }

void onReceive(int packetSize) {
    // Read packet
    char* packet = (char*)(&(receivedPacket.packet));

    for (int i = 0; i < packetSize; i++) {
        packet[i] = LoRa.read();
    }
    // receivedPacket.packet.body[packetSize - 1] = '\0'; // -1 because packet type

    receivedPacket.bodySize = packetSize - 1;
    receivedPacket.rssi = LoRa.packetRssi();

    queue_try_add(&receiveQueue, &receivedPacket);
}

int main() {
    stdio_init_all();
    board_init();

    // init device stack on configured roothub port
    tud_init(BOARD_TUD_RHPORT);
    queue_init(&receiveQueue, sizeof(packetGround_t), 10);
    // tusb_init();

    sleep_ms(2000);

    // Set pins and initialise LoRa
    LoRa.setPins(RADIO_NSS_PIN, RADIO_RESET_PIN, RADIO_DIO0_PIN);
    initLoRa();

    LoRa.onReceive(onReceive);
    LoRa.receive();

    packetGround_t packet;

    while (true) {
        // putStr("Writing\n");
        // LoRa.beginPacket();
        // LoRa.write((const uint8_t*)"asdf", 4);
        // LoRa.endPacket();
        // LoRa.receive();
        tud_task();

        if (tud_cdc_n_available(0)) {
            if (LoRa.beginPacket() == 0) {
                continue;
            }

            uint8_t buf[RADIO_MAX_PACKET_SIZE]{};
            uint32_t count = tud_cdc_n_read(0, buf, sizeof(buf));

            LoRa.beginPacket();
            for (int i = 0; i < count; i++) {
                LoRa.write(buf[i]);
            }
            LoRa.endPacket(true);
            LoRa.receive();
        }

        if (queue_try_remove(&receiveQueue, &packet)) {
            if (packet.packet.type == 't')
                putnStr((const char*)packet.packet.body, packet.bodySize, 0);
            else if (packet.packet.type == 'd')
                putStr("Asdf");
            LoRa.receive();
        }
    }
}

void handleDataPacket(packet_t body) {
    
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

void putnStr(const char* str, unsigned int n, uint8_t printToItf) {
    unsigned int len = n;

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