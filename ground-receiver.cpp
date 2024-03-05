#include <stdio.h>
#include "pico/stdlib.h"
#include "pico-lora/src/LoRa-RP2040.h"

#include "config.h"
#include "CanSat/src/commonTypes.h"
#include "CanSat/src/config.h"

// Global vars
const uint attempts = 3;

void initLoRa();

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
    for (int i = 0; i < packetSize; i++) {
        char c = LoRa.read();
        if (i > 0)
            printf("%c", c);
    }

    // print RSSI of packet
    // printf("' with RSSI ");
    // printf("%d\n", LoRa.packetRssi());
}

int main() {
    stdio_init_all();

    sleep_ms(1000);

    puts("Hello, world!");

    // Set pins and initialise LoRa
    LoRa.setPins(RADIO_NSS_PIN, RADIO_RESET_PIN, RADIO_DIO0_PIN);
    initLoRa();

    LoRa.onReceive(onReceive);
    LoRa.receive();
    while (true) {
        sleep_ms(5000);
        printf("Writing\n");
        LoRa.beginPacket();
        LoRa.write((const uint8_t*)"asdf", 4);
        LoRa.endPacket();
        LoRa.receive();
    }
}

void initLoRa() {
    for (int i = 0; i < attempts; i++) {
        printf("Radio:      Trying to connect, attempt %d of %d\n", i + 1, attempts);

        int result = LoRa.begin(433000000);

        // Re-attempt if failed
        if (result != 1) {
            printf("Radio:      Failed to connect\n");
            sleep_ms(2000);

            if (i < attempts - 1)
                printf("Radio:      Retrying...\n");
            else {
                printf("Radio:      Failed to initialise\n");
                while (true);
            }

            continue;
        }

        printf("Radio:      Initialised successfully\n");
        LoRa.enableCrc();
        LoRa.setSignalBandwidth(RADIO_BANDWIDTH);
        LoRa.setSpreadingFactor(RADIO_SPREAD_FACTOR);

        return;
    }
}