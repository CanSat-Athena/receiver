#include <stdio.h>
#include "pico/stdlib.h"
#include "pico-lora/src/LoRa-RP2040.h"

#include "config.h"

// Global vars
const uint attempts = 3;

void initLoRa();

int main() {

    stdio_init_all();

    sleep_ms(1000);
    
    puts("Hello, world!");

    // Set pins and initialise LoRa
    LoRa.setPins(RADIO_NSS_PIN, RADIO_RESET_PIN, RADIO_DIO0_PIN);
    initLoRa();

    int counter = 0;

    while (true) {
        int packetSize = LoRa.parsePacket();
        if (packetSize) {
            // received a packet
            printf("Received packet \n'");

            // read packet
            while (LoRa.available()) {
                putchar((char)LoRa.read());
            }

            // print RSSI of packet
            printf("' with RSSI \n");
            printf((char*)LoRa.packetRssi(), "\n");
        }
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
        return;
    }
}