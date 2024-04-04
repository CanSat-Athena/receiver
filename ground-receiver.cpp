#include <stdio.h>
#include <ctype.h>
#include <stdarg.h>
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
    int rssi;
} packetGround_t;

// Global vars
const uint attempts = 3;
queue_t receiveQueue;

void initLoRa();
void putStr(const char* str, uint8_t printToItf = 0);
void putnStr(const char* str, unsigned int n, uint8_t printToItf = 0);
void putChar(const char c, uint8_t printToItf = 0);
void usbPrintf(uint8_t printToItf, const char* str, ...);
void handleDataPacket(packetGround_t body);

// for (uint8_t i = 0; i < 3; i++) {
//     accel_g[i] = (float)accel_raw[i] / (16384.0f / 1);
//     gyro_dps[i] = (float)gyro_raw[i] / 131.0f;
//     mag_ut[i] = ((float)mag_raw[i] / 20) * 3;
// }

void onReceive(int packetSize) {
    // Read packet
    packetGround_t receivedPacket{};
    // char* packet = (char*)(&(receivedPacket.packet));

    receivedPacket.packet.type = LoRa.read();
    receivedPacket.packet.size = LoRa.read();

    for (int i = 0; i < packetSize - (sizeof(receivedPacket.packet.type) + sizeof(receivedPacket.packet.size)); i++) {
        receivedPacket.packet.body[i] = LoRa.read();
    }
    // receivedPacket.packet.body[packetSize - 1] = '\0'; // -1 because packet type

    receivedPacket.rssi = LoRa.packetRssi();

    queue_try_add(&receiveQueue, (const void*)&receivedPacket);
}

int main() {
    stdio_init_all();
    board_init();

    // init device stack on configured roothub port
    tud_init(BOARD_TUD_RHPORT);
    queue_init(&receiveQueue, sizeof(packetGround_t), 10);

    sleep_ms(2000);

    // Set pins and initialise LoRa
    LoRa.setPins(RADIO_NSS_PIN, RADIO_RESET_PIN, RADIO_DIO0_PIN);
    initLoRa();

    LoRa.onReceive(onReceive);
    LoRa.receive();

    // packetGround_t packet{};

    while (true) {
        // putStr("Writing\n");
        // LoRa.beginPacket();
        // LoRa.write((const uint8_t*)"a", 4);
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

        packetGround_t packet{};
        if (queue_try_remove(&receiveQueue, &packet)) {
            if (packet.packet.type == 't') {
                putnStr((const char*)packet.packet.body, packet.packet.size, 0);
                tud_task();
            } else if (packet.packet.type == 'd')
                handleDataPacket(packet);    //putnStr((const char*)packet.packet.body, packet.packet.size, 1);
        }
    }
}

void handleDataPacket(packetGround_t packet) {
    // dataRadioLine_t* pReceivedLine = (dataRadioLine_t*)packet.body;
    dataRadioLine_t& receivedLine = (dataRadioLine_t&)packet.packet.body;
    // int port = packet.type == 't' ? 

    usbPrintf(1, "%u: ", receivedLine.timestamp);

    // Get DHT20 data
    usbPrintf(1, "[");
    for (int i = 0; i < DHT20_READ_FREQ; i++) {
        usbPrintf(1, "[%f,%f]", receivedLine.dht20[i].temperature, receivedLine.dht20[i].humidity);
    }
    usbPrintf(1, "]");
    tud_task();

    // // Get BME680 data
    // usbPrintf(1, "[");
    // for (int i = 0; i < BME680_READ_FREQ; i++) {
    //     usbPrintf(1, "[%f,%f,%f,%f]",
    //         receivedLine.bme680[i].temperature,
    //         receivedLine.bme680[i].humidity,
    //         receivedLine.bme680[i].pressure,
    //         receivedLine.bme680[i].gasResistance
    //     );
    // }
    // usbPrintf(1, "]");
    // tud_task();

    // // Get IMU data
    // usbPrintf(1, "[");
    // for (int i = 0; i < IMU_READ_FREQ; i++) {
    //     usbPrintf(1, "[%d,%d,%d,%d,%d,%d,%d,%d,%d]",
    //         receivedLine.imu[i].accel[0], receivedLine.imu[i].accel[1], receivedLine.imu[i].accel[2],
    //         receivedLine.imu[i].gyro[0], receivedLine.imu[i].gyro[1], receivedLine.imu[i].gyro[2],
    //         receivedLine.imu[i].mag[0], receivedLine.imu[i].mag[1], receivedLine.imu[i].mag[2]
    //     );
    //     tud_task();
    // }
    // usbPrintf(1, "]");

    // // Get light data
    // usbPrintf(1, "[");
    // for (int i = 0; i < LIGHT_READ_FREQ; i++) {
    //     usbPrintf(1, "%u,", receivedLine.lightData[i].lightIntensity);
    // }
    // usbPrintf(1, "]");
    // tud_task();

    // // Get anemometer data
    // usbPrintf(1, "[");
    // for (int i = 0; i < ANEMOMETER_READ_FREQ; i++) {
    //     usbPrintf(1, "%u,", receivedLine.anemometerData[i].triggerCount);
    // }
    // usbPrintf(1, "]");
    // tud_task();

    // Get GPS data
    usbPrintf(1, "[");
    usbPrintf(1, "%f,%f,%f,%d,%d,%d,%u", receivedLine.gpsData[0].latitude, receivedLine.gpsData[0].longitude, receivedLine.gpsData[0].altitude,
        receivedLine.gpsData[0].hours, receivedLine.gpsData[0].minutes, receivedLine.gpsData[0].seconds, receivedLine.gpsData[0].fix);
    usbPrintf(1, "]");
    tud_task();

    // Battery
    usbPrintf(1, "[");
    // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    const float conversionFactor = (3 * 3.3f) / (1 << 12);
    float voltage = receivedLine.batteryLevel * conversionFactor;
    float percentage = 100 * ((voltage - BATTERY_EMPTY_VOLTAGE) / (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE));
    usbPrintf(1, "%f,%f", voltage, percentage);
    usbPrintf(1, "]");
    tud_task();

    // Filesystem
    usbPrintf(1, "[");
    usbPrintf(1, "%d,%d", receivedLine.fsSize * (1u << 12), FS_SIZE);   // Block size is 4096
    usbPrintf(1, "]");
    tud_task();

    // Get RSSI
    usbPrintf(1, "[");
    usbPrintf(1, "%d", packet.rssi);
    usbPrintf(1, "]");
    tud_task();

    usbPrintf(1, "\n");
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
        LoRa.setCodingRate4(RADIO_CODING_RATE);
        LoRa.setTxPower(RADIO_TX_POWER);

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

void usbPrintf(uint8_t printToItf, const char* str, ...) {
    va_list args;
    va_start(args, str);     // Very important

    char buffer[TERMINAL_BUFFER_SIZE];
    vsnprintf(buffer, TERMINAL_BUFFER_SIZE - 1, str, args);  // -1 just to be safe
    buffer[TERMINAL_BUFFER_SIZE - 1] = '\0';                    // Prevent memory leak, just in case

    putStr(buffer, printToItf);

    va_end(args);
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