#pragma once

#include <cstdint>

#define config__ static constexpr

struct PinConfig {
    config__ int32_t SDA_PIN = 8;
    config__ int32_t SCL_PIN = 9;

    config__ int32_t CAN_TX_PIN = 5;
    config__ int32_t CAN_RX_PIN = 4;

    config__ int32_t BTN1_PIN = 6;
    config__ int32_t HALL_PIN = 6;
    config__ int32_t BTN2_PIN = 7;

    config__ int32_t SD_MOSI = 11;
    config__ int32_t SD_MISO = 13;
    config__ int32_t SD_SCK  = 12;
    config__ int32_t SD_CS   = 10;

    config__ int32_t LED_PIN = 2;
};

struct IMUConfig {
    config__ uint8_t  BMI160_ADDR     = 0x68;
    config__ uint32_t SAMPLE_RATE_HZ  = 200;

    config__ uint8_t ACC_CONF  = 0x29;
    config__ uint8_t ACC_RANGE = 0x05;

    config__ uint8_t GYR_CONF  = 0x29;
    config__ uint8_t GYR_RANGE = 0x02;

    config__ float ACCEL_SCALE = 9.81f / 8192.0f;
    config__ float GYRO_SCALE  = 1.0f / 65.6f;

    config__ uint32_t CALIBRATION_SAMPLES = 500;
};

struct BTConfig {
    config__ const char* DEVICE_NAME    = "GPFix";
    config__ const char* SERVICE_UUID   = "0000aaaa-0000-1000-8000-00805f9b34fb";
    config__ const char* DELTA_CHAR_UUID = "0000aaab-0000-1000-8000-00805f9b34fb";
    config__ const char* ACK_CHAR_UUID  = "0000aaac-0000-1000-8000-00805f9b34fb";
    config__ uint16_t NOTIFY_INTERVAL_MS = 50;
    config__ uint32_t FREQUENCY          = 20;
    config__ uint16_t MTU_SIZE           = 185;
};

struct CANConfig {
    config__ uint32_t BAUD_RATE       = 500000;
    config__ uint16_t SPEED_PID       = 0x0D;
    config__ uint16_t RPM_PID         = 0x0C;
    config__ uint32_t POLL_INTERVAL_MS = 50;
    config__ uint32_t TX_TIMEOUT_MS   = 100;
    config__ uint32_t RX_TIMEOUT_MS   = 100;
};

struct BikeConfig {
    config__ bool        USE_BLE_SPEED         = true;               // true = CYCPLUS C3 (BLE CSC), false = HallSpeed
    config__ const char* CSC_DEVICE_NAME       = "CYCPLUS C3 15481";
    config__ float       WHEEL_CIRCUMFERENCE_M = 2.150f;
    config__ uint32_t    HALL_TIMEOUT_US       = 3000000UL;
    config__ uint32_t    HALL_DEBOUNCE_US      = 60000UL;
    config__ uint32_t    POLL_INTERVAL_MS      = 50;
};

struct SDConfig {
    config__ bool     ENABLED        = false;  // false, если модуль SD-карты не распаян
    config__ uint32_t SPI_FREQ       = 20000000;
    config__ uint32_t QUEUE_SIZE     = 512;
    config__ uint32_t WRITE_BATCH    = 32;
    config__ uint32_t FLUSH_INTERVAL = 200;
    config__ uint32_t WRITER_STACK   = 6144;
    config__ uint32_t WRITER_PRIO    = 1;
    config__ uint8_t  WRITER_CORE    = 0;
    config__ uint64_t MIN_FREE_BYTES = 1024ULL * 1024;
};

struct TaskConfig {
    config__ uint8_t  IMU_CORE  = 1;
    config__ uint8_t  BLE_CORE  = 0;
    config__ uint8_t  CAN_CORE  = 0;
    config__ uint32_t IMU_STACK = 16384;
    config__ uint32_t BLE_STACK = 4096;
    config__ uint32_t CAN_STACK = 6144;
    config__ uint32_t IMU_PRIO  = 10;
    config__ uint32_t BLE_PRIO  = 2;
    config__ uint32_t CAN_PRIO  = 3;
};
