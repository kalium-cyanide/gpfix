#pragma once

#include <cstdint>

#define config__ static constexpr

struct PinConfig {
    config__ int32_t SDA_PIN = 21;
    config__ int32_t SCL_PIN = 22;
    config__ int32_t CAN_TX_PIN = 4;
    config__ int32_t CAN_RX_PIN = 5;
    config__ int32_t LED_PIN = 13;
};

struct IMUConfig {
    config__ uint8_t MPU_ADDR = 0x68;
    config__ uint32_t SAMPLE_RATE_HZ = 200;
    config__ uint8_t ACCEL_RANGE = 2;
    config__ uint8_t GYRO_RANGE = 1;
    config__ float ACCEL_SCALE = 9.81f / 16384.0f;
    config__ float GYRO_SCALE = 1.0f / 65.5f;
    config__ uint32_t CALIBRATION_SAMPLES = 500;
};

struct BTConfig {
    config__ const char* DEVICE_NAME = "GPFix";
    config__ const char* SERVICE_UUID = "";
    config__ const char* DELTA_CHAR_UUID = "";
    config__ const char* ACK_CHAR_UUID = "";
    config__ uint16_t NOTIFY_INTERVAL_MS = 50;
    config__ uint32_t FREQUENCY = 20;
    config__ uint16_t MTU_SIZE = 128;
};

struct CANConfig {
    config__ uint32_t BAUD_RATE = 500000;
    config__ uint16_t SPEED_PID = 0x0D;
    config__ uint16_t RPM_PID = 0x0C;
};

struct TaskConfig {
    config__ uint8_t IMU_CORE = 1;
    config__ uint8_t BLE_CORE = 0;
    config__ uint32_t IMU_STACK = 4096;
    config__ uint32_t BLE_STACK = 4096;
    config__ uint32_t IMU_PRIO = 10;
    config__ uint32_t BLE_PRIO = 2;
};