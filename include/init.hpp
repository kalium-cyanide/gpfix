#pragma once

#include "MPU6050.h"
#include "config.hpp"
#include <Arduino.h>
#include <Wire.h>

inline MPU6050 initMPU()
{
    Wire.begin(PinConfig::SDA_PIN, PinConfig::SCL_PIN);
    Wire.setClock(400000);

    MPU6050 mpu(IMUConfig::MPU_ADDR);
    mpu.initialize();
    mpu.setFullScaleAccelRange(IMUConfig::ACCEL_RANGE);
    mpu.setFullScaleGyroRange(IMUConfig::GYRO_RANGE);

    uint8_t div = (1000 / IMUConfig::SAMPLE_RATE_HZ) - 1;
    mpu.setRate(div);
    mpu.setDLPFMode(MPU6050_DLPF_BW_42);

    return mpu;
}

namespace Status {
inline void init()
{
    pinMode(PinConfig::LED_PIN, OUTPUT);
}

inline void blink(int times, int ms = 200)
{
    for (int i = 0; i < times; i++) {
        digitalWrite(PinConfig::LED_PIN, HIGH);
        delay(ms);
        digitalWrite(PinConfig::LED_PIN, LOW);
        delay(ms);
    }
}

inline void on()
{
    digitalWrite(PinConfig::LED_PIN, HIGH);
}

inline void off()
{
    digitalWrite(PinConfig::LED_PIN, LOW);
}
}
