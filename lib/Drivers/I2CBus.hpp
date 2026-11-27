#pragma once

#include "Config.hpp"
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ── Потокобезопасная обёртка над I2C-шиной ───────────────────
//
//  I2CBus::init()  — вызвать один раз в setup() до запуска задач
//  I2CBus::Lock    — RAII-замок, захватывает мьютекс на время
//                    жизни объекта

struct I2CBus {
    static inline SemaphoreHandle_t mutex_ = nullptr;

    static void init(int sda = PinConfig::SDA_PIN,
                     int scl = PinConfig::SCL_PIN,
                     uint32_t freq = 400000)
    {
        mutex_ = xSemaphoreCreateMutex();
#if defined(PIN_I2C_POWER)
        // Включение питания I2C / STEMMA QT на платах Adafruit Feather
        pinMode(PIN_I2C_POWER, OUTPUT);
        digitalWrite(PIN_I2C_POWER, HIGH);
        delay(10);
#endif
        Wire.begin(sda, scl);
        Wire.setClock(freq);
    }

    static void scan()
    {
        Lock lock;
        ESP_LOGI("I2C", "Scanning I2C bus (SDA=%d, SCL=%d)...", PinConfig::SDA_PIN, PinConfig::SCL_PIN);
        uint8_t count = 0;
        for (uint8_t addr = 1; addr < 127; addr++) {
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() == 0) {
                ESP_LOGI("I2C", " -> Found device at 0x%02X", addr);
                count++;
            }
        }
        if (count == 0) {
            ESP_LOGE("I2C", "No I2C devices responding! Check SDA=%d, SCL=%d, VCC, GND",
                     PinConfig::SDA_PIN, PinConfig::SCL_PIN);
        } else {
            ESP_LOGI("I2C", "Total devices responding: %d", count);
        }
    }

    struct Lock {
        Lock()  { if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY); }
        ~Lock() { if (mutex_) xSemaphoreGive(mutex_); }
        Lock(const Lock&) = delete;
        Lock& operator=(const Lock&) = delete;
    };
};
