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
        Wire.begin(sda, scl);
        Wire.setClock(freq);
    }

    struct Lock {
        Lock()  { if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY); }
        ~Lock() { if (mutex_) xSemaphoreGive(mutex_); }
        Lock(const Lock&) = delete;
        Lock& operator=(const Lock&) = delete;
    };
};
