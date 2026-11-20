#pragma once

#include "Config.hpp"
#include "Data.hpp"
#include "ISpeedSource.hpp"
#include <Arduino.h>
#include <esp_log.h>
#include <esp_timer.h>

// ── Датчик Холла для измерения скорости велосипеда ───────────
//
//  Магнит на колесе → датчик A3144 / LM393 → FALLING на HALL_PIN.
//  Скорость = окружность_колеса / dt_между_импульсами.
//  Гиперболическое затухание: если прошло больше dt — скорость плавно падает.
//  Тайм-аут 3 с — полная остановка (0 км/ч).

struct HallSpeed : public ISpeedSource {

    static inline volatile uint32_t hallPulseCount = 0;
    static inline volatile uint32_t hallLastUs     = 0;
    static inline volatile uint32_t hallDtUs       = 0;

    static void onHallPulse()
    {
        if (digitalRead(PinConfig::HALL_PIN) != LOW) return;

        uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
        uint32_t dt  = now - hallLastUs;

        if (dt > BikeConfig::HALL_DEBOUNCE_US) {
            if (hallLastUs != 0 && dt < BikeConfig::HALL_TIMEOUT_US) {
                hallDtUs = dt;
            } else {
                hallDtUs = 0;
            }
            hallLastUs = now;
            hallPulseCount = hallPulseCount + 1;
        }
    }

    bool begin() override
    {
        pinMode(PinConfig::HALL_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(PinConfig::HALL_PIN),
                        onHallPulse, FALLING);
        ESP_LOGI("HALL", "Hall sensor on GPIO %d, wheel %.3f m",
                 PinConfig::HALL_PIN, BikeConfig::WHEEL_CIRCUMFERENCE_M);
        return true;
    }

    SpeedData readSpeed() override
    {
        uint32_t cnt = 0, dtUs = 0, lastPulse = 0;
        noInterrupts();
        cnt       = hallPulseCount;
        dtUs      = hallDtUs;
        lastPulse = hallLastUs;
        interrupts();

        uint32_t nowUs     = static_cast<uint32_t>(esp_timer_get_time());
        uint32_t elapsedUs = (nowUs >= lastPulse) ? (nowUs - lastPulse) : 0;

        uint32_t effDtUs = dtUs;
        if (lastPulse == 0 || elapsedUs > BikeConfig::HALL_TIMEOUT_US) {
            effDtUs = 0;
        } else if (dtUs > 0 && elapsedUs > dtUs) {
            effDtUs = elapsedUs;
        }

        float speed_kmh = (effDtUs > 0)
            ? (BikeConfig::WHEEL_CIRCUMFERENCE_M
               / (static_cast<float>(effDtUs) * 1e-6f)) * 3.6f
            : 0.0f;

        SpeedData result;
        result.speed_kmh    = speed_kmh;
        result.timestamp_us = static_cast<uint64_t>(nowUs);
        result.valid        = true;
        return result;
    }
};
