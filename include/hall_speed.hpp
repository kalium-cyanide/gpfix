#pragma once

#include "config.hpp"
#include "data.hpp"
#include "speed_source.hpp"
#include <Arduino.h>
#include <esp_log.h>
#include <esp_timer.h>

static const char* HALL_TAG = "HallSpeed";

// Драйвер датчика Холла (A3144 / MH Sensor LM393) для измерения скорости велосипеда
struct HallSpeed : public ISpeedSource {
    static inline volatile uint32_t hallPulseCount = 0;
    static inline volatile uint32_t hallLastUs = 0;
    static inline volatile uint32_t hallDtUs = 0;

    // Прерывание по прохождению магнита
    static void onHallPulse()
    {
        // Проверяем активный уровень LOW на выходе D0 компаратора
        if (digitalRead(PinConfig::HALL_PIN) != LOW) return;

        uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
        uint32_t dt = now - hallLastUs;

        // Фильтр антидребезга: отсекаем дребезг контактов и ложные фронты при отведении магнита
        if (dt > BikeConfig::HALL_DEBOUNCE_US) {
            // Если первый импульс после стоянки (> 3 сек), только фиксируем время старта отсчёта
            if (hallLastUs != 0 && dt < BikeConfig::HALL_TIMEOUT_US) {
                hallDtUs = dt;
            } else {
                hallDtUs = 0;
            }
            hallLastUs = now;
            hallPulseCount++;
        }
    }

    bool begin() override
    {
        pinMode(PinConfig::HALL_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(PinConfig::HALL_PIN), onHallPulse, FALLING);
        ESP_LOGI(HALL_TAG, "Hall speed sensor initialized on GPIO %d, wheel: %.3f m",
                 PinConfig::HALL_PIN, BikeConfig::WHEEL_CIRCUMFERENCE_M);
        return true;
    }

    SpeedData readSpeed() override
    {
        // Атомарно забираем данные из прерывания
        uint32_t cnt = 0, dtUs = 0, lastPulse = 0;
        noInterrupts();
        cnt = hallPulseCount;
        dtUs = hallDtUs;
        lastPulse = hallLastUs;
        interrupts();

        uint32_t nowUs = static_cast<uint32_t>(esp_timer_get_time());

        // Защита от состояния гонки и переполнения таймера (underflow)
        uint32_t elapsedUs = 0;
        if (nowUs >= lastPulse) {
            elapsedUs = nowUs - lastPulse;
        }

        // Эффективный интервал с плавным гиперболическим затуханием скорости при замедлении
        uint32_t effDtUs = dtUs;
        if (lastPulse == 0 || elapsedUs > BikeConfig::HALL_TIMEOUT_US) {
            effDtUs = 0; // Колесо стоит более 3 секунд -> полная остановка
        } else if (dtUs > 0 && elapsedUs > dtUs) {
            effDtUs = elapsedUs; // Магнита еще нет, колесо замедляется -> v плавно снижается
        }

        float speed_kmh = (effDtUs > 0)
            ? (BikeConfig::WHEEL_CIRCUMFERENCE_M / (static_cast<float>(effDtUs) * 1e-6f)) * 3.6f
            : 0.0f;

        SpeedData result;
        result.speed_kmh = speed_kmh;
        result.timestamp_us = static_cast<uint64_t>(nowUs);
        result.valid = true;
        return result;
    }
};
