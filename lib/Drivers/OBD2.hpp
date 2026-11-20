#pragma once

#include "Config.hpp"
#include "Data.hpp"
#include "ISpeedSource.hpp"
#include <driver/twai.h>
#include <esp_log.h>
#include <esp_timer.h>

// ── OBD2 через TWAI (CAN) — датчик скорости автомобиля ──────

struct OBD2 : public ISpeedSource {

    bool initialized = false;

    static constexpr uint32_t REQUEST_ID     = 0x7DF;
    static constexpr uint32_t RESPONSE_BASE  = 0x7E8;
    static constexpr uint32_t RESPONSE_END   = 0x7EF;
    static constexpr uint8_t  SERVICE_CURRENT_DATA    = 0x01;
    static constexpr uint8_t  SERVICE_RESPONSE_OFFSET = 0x40;

    bool begin() override
    {
        twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
            static_cast<gpio_num_t>(PinConfig::CAN_TX_PIN),
            static_cast<gpio_num_t>(PinConfig::CAN_RX_PIN),
            TWAI_MODE_NORMAL);
        g.tx_queue_len = 5;
        g.rx_queue_len = 10;

        twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
        twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        if (twai_driver_install(&g, &t, &f) != ESP_OK) {
            ESP_LOGE("OBD2", "Driver install failed");
            return false;
        }
        if (twai_start() != ESP_OK) {
            ESP_LOGE("OBD2", "Start failed");
            twai_driver_uninstall();
            return false;
        }

        initialized = true;
        ESP_LOGI("OBD2", "TWAI on TX=%d RX=%d @ 500kbps",
                 PinConfig::CAN_TX_PIN, PinConfig::CAN_RX_PIN);
        return true;
    }

    SpeedData readSpeed() override
    {
        SpeedData result {};
        result.timestamp_us = static_cast<uint64_t>(esp_timer_get_time());
        result.valid = false;

        if (!sendPID(CANConfig::SPEED_PID)) return result;

        uint8_t data[5];
        uint8_t len = 0;
        if (receiveResponse(CANConfig::SPEED_PID, data, len) && len >= 1) {
            result.speed_kmh = static_cast<float>(data[0]);
            result.valid = true;
            result.timestamp_us = static_cast<uint64_t>(esp_timer_get_time());
        }
        return result;
    }

    float readRPM()
    {
        if (!sendPID(CANConfig::RPM_PID)) return -1.0f;

        uint8_t data[5];
        uint8_t len = 0;
        if (receiveResponse(CANConfig::RPM_PID, data, len) && len >= 2) {
            return (static_cast<float>(data[0]) * 256.0f
                  + static_cast<float>(data[1])) / 4.0f;
        }
        return -1.0f;
    }

    void stop()
    {
        if (initialized) {
            twai_stop();
            twai_driver_uninstall();
            initialized = false;
            ESP_LOGI("OBD2", "TWAI stopped");
        }
    }

    ~OBD2() { stop(); }

private:
    bool sendPID(uint8_t pid)
    {
        if (!initialized) return false;

        twai_message_t msg {};
        msg.identifier = REQUEST_ID;
        msg.data_length_code = 8;
        msg.data[0] = 0x02;
        msg.data[1] = SERVICE_CURRENT_DATA;
        msg.data[2] = pid;
        for (int i = 3; i < 8; i++) msg.data[i] = 0x55;

        return twai_transmit(&msg, pdMS_TO_TICKS(CANConfig::TX_TIMEOUT_MS)) == ESP_OK;
    }

    bool receiveResponse(uint8_t pid, uint8_t* out, uint8_t& out_len,
                         uint32_t timeout_ms = CANConfig::RX_TIMEOUT_MS)
    {
        if (!initialized) return false;

        twai_message_t rx {};
        uint64_t deadline = esp_timer_get_time() + timeout_ms * 1000ULL;

        while (esp_timer_get_time() < deadline) {
            if (twai_receive(&rx, pdMS_TO_TICKS(10)) != ESP_OK) continue;
            if (rx.identifier < RESPONSE_BASE || rx.identifier > RESPONSE_END) continue;
            if (rx.data[1] != (SERVICE_CURRENT_DATA + SERVICE_RESPONSE_OFFSET)) continue;
            if (rx.data[2] != pid) continue;

            out_len = rx.data[0] - 2;
            if (out_len > 5) out_len = 5;
            for (uint8_t i = 0; i < out_len; i++) out[i] = rx.data[3 + i];
            return true;
        }
        return false;
    }
};
