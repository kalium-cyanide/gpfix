#pragma once

#include "Config.hpp"
#include "I2CBus.hpp"
#include <U8g2lib.h>
#include <cmath>
#include <esp_log.h>

// ── SSD1306 128×64 OLED (I2C, адрес 0x3C или 0x3D) ──────────

struct Display {
    // В конструктор обязательно передаём SCL и SDA из PinConfig,
    // чтобы U8g2 не сбрасывал Wire.begin() на дефолтные пины платы (3 и 4)
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2 {
        U8G2_R0,
        U8X8_PIN_NONE,
        static_cast<uint8_t>(PinConfig::SCL_PIN),
        static_cast<uint8_t>(PinConfig::SDA_PIN)
    };

    bool available = false;
    uint8_t i2c_addr = 0x3C;

    bool begin()
    {
        I2CBus::Lock lock;

        // 1. Проверяем наличие экрана по адресам 0x3C и 0x3D
        uint8_t found = 0;
        Wire.beginTransmission(0x3C);
        if (Wire.endTransmission() == 0) {
            found = 0x3C;
        } else {
            Wire.beginTransmission(0x3D);
            if (Wire.endTransmission() == 0) {
                found = 0x3D;
            }
        }

        if (found == 0) {
            ESP_LOGW("OLED", "SSD1306 not found at 0x3C or 0x3D (SDA=%d, SCL=%d)",
                     PinConfig::SDA_PIN, PinConfig::SCL_PIN);
            // Сканируем шину I2C для диагностики
            uint8_t dev_count = 0;
            for (uint8_t a = 1; a < 127; a++) {
                Wire.beginTransmission(a);
                if (Wire.endTransmission() == 0) {
                    ESP_LOGI("OLED", " -> Found I2C device at 0x%02X", a);
                    dev_count++;
                }
            }
            if (dev_count == 0) {
                ESP_LOGE("OLED", "No I2C devices responding! Check wiring: SDA (pin %d), SCL (pin %d), VCC, GND",
                         PinConfig::SDA_PIN, PinConfig::SCL_PIN);
            }
            available = false;
            return false;
        }

        i2c_addr = found;
        u8g2.setI2CAddress(found << 1);

        u8g2.begin();
        u8g2.setBusClock(400000);
        Wire.setClock(400000); // Гарантируем частоту после инициализации U8g2

        u8g2.setPowerSave(0);   // Пробуждение экрана из спящего режима
        u8g2.setContrast(255);  // Максимальная яркость

        u8g2.setFont(u8g2_font_6x10_tf);

        // Приветственный экран
        u8g2.clearBuffer();
        u8g2.drawStr(0, 15, "GPFix Starting...");
        char buf[32];
        snprintf(buf, sizeof(buf), "OLED: OK (0x%02X)", i2c_addr);
        u8g2.drawStr(0, 35, buf);
        u8g2.sendBuffer();

        available = true;
        ESP_LOGI("OLED", "SSD1306 128x64 initialized at 0x%02X (SDA=%d, SCL=%d)",
                 i2c_addr, PinConfig::SDA_PIN, PinConfig::SCL_PIN);
        return true;
    }

    void render(const char* l1, const char* l2, const char* l3,
                const char* l4, const char* l5, const char* l6,
                bool show_arrow = false, float arrow_ux = 0.0f, float arrow_uy = -1.0f,
                const char* placeholder = "?")
    {
        if (!available) return;
        I2CBus::Lock lock;
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x10_tf);
        if (l1) u8g2.drawStr(0, 9,  l1);
        if (l2) u8g2.drawStr(0, 19, l2);
        if (l3) u8g2.drawStr(0, 29, l3);
        if (l4) u8g2.drawStr(0, 39, l4);
        if (l5) u8g2.drawStr(0, 49, l5);
        if (l6) u8g2.drawStr(0, 61, l6);

        constexpr int cx = 114, cy = 36, r = 12;
        u8g2.drawCircle(cx, cy, r);
        if (show_arrow) {
            float norm = std::hypot(arrow_ux, arrow_uy);
            if (norm > 1e-3f) {
                float ux = arrow_ux / norm;
                float uy = arrow_uy / norm;
                float px = -uy;
                float py =  ux;

                int x0 = static_cast<int>(std::lround(cx - ux * (r - 4)));
                int y0 = static_cast<int>(std::lround(cy - uy * (r - 4)));
                int x1 = static_cast<int>(std::lround(cx + ux * (r - 2)));
                int y1 = static_cast<int>(std::lround(cy + uy * (r - 2)));

                int bx1 = static_cast<int>(std::lround(x1 - ux * 5.0f + px * 3.5f));
                int by1 = static_cast<int>(std::lround(y1 - uy * 5.0f + py * 3.5f));
                int bx2 = static_cast<int>(std::lround(x1 - ux * 5.0f - px * 3.5f));
                int by2 = static_cast<int>(std::lround(y1 - uy * 5.0f - py * 3.5f));

                u8g2.drawLine(x0, y0, x1, y1);
                u8g2.drawTriangle(x1, y1, bx1, by1, bx2, by2);
            }
        } else {
            int pw = u8g2.getStrWidth(placeholder ? placeholder : "?");
            u8g2.drawStr(cx - pw / 2, cy + 3, placeholder ? placeholder : "?");
        }

        u8g2.sendBuffer();
    }
};
