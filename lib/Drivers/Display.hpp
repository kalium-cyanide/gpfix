#pragma once

#include "I2CBus.hpp"
#include <U8g2lib.h>
#include <esp_log.h>

// ── SSD1306 128×64 OLED (I2C, адрес 0x3C) ───────────────────

struct Display {
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2 { U8G2_R0, U8X8_PIN_NONE };
    bool available = false;

    bool begin()
    {
        {
            I2CBus::Lock lock;
            Wire.beginTransmission(0x3C);
            if (Wire.endTransmission() != 0) {
                ESP_LOGW("OLED", "SSD1306 not responding at 0x3C");
                return false;
            }
            u8g2.begin();
            u8g2.setFont(u8g2_font_6x10_tf);
        }
        available = true;
        ESP_LOGI("OLED", "SSD1306 128x64 initialized");
        return true;
    }

    void render(const char* l1, const char* l2, const char* l3,
                const char* l4, const char* l5, const char* l6)
    {
        if (!available) return;
        I2CBus::Lock lock;
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_6x10_tf);
        if (l1) u8g2.drawStr(0, 10, l1);
        if (l2) u8g2.drawStr(0, 20, l2);
        if (l3) u8g2.drawStr(0, 30, l3);
        if (l4) u8g2.drawStr(0, 40, l4);
        if (l5) u8g2.drawStr(0, 50, l5);
        if (l6) u8g2.drawStr(0, 60, l6);
        u8g2.sendBuffer();
    }
};
