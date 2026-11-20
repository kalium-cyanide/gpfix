#pragma once

#include "Config.hpp"
#include <Arduino.h>

namespace StatusLed {

inline void init() { pinMode(PinConfig::LED_PIN, OUTPUT); }

inline void blink(int times, int ms = 200)
{
    for (int i = 0; i < times; i++) {
        digitalWrite(PinConfig::LED_PIN, HIGH);
        delay(ms);
        digitalWrite(PinConfig::LED_PIN, LOW);
        delay(ms);
    }
}

inline void on()  { digitalWrite(PinConfig::LED_PIN, HIGH); }
inline void off() { digitalWrite(PinConfig::LED_PIN, LOW);  }

} // namespace StatusLed
